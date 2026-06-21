#include "multi_explore_mapping/icp.hpp"
#include <limits>
#include <cmath>

namespace icp {

namespace {

// target 点群を一様グリッドに登録し、近傍探索を O(1) 近傍セルのみに絞る。
// cell_size = max_correspondence_distance にすると、3x3 セル探索だけで
// 距離 <= max_correspondence_distance の最近傍を漏れなく見つけられる
// (1次元で考えると、セル内のどの位置からも距離D以内の点は隣接セルまでに必ず収まるため)。
struct SpatialGrid {
    float cell_size;
    float min_x, min_y;
    int cols, rows;
    std::vector<std::vector<int>> buckets;

    int clamped_cell(float x, float y) const {
        int cx = std::max(0, std::min(cols - 1,
            static_cast<int>(std::floor((x - min_x) / cell_size))));
        int cy = std::max(0, std::min(rows - 1,
            static_cast<int>(std::floor((y - min_y) / cell_size))));
        return cy * cols + cx;
    }
};

SpatialGrid build_grid(const Eigen::MatrixXf& target, float cell_size) {
    SpatialGrid grid;
    grid.cell_size = cell_size;
    grid.min_x = target.col(0).minCoeff();
    grid.min_y = target.col(1).minCoeff();
    const float max_x = target.col(0).maxCoeff();
    const float max_y = target.col(1).maxCoeff();
    grid.cols = std::max(1, static_cast<int>((max_x - grid.min_x) / cell_size) + 1);
    grid.rows = std::max(1, static_cast<int>((max_y - grid.min_y) / cell_size) + 1);
    grid.buckets.resize(static_cast<size_t>(grid.cols) * grid.rows);

    for (int j = 0; j < target.rows(); ++j) {
        grid.buckets[grid.clamped_cell(target(j, 0), target(j, 1))].push_back(j);
    }
    return grid;
}

}  // namespace

ICP::ICP(int max_iterations, float convergence_threshold,
         float max_correspondence_distance)
    : max_iterations_(max_iterations),
      convergence_threshold_(convergence_threshold),
      max_correspondence_distance_(max_correspondence_distance) {}

ICPResult ICP::align(const Eigen::MatrixXf& source_input,
                     const Eigen::MatrixXf& target) {
    Eigen::MatrixXf source = source_input;

    Eigen::Matrix2f R = Eigen::Matrix2f::Identity();
    Eigen::Vector2f t = Eigen::Vector2f::Zero();

    ICPResult result;
    result.iterations = 0;
    result.valid_correspondences = 0;
    result.R = R;
    result.t = t;
    result.final_error = std::numeric_limits<float>::max();

    for (int iter = 0; iter < max_iterations_; ++iter) {
        std::vector<int> indices = find_closest_points(source, target);

        int valid = 0;
        for (int idx : indices) if (idx != -1) ++valid;
        result.valid_correspondences = valid;

        if (valid < 5) break;

        Eigen::Matrix2f R_delta;
        Eigen::Vector2f t_delta;
        compute_transform(source, target, indices, R_delta, t_delta);

        Eigen::MatrixXf source_new =
            (R_delta * source.transpose()).colwise() + t_delta;
        source_new.transposeInPlace();

        R = R_delta * R;
        t = R_delta * t + t_delta;

        float error = compute_error(source_new, target, indices);

        result.iterations = iter + 1;
        result.final_error = error;

        if (error < convergence_threshold_) break;

        source = source_new;
    }

    result.R = R;
    result.t = t;
    return result;
}

std::vector<int> ICP::find_closest_points(const Eigen::MatrixXf& source,
                                           const Eigen::MatrixXf& target) {
    std::vector<int> indices(source.rows(), -1);
    if (target.rows() == 0) return indices;

    // max_correspondence_distance_ が事実上無制限の場合はグリッド分割が無意味
    // (セルサイズが定義できない) ため全探索にフォールバック
    const bool bounded =
        max_correspondence_distance_ < std::numeric_limits<float>::max() / 2.0f;

    if (!bounded) {
        for (int i = 0; i < source.rows(); ++i) {
            Eigen::Vector2f sp = source.row(i);
            float min_dist = std::numeric_limits<float>::max();
            int best = -1;
            for (int j = 0; j < target.rows(); ++j) {
                float d = (sp - target.row(j).transpose()).norm();
                if (d < min_dist) { min_dist = d; best = j; }
            }
            indices[i] = best;
        }
        return indices;
    }

    const SpatialGrid grid = build_grid(target, max_correspondence_distance_);

    for (int i = 0; i < source.rows(); ++i) {
        const float sx = source(i, 0), sy = source(i, 1);
        const int cx = static_cast<int>(std::floor((sx - grid.min_x) / grid.cell_size));
        const int cy = static_cast<int>(std::floor((sy - grid.min_y) / grid.cell_size));

        float min_dist = std::numeric_limits<float>::max();
        int best = -1;

        for (int dy = -1; dy <= 1; ++dy) {
            const int ncy = cy + dy;
            if (ncy < 0 || ncy >= grid.rows) continue;
            for (int dx = -1; dx <= 1; ++dx) {
                const int ncx = cx + dx;
                if (ncx < 0 || ncx >= grid.cols) continue;
                for (int j : grid.buckets[ncy * grid.cols + ncx]) {
                    const float d = std::hypot(sx - target(j, 0), sy - target(j, 1));
                    if (d < min_dist) { min_dist = d; best = j; }
                }
            }
        }

        if (best != -1 && min_dist <= max_correspondence_distance_)
            indices[i] = best;
    }

    return indices;
}

void ICP::compute_transform(const Eigen::MatrixXf& source,
                             const Eigen::MatrixXf& target,
                             const std::vector<int>& closest_indices,
                             Eigen::Matrix2f& R,
                             Eigen::Vector2f& t) {
    std::vector<int> valid_src, valid_tgt;
    for (int i = 0; i < (int)closest_indices.size(); ++i) {
        if (closest_indices[i] != -1) {
            valid_src.push_back(i);
            valid_tgt.push_back(closest_indices[i]);
        }
    }

    if (valid_src.empty()) {
        R = Eigen::Matrix2f::Identity();
        t = Eigen::Vector2f::Zero();
        return;
    }

    int n = (int)valid_src.size();
    Eigen::MatrixXf src_pts(n, 2), tgt_pts(n, 2);
    for (int i = 0; i < n; ++i) {
        src_pts.row(i) = source.row(valid_src[i]);
        tgt_pts.row(i) = target.row(valid_tgt[i]);
    }

    Eigen::Vector2f src_center = src_pts.colwise().mean();
    Eigen::Vector2f tgt_center = tgt_pts.colwise().mean();

    Eigen::MatrixXf src_c = src_pts.rowwise() - src_center.transpose();
    Eigen::MatrixXf tgt_c = tgt_pts.rowwise() - tgt_center.transpose();

    Eigen::Matrix2f H = src_c.transpose() * tgt_c;
    Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix2f U = svd.matrixU();
    Eigen::Matrix2f V = svd.matrixV();
    R = V * U.transpose();

    if (R.determinant() < 0) {
        V.col(1) *= -1;
        R = V * U.transpose();
    }

    t = tgt_center - R * src_center;
}

float ICP::compute_error(const Eigen::MatrixXf& source,
                          const Eigen::MatrixXf& target,
                          const std::vector<int>& closest_indices) {
    float total = 0.0f;
    int valid = 0;

    for (int i = 0; i < source.rows(); ++i) {
        if (closest_indices[i] == -1) continue;
        total += (source.row(i).transpose() -
                  target.row(closest_indices[i]).transpose()).norm();
        ++valid;
    }

    return valid > 0 ? total / valid : std::numeric_limits<float>::max();
}

} // namespace icp
