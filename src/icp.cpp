#include "multi_explore_mapping/icp.hpp"
#include <iostream>
#include <limits>

namespace icp {

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

        // 有効対応数を確認
        int valid = 0;
        for (int idx : indices) if (idx != -1) ++valid;
        result.valid_correspondences = valid;

        if (valid < 5) {
            std::cout << "ICP: too few valid correspondences (" << valid
                      << "), stopping at iter " << iter << std::endl;
            break;
        }

        Eigen::Matrix2f R_delta;
        Eigen::Vector2f t_delta;
        compute_transform(source, target, indices, R_delta, t_delta);

        Eigen::MatrixXf source_new =
            (R_delta * source.transpose()).colwise() + t_delta;
        source_new.transposeInPlace();

        R = R_delta * R;
        t = R_delta * t + t_delta;

        float error = compute_error(source_new, target, indices);

        std::cout << "ICP iter " << iter
                  << ": error=" << error
                  << " valid=" << valid << std::endl;

        result.iterations = iter + 1;
        result.final_error = error;

        if (error < convergence_threshold_) {
            std::cout << "ICP converged at iter " << iter << std::endl;
            break;
        }

        source = source_new;
    }

    result.R = R;
    result.t = t;
    return result;
}

std::vector<int> ICP::find_closest_points(const Eigen::MatrixXf& source,
                                           const Eigen::MatrixXf& target) {
    std::vector<int> indices(source.rows(), -1);

    for (int i = 0; i < source.rows(); ++i) {
        Eigen::Vector2f sp = source.row(i);
        float min_dist = std::numeric_limits<float>::max();
        int best = -1;

        for (int j = 0; j < target.rows(); ++j) {
            float d = (sp - target.row(j).transpose()).norm();
            if (d < min_dist) {
                min_dist = d;
                best = j;
            }
        }

        if (min_dist <= max_correspondence_distance_)
            indices[i] = best;
    }

    return indices;
}

void ICP::compute_transform(const Eigen::MatrixXf& source,
                             const Eigen::MatrixXf& target,
                             const std::vector<int>& closest_indices,
                             Eigen::Matrix2f& R,
                             Eigen::Vector2f& t) {
    // 有効なペアのみ収集
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
