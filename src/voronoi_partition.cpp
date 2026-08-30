#include "multi_explore_mapping/voronoi_partition.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace voronoi {

namespace {

struct QNode {
    float dist;
    int idx;
};
struct QNodeGreater {
    bool operator()(const QNode& a, const QNode& b) const { return a.dist > b.dist; }
};

// Single-source Dijkstra distance field over 8-connected free cells.
// Returns width*height floats; unreachable cells are +inf.
std::vector<float> dijkstra_from(
    const nav_msgs::msg::OccupancyGrid& map,
    int src_x, int src_y,
    int8_t obstacle_threshold)
{
    const int W = static_cast<int>(map.info.width);
    const int H = static_cast<int>(map.info.height);
    const float res = map.info.resolution;
    std::vector<float> dist(static_cast<size_t>(W) * static_cast<size_t>(H),
                            std::numeric_limits<float>::infinity());
    if (W <= 0 || H <= 0) return dist;

    auto passable = [&](int x, int y) {
        if (x < 0 || x >= W || y < 0 || y >= H) return false;
        const int8_t v = map.data[static_cast<size_t>(y) * W + x];
        return v >= 0 && v < obstacle_threshold;
    };

    // The robot's own footprint cell can momentarily read as occupied/
    // unknown (SLAM noise, or the map not yet refreshed around it). Snap to
    // the nearest passable cell within a small radius so a robot is never
    // stranded with an all-infinite distance field just because of that.
    if (!passable(src_x, src_y)) {
        int best_x = -1, best_y = -1;
        float best_d = std::numeric_limits<float>::infinity();
        constexpr int kSnapRadius = 5;
        for (int dy = -kSnapRadius; dy <= kSnapRadius; ++dy) {
            for (int dx = -kSnapRadius; dx <= kSnapRadius; ++dx) {
                const int x = src_x + dx, y = src_y + dy;
                if (!passable(x, y)) continue;
                const float d = std::hypot(static_cast<float>(dx), static_cast<float>(dy));
                if (d < best_d) { best_d = d; best_x = x; best_y = y; }
            }
        }
        if (best_x < 0) return dist;  // nothing passable nearby: give up
        src_x = best_x;
        src_y = best_y;
    }

    std::priority_queue<QNode, std::vector<QNode>, QNodeGreater> pq;
    const int src_idx = src_y * W + src_x;
    dist[static_cast<size_t>(src_idx)] = 0.0f;
    pq.push({0.0f, src_idx});

    static const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const float diag_step = res * 1.41421356f;

    while (!pq.empty()) {
        const QNode cur = pq.top();
        pq.pop();
        if (cur.dist > dist[static_cast<size_t>(cur.idx)]) continue;  // stale entry

        const int x = cur.idx % W;
        const int y = cur.idx / W;

        for (int k = 0; k < 8; ++k) {
            const int nx = x + kDx[k];
            const int ny = y + kDy[k];
            if (!passable(nx, ny)) continue;

            const float step = (kDx[k] != 0 && kDy[k] != 0) ? diag_step : res;
            const float nd = cur.dist + step;
            const int nidx = ny * W + nx;
            if (nd < dist[static_cast<size_t>(nidx)]) {
                dist[static_cast<size_t>(nidx)] = nd;
                pq.push({nd, nidx});
            }
        }
    }
    return dist;
}

}  // namespace

PartitionFields compute_partition(
    const nav_msgs::msg::OccupancyGrid& map,
    const std::vector<RobotPose>& robots,
    int8_t obstacle_threshold)
{
    PartitionFields out;
    out.width      = static_cast<int>(map.info.width);
    out.height     = static_cast<int>(map.info.height);
    out.resolution = map.info.resolution;
    out.origin_x   = map.info.origin.position.x;
    out.origin_y   = map.info.origin.position.y;

    const size_t N = static_cast<size_t>(out.width) * static_cast<size_t>(out.height);
    out.owner.assign(N, -1);
    out.second_owner.assign(N, -1);
    out.dist_owner.assign(N, std::numeric_limits<float>::infinity());
    out.dist_second.assign(N, std::numeric_limits<float>::infinity());

    if (out.width <= 0 || out.height <= 0 || robots.empty() || out.resolution <= 0.0)
        return out;

    for (size_t r = 0; r < robots.size(); ++r) {
        const int gx = static_cast<int>(
            std::floor((robots[r].x - out.origin_x) / out.resolution));
        const int gy = static_cast<int>(
            std::floor((robots[r].y - out.origin_y) / out.resolution));

        const auto field = dijkstra_from(map, gx, gy, obstacle_threshold);

        for (size_t i = 0; i < N; ++i) {
            const float d = field[i];
            if (d < out.dist_owner[i]) {
                // Current owner demotes to second place.
                out.dist_second[i]   = out.dist_owner[i];
                out.second_owner[i]  = out.owner[i];
                out.dist_owner[i]    = d;
                out.owner[i]         = static_cast<int8_t>(r);
            } else if (d < out.dist_second[i]) {
                out.dist_second[i]  = d;
                out.second_owner[i] = static_cast<int8_t>(r);
            }
        }
    }

    return out;
}

nav_msgs::msg::OccupancyGrid build_mask(
    const PartitionFields& fields,
    int robot_index,
    double buffer_width_m)
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width      = static_cast<uint32_t>(std::max(0, fields.width));
    grid.info.height     = static_cast<uint32_t>(std::max(0, fields.height));
    grid.info.resolution = static_cast<float>(fields.resolution);
    grid.info.origin.position.x    = fields.origin_x;
    grid.info.origin.position.y    = fields.origin_y;
    grid.info.origin.orientation.w = 1.0;

    const size_t N = fields.owner.size();
    grid.data.assign(N, -1);

    for (size_t i = 0; i < N; ++i) {
        const int8_t own = fields.owner[i];
        if (own < 0) {
            grid.data[i] = -1;  // unassigned: no robot has reached this cell yet
            continue;
        }

        if (own == robot_index) {
            const bool shared =
                (fields.second_owner[i] >= 0) &&
                ((fields.dist_second[i] - fields.dist_owner[i]) < static_cast<float>(buffer_width_m));
            grid.data[i] = shared ? 50 : 100;
        } else {
            // Someone else's exclusive territory, unless this robot is the
            // second-nearest and within the buffer band — the band is
            // symmetric around the boundary so both sides see it as shared.
            const bool i_am_second_within_buffer =
                (fields.second_owner[i] == static_cast<int8_t>(robot_index)) &&
                ((fields.dist_second[i] - fields.dist_owner[i]) < static_cast<float>(buffer_width_m));
            grid.data[i] = i_am_second_within_buffer ? 50 : 0;
        }
    }
    return grid;
}

bool in_own_territory(const nav_msgs::msg::OccupancyGrid& mask, double wx, double wy) {
    const auto& info = mask.info;
    if (info.width == 0 || info.height == 0 || info.resolution <= 0.0f) return true;

    const int gx = static_cast<int>(std::floor((wx - info.origin.position.x) / info.resolution));
    const int gy = static_cast<int>(std::floor((wy - info.origin.position.y) / info.resolution));
    if (gx < 0 || gx >= static_cast<int>(info.width) ||
        gy < 0 || gy >= static_cast<int>(info.height))
        return true;  // outside current mask coverage: don't block on it

    const size_t idx = static_cast<size_t>(gy) * info.width + static_cast<size_t>(gx);
    if (idx >= mask.data.size()) return true;

    const int8_t v = mask.data[idx];
    return v == 50 || v == 100;
}

}  // namespace voronoi
