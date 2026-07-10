#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <limits>
#include <vector>

/**
 * Euclidean breakpoint clustering for 2D LiDAR (LaserScan).
 *
 * Algorithm:
 *   1. Convert valid scan beams to Cartesian (robot/laser frame)
 *   2. Split into clusters when the Euclidean distance between consecutive
 *      points exceeds an adaptive threshold:
 *        thresh = thresh_base + thresh_factor * min(r_i, r_{i+1})
 *      This accounts for beam divergence: at 1m/10° spacing the gap is
 *      ~17cm; the factor scales the tolerance proportionally with range.
 *   3. Compute cluster centroid, bounding radius, closest-point distance.
 *   4. Discard clusters with fewer than min_pts (noise rejection).
 *
 * Reference: Paper A (Makino et al. 2019) Sec.3.2, adapted for 2D LiDAR.
 */

namespace cluster {

struct Cluster {
    double cx, cy;    // centroid in laser frame [m]
    double angle;     // atan2(cy, cx) – angle to centroid [rad]
    double radius;    // bounding circle radius [m]
    double min_dist;  // closest point distance from origin [m]
    int    n_pts;     // number of scan beams in this cluster
};

/**
 * Detect clusters from a LaserScan.
 *
 * @param scan          Input LaserScan message
 * @param thresh_base   Base split threshold [m] (default 0.10 m)
 * @param thresh_factor Distance-proportional factor (default 0.10)
 * @param min_pts       Minimum points to form a valid cluster (default 3)
 */
inline std::vector<Cluster> detect(
    const sensor_msgs::msg::LaserScan& scan,
    double thresh_base   = 0.10,
    double thresh_factor = 0.10,
    int    min_pts       = 3)
{
    // --- Step 1: Convert valid beams to Cartesian ---
    struct Pt { double x, y, r; };
    std::vector<Pt> pts;
    pts.reserve(scan.ranges.size());

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double r = scan.ranges[i];
        if (!std::isfinite(r) || r < scan.range_min || r >= scan.range_max * 0.99)
            continue;
        double a = scan.angle_min + i * scan.angle_increment;
        pts.push_back({r * std::cos(a), r * std::sin(a), r});
    }

    if (pts.empty()) return {};

    // --- Step 2: Breakpoint split ---
    // raw[k] holds the indices into pts[] for the k-th raw segment
    std::vector<std::vector<int>> raw;
    raw.push_back({0});

    for (size_t i = 1; i < pts.size(); ++i) {
        double d = std::hypot(pts[i].x - pts[i-1].x,
                              pts[i].y - pts[i-1].y);
        double thresh = thresh_base
                        + thresh_factor * std::min(pts[i].r, pts[i-1].r);
        if (d > thresh)
            raw.push_back({});
        raw.back().push_back(static_cast<int>(i));
    }

    // --- Step 3: Compute stats + filter noise ---
    std::vector<Cluster> result;
    result.reserve(raw.size());

    for (const auto& seg : raw) {
        if (static_cast<int>(seg.size()) < min_pts) continue;

        double sx = 0.0, sy = 0.0;
        double min_d = std::numeric_limits<double>::max();

        for (int k : seg) {
            sx    += pts[k].x;
            sy    += pts[k].y;
            min_d  = std::min(min_d, pts[k].r);
        }

        const int    cnt = static_cast<int>(seg.size());
        const double cx  = sx / cnt;
        const double cy  = sy / cnt;

        double radius = 0.0;
        for (int k : seg)
            radius = std::max(radius,
                              std::hypot(pts[k].x - cx, pts[k].y - cy));

        result.push_back({cx, cy, std::atan2(cy, cx), radius, min_d, cnt});
    }

    return result;
}

} // namespace cluster
