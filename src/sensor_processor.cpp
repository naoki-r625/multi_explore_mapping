/**
 * Sensor processing pipeline: LaserScan → clusters → VFH → gap frontiers.
 *
 * See sensor_processor.hpp for design rationale.
 */
#include <multi_explore_mapping/sensor_processor.hpp>
#include <multi_explore_mapping/cluster_detector.hpp>
#include <multi_explore_mapping/vfh_avoidance.hpp>
#include <algorithm>
#include <cmath>

namespace sensor_proc {

// ---------------------------------------------------------------------------
// Gap frontier detection on the raw scan.
//
// Case A: hit ↔ miss transition between adjacent beams (basic gap, Paper A 3.2)
// Case B: both beams finite but |r_i - r_{i+1}| > safe * 1.5
//         → doorways and corridor ends where the beam depth changes abruptly
//
// Results sorted nearest-to-forward so control_loop picks the closest target.

static std::vector<std::pair<double, double>> detect_gaps(
    const sensor_msgs::msg::LaserScan& scan,
    double safe_distance)
{
    std::vector<std::pair<double, double>> targets;
    const auto& r = scan.ranges;
    const float rmax  = scan.range_max;
    const float safe_f = static_cast<float>(safe_distance);

    for (size_t i = 0; i + 1 < r.size(); ++i) {
        const float ri  = r[i];
        const float ri1 = r[i + 1];
        const bool h1 = std::isfinite(ri)  && ri  > scan.range_min && ri  < rmax * 0.97f;
        const bool h2 = std::isfinite(ri1) && ri1 > scan.range_min && ri1 < rmax * 0.97f;

        bool is_gap = (h1 != h2);
        if (!is_gap && h1 && h2)
            is_gap = (std::abs(ri - ri1) > safe_f * 1.5f);
        if (!is_gap) continue;

        double a = scan.angle_min + (i + 0.5) * scan.angle_increment;
        a = std::atan2(std::sin(a), std::cos(a));

        // Exclude rear arc (> 135° off-forward) — not useful as exploration targets
        if (std::abs(a) >= M_PI * 0.75) continue;

        const double dist = h1 ? ri : (h2 ? ri1 : std::min(ri, ri1));
        targets.push_back({a, dist});
    }

    std::sort(targets.begin(), targets.end(),
        [](const auto& p, const auto& q) {
            return std::abs(p.first) < std::abs(q.first);
        });
    return targets;
}

// ---------------------------------------------------------------------------

ProcessedScan process(
    const sensor_msgs::msg::LaserScan& scan,
    double safe_distance,
    double vfh_threshold,
    double valley_min_deg,
    double emergency_dist,
    double angular_speed,
    double robot_radius)
{
    ProcessedScan ps;

    // Step 1: Euclidean breakpoint clustering.
    // min_pts=2: a 2-beam corner at close range still forms a cluster and can
    // trigger the emergency stop before the robot physically contacts it.
    ps.clusters = cluster::detect(scan, /*thresh_base=*/0.10,
                                        /*thresh_factor=*/0.10,
                                        /*min_pts=*/2);

    // Step 2: Cluster-based VFH.
    // build_histogram() spreads each cluster over atan2(radius+0.05, min_dist)
    // in the histogram, giving narrow obstacles their correct angular footprint.
    avoidance::VFH vfh(safe_distance, vfh_threshold, valley_min_deg,
                       emergency_dist, angular_speed, robot_radius);
    const auto result = vfh.compute(ps.clusters);
    ps.vfh_hist = vfh.histogram();

    switch (result.state) {
        using S = avoidance::AvoidResult::State;
        case S::EMERGENCY:
            ps.emergency           = true;
            ps.front_blocked       = true;
            ps.avoidance_angular_z = result.angular_z;
            break;
        case S::BLOCKED:
            ps.emergency           = false;
            ps.front_blocked       = true;
            ps.avoidance_angular_z = result.angular_z;
            break;
        case S::CLEAR:
            ps.emergency           = false;
            ps.front_blocked       = false;
            ps.avoidance_angular_z = 0.0;
            break;
    }

    // Step 3: Gap frontier detection from raw scan.
    ps.gap_targets = detect_gaps(scan, safe_distance);

    return ps;
}

} // namespace sensor_proc
