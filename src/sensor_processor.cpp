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
// Case A: hit ↔ miss transition between adjacent beams (wall edge into open space)
// Case B: both beams finite but |r_i - r_{i+1}| > safe * 1.5
//         → doorways and corridor ends where beam depth changes abruptly
//
// Branch point coordinate: Cartesian midpoint of the two boundary scan points
// (Paper 18KMJ27 Sec.3.2: B_c = [(X_{P_{l+1}}+X_{P_l})/2, (Y_{P_{l+1}}+Y_{P_l})/2])
//
// For miss beams, a virtual range is estimated as min(rmax*0.7, r_hit*2.0)
// to represent the near edge of the unknown space.

static std::vector<sensor_proc::ProcessedScan::GapTarget> detect_gaps(
    const sensor_msgs::msg::LaserScan& scan,
    double safe_distance,
    double min_gap_width)
{
    using GapTarget = sensor_proc::ProcessedScan::GapTarget;
    std::vector<GapTarget> targets;
    const auto& r  = scan.ranges;
    const float rmax   = scan.range_max;
    const float safe_f = static_cast<float>(safe_distance);

    for (size_t i = 0; i + 1 < r.size(); ++i) {
        const float ri  = r[i];
        const float ri1 = r[i + 1];
        const bool h1 = std::isfinite(ri)  && ri  >= scan.range_min && ri  < rmax * 0.97f;
        const bool h2 = std::isfinite(ri1) && ri1 >= scan.range_min && ri1 < rmax * 0.97f;

        bool is_gap = (h1 != h2);
        if (!is_gap && h1 && h2)
            is_gap = (std::abs(ri - ri1) > safe_f * 1.5f);
        if (!is_gap) continue;

        // Beam angles
        const double a_i  = scan.angle_min + static_cast<double>(i)     * scan.angle_increment;
        const double a_i1 = scan.angle_min + static_cast<double>(i + 1) * scan.angle_increment;

        // Exclude rear arc (> 135° off-forward)
        const double a_mid = (a_i + a_i1) * 0.5;
        if (std::abs(std::atan2(std::sin(a_mid), std::cos(a_mid))) >= M_PI * 0.75) continue;

        // Effective range for each endpoint.
        // Miss beam: estimate as min(rmax*0.7, r_hit*2) — near edge of unknown space.
        const float r1 = h1 ? ri  : std::min(rmax * 0.7f, ri1 * 2.0f);
        const float r2 = h2 ? ri1 : std::min(rmax * 0.7f, ri  * 2.0f);

        // Cartesian endpoints in robot (laser) frame
        const double p1x = r1 * std::cos(a_i);
        const double p1y = r1 * std::sin(a_i);
        const double p2x = r2 * std::cos(a_i1);
        const double p2y = r2 * std::sin(a_i1);

        // Gap width: Euclidean distance between the two boundary points
        const double gap_width = std::hypot(p2x - p1x, p2y - p1y);
        if (gap_width < min_gap_width) continue;

        // Branch point: Cartesian midpoint (Paper 18KMJ27 Sec.3.2 formula)
        const double mx = (p1x + p2x) * 0.5;
        const double my = (p1y + p2y) * 0.5;

        targets.push_back({std::atan2(my, mx), std::hypot(mx, my), gap_width});
    }

    std::sort(targets.begin(), targets.end(),
        [](const GapTarget& p, const GapTarget& q) {
            return std::abs(p.angle) < std::abs(q.angle);
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
    double robot_radius,
    double front_cone_deg,
    double min_gap_width)
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
                       emergency_dist, angular_speed, robot_radius, front_cone_deg);
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
    ps.gap_targets = detect_gaps(scan, safe_distance, min_gap_width);

    return ps;
}

} // namespace sensor_proc
