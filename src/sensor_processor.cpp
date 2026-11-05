/**
 * Sensor processing pipeline: LaserScan → clusters → VFH → gap frontiers.
 *
 * See sensor_processor.hpp for design rationale.
 */
#include <multi_explore_mapping/sensor_processor.hpp>
#include <multi_explore_mapping/cluster_detector.hpp>
#include <multi_explore_mapping/gap_extraction.hpp>
#include <multi_explore_mapping/vfh_avoidance.hpp>
#include <algorithm>
#include <cmath>

namespace sensor_proc {

// ---------------------------------------------------------------------------
// Gap frontier detection on the raw scan. 実装は gap_extraction.hpp
// (docs/gap_extraction_spec.md の移植、golden データで検証済み)。
// ここでは robot-frame polar (angle, dist) + width への変換と、既存の
// 消費側(sensor_explore_node / sensor_mb_explore_node)が期待する
// 「前方に近い順」ソートだけを行う。open/jumpの別やidx_start/idx_endは
// コスト関数より下流では使わないため、ここで切り捨てる。
static std::vector<sensor_proc::ProcessedScan::GapTarget> detect_gaps(
    const sensor_msgs::msg::LaserScan& scan,
    const gap_extraction::Params& params)
{
    using GapTarget = sensor_proc::ProcessedScan::GapTarget;
    std::vector<GapTarget> targets;

    const auto gaps = gap_extraction::extract_gaps(scan, params);
    targets.reserve(gaps.size());
    for (const auto& g : gaps) {
        const double angle = std::atan2(g.target_y, g.target_x);
        // 後方(135°超)の目標は除外する。gap_extraction自体はこの制限をしない
        // (golden検証もこのフィルタなしで一致させている)が、本番のフロンティア
        // 選択では以前から後方を除いており、その挙動を維持する。
        if (std::abs(angle) >= M_PI * 0.75) continue;
        targets.push_back({angle, std::hypot(g.target_x, g.target_y), g.extent});
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
    const gap_extraction::Params& gap_params)
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
    ps.gap_targets = detect_gaps(scan, gap_params);

    return ps;
}

} // namespace sensor_proc
