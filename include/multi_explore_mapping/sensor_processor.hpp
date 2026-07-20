#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <multi_explore_mapping/cluster_detector.hpp>
#include <vector>
#include <utility>

/**
 * Sensor processing pipeline for obstacle avoidance and gap frontier detection.
 *
 * Raw LaserScan → Euclidean-breakpoint clusters → cluster-based VFH → gap targets.
 *
 * Why cluster-based VFH instead of raw-beam VFH:
 *   Raw-beam VFH maps each individual beam to one histogram sector.
 *   A box corner struck by only 2–3 beams may not push any sector above the
 *   blocking threshold, causing the robot to collide.
 *   Cluster-based VFH widens each detected object by atan2(radius, min_dist),
 *   so even a 2-beam corner is represented with its true angular extent.
 *   Additionally, an EMERGENCY state hard-stops the robot when any cluster
 *   enters a configurable close-range safety fence — an escape mechanism that
 *   the raw-beam approach lacks entirely.
 */

namespace sensor_proc {

struct ProcessedScan {
    /// Obstacle clusters (Euclidean breakpoint; ≥2 consecutive beams each)
    std::vector<cluster::Cluster> clusters;

    /// VFH polar density histogram (36 sectors × 10°)
    std::vector<double> vfh_hist;

    /// Any forward-hemisphere cluster is within emergency_dist → hard-stop
    bool emergency{false};

    /// Forward ±15° cone is obstacle-dense (set also when emergency is true)
    bool front_blocked{false};

    /// Rotation command to escape the obstacle; 0.0 when CLEAR
    double avoidance_angular_z{0.0};

    /// Gap frontier target: gap center in robot-frame polar + physical opening width
    struct GapTarget {
        double angle;  // bearing to gap center [rad], robot frame
        double dist;   // distance to gap center [m]
        double width;  // Cartesian gap width |P2 - P1| [m]
    };

    /// Gap frontier targets sorted nearest-to-forward first
    std::vector<GapTarget> gap_targets;
};

/**
 * @param scan           Raw LaserScan message
 * @param safe_distance  VFH weight drops to zero at 2× this value [m]
 * @param vfh_threshold  Sector density above which the direction is blocked
 * @param valley_min_deg Minimum passable gap width [deg]
 * @param emergency_dist Cluster closer than this triggers EMERGENCY [m]
 * @param angular_speed  Reference rotation speed used to set avoidance_angular_z
 * @param robot_radius   Half-width of the robot body [m]; inflates obstacle angular extent
 * @param front_cone_deg Half-width of the "front blocked" check cone [deg].
 *                       Default 30° catches walls at up to ±30° off-forward.
 *                       The original 10° (due to integer truncation of the intended 15°)
 *                       was too narrow for diagonal wall approaches.
 */
ProcessedScan process(
    const sensor_msgs::msg::LaserScan& scan,
    double safe_distance,
    double vfh_threshold,
    double valley_min_deg,
    double emergency_dist,
    double angular_speed,
    double robot_radius   = 0.0,
    double front_cone_deg = 30.0,
    double min_gap_width  = 0.4);

} // namespace sensor_proc
