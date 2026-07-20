#pragma once

#include <multi_explore_mapping/cluster_detector.hpp>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

/**
 * Vector Field Histogram (VFH) obstacle avoidance.
 *
 * Input : cluster::Cluster list (from cluster_detector.hpp)
 * Output: AvoidResult — one of CLEAR / BLOCKED / EMERGENCY
 *
 * Pipeline:
 *   1. build_histogram()  – each cluster paints its angular extent into N sectors
 *   2. check_emergency()  – any forward cluster closer than emergency_dist → EMERGENCY
 *   3. front_blocked()    – forward sectors exceed vfh_threshold → BLOCKED
 *   4. find_valleys()     – locate passable gaps; pick the one closest to forward
 *
 * Reference: Paper A (Makino et al. 2019) Sec.3.1
 */

namespace avoidance {

struct AvoidResult {
    enum class State {
        CLEAR,      // forward path is free; caller may move ahead
        BLOCKED,    // forward blocked; angular_z gives the rotation direction
        EMERGENCY   // confirmed obstacle inside safety envelope; hard stop
    } state;
    double angular_z;   // [rad/s], meaningful for BLOCKED and EMERGENCY
};

class VFH {
public:
    static constexpr int N = 36;   // sectors (10° each)

    VFH(double safe_dist,
        double vfh_threshold,
        double valley_min_deg,
        double emergency_dist,
        double angular_speed,
        double robot_radius   = 0.0,
        double front_cone_deg = 30.0);  // half-width of "blocked" check [deg]

    // Run the full VFH pipeline on the given cluster list.
    AvoidResult compute(const std::vector<cluster::Cluster>& clusters);

    // Last-computed histogram (for debugging / visualization).
    const std::vector<double>& histogram() const { return hist_; }

private:
    double safe_, vfh_t_, v_min_, emerg_d_, ang_, robot_r_, front_cone_;
    std::vector<double> hist_;

    void   build_histogram(const std::vector<cluster::Cluster>& clusters);
    bool   front_blocked() const;
    double check_emergency(const std::vector<cluster::Cluster>& clusters) const;
    std::vector<std::pair<double, double>> find_valleys() const;
};

} // namespace avoidance
