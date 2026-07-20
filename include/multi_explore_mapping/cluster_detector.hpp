#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

/**
 * PCL Euclidean Clustering for 2D LiDAR (LaserScan).
 *
 * Algorithm:
 *   1. Convert valid scan beams to Cartesian (laser frame)
 *   2. PCL EuclideanClusterExtraction with KD-tree search
 *      — groups spatially adjacent points regardless of beam order,
 *        unlike breakpoint clustering which only links consecutive beams.
 *   3. Adaptive wall classification: clusters with more points than a
 *      distance-dependent threshold are labelled is_wall=true.
 *        < 1.5 m → 30 pts,  < 2.5 m → 20 pts,  else → 5 pts
 *      (Close walls have higher LiDAR resolution → more points.)
 */

namespace cluster {

struct Cluster {
    double cx, cy;    // centroid in laser frame [m]
    double angle;     // atan2(cy, cx) – angle to centroid [rad]
    double radius;    // bounding circle radius [m]
    double min_dist;  // closest point distance from origin [m]
    int    n_pts;     // number of scan beams in this cluster
    bool   is_wall;   // true if cluster is large enough to be a wall
};

/**
 * @param scan          Input LaserScan message
 * @param thresh_base   PCL cluster tolerance [m] (default 0.10 m)
 * @param thresh_factor Unused (kept for API compatibility)
 * @param min_pts       Minimum cluster size (default 5)
 */
std::vector<Cluster> detect(
    const sensor_msgs::msg::LaserScan& scan,
    double thresh_base   = 0.10,
    double thresh_factor = 0.10,
    int    min_pts       = 5);

} // namespace cluster
