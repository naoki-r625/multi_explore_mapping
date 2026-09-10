#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cstdint>
#include <string>
#include <vector>

/**
 * Multi-robot Voronoi territory partition over a shared occupancy grid.
 *
 * Given the merged map (published by icp_map_matching_node) and each active
 * robot's current world-frame position, computes for every free cell:
 *   - which robot "owns" it (nearest by geodesic distance through known
 *     free space — NOT straight-line distance, so walls correctly block
 *     ownership from leaking through them)
 *   - whether it falls inside a shared buffer band near a territory
 *     boundary, so that ICP map merging always has correspondence
 *     candidates near the seam between two robots' explored regions
 *
 * Distance is computed by one Dijkstra pass per robot over 8-connected free
 * cells (weight = resolution for orthogonal steps, resolution*sqrt(2) for
 * diagonal steps). Cells with value >= obstacle_threshold, or == -1
 * (unknown), are impassable.
 *
 * A cell that no robot can currently reach through known free space is left
 * unassigned (owner = -1). This is deliberate: a robot cannot meaningfully
 * "own" territory it has no known path to yet (this happens routinely early
 * in exploration, before two robots' separately-explored patches have been
 * connected in the merged map). Consumers must treat "my Voronoi cell has
 * no candidates" as a temporary condition, not a dead end — see
 * in_own_territory() below and the escape-hatch behaviour documented in
 * docs/voronoi_partition.md.
 */
namespace voronoi {

struct RobotPose {
    std::string name;   // e.g. "robot_1" (informational; index in the input
                        // vector is what actually identifies the robot)
    double x, y;         // world ("map") frame [m]
};

// Per-cell result of the multi-robot distance computation.
// All vectors are width*height long, row-major, matching the input
// OccupancyGrid's layout.
struct PartitionFields {
    int width = 0, height = 0;
    double resolution = 0.0;
    double origin_x = 0.0, origin_y = 0.0;

    std::vector<int8_t> owner;         // nearest robot index, or -1
    std::vector<int8_t> second_owner;  // second-nearest robot index, or -1
    std::vector<float>  dist_owner;    // [m] geodesic distance to owner
    std::vector<float>  dist_second;   // [m] geodesic distance to
                                       // second_owner (+inf if none reached)
};

// Runs one Dijkstra per robot and combines the resulting distance fields.
// obstacle_threshold matches the convention used elsewhere in this package
// (icp_map_matching_node, frontier detection): cell values >= this are
// treated as obstacle/inflated and block propagation.
//
// `previous` + `hysteresis_margin_m` add optional stability across calls:
// since ownership is otherwise recomputed from scratch every cycle from
// live robot positions, two robots that happen to be roughly equidistant
// from a region can flip which one owns it cycle to cycle (e.g. as they
// move, or as a newly-discovered corridor changes the geodesic distance),
// which sends whichever robot just lost that region backtracking toward
// territory it had already covered. When `previous` is non-null and
// `hysteresis_margin_m > 0`, a cell's previous owner (looked up in
// `previous` by world coordinates, so this still works across recomputes
// where the map's bounding box has grown/shifted) gets a discount of that
// many metres on its distance, so a challenger must be closer by more than
// the margin before territory actually changes hands. Pass
// previous=nullptr (the default) to get the original stateless behaviour.
PartitionFields compute_partition(
    const nav_msgs::msg::OccupancyGrid& map,
    const std::vector<RobotPose>& robots,
    int8_t obstacle_threshold = 50,
    const PartitionFields* previous = nullptr,
    double hysteresis_margin_m = 0.0);

// Looks up the owning robot index recorded in `fields` at world position
// (wx, wy). Returns -1 if that position falls outside fields' spatial
// coverage or has no recorded owner there. Used to implement the
// hysteresis bias in compute_partition() above.
int8_t owner_at(const PartitionFields& fields, double wx, double wy);

// Builds robot `robot_index`'s publishable territory mask from `fields`.
// Cell values:
//   100 -> exclusively this robot's territory
//    50 -> shared buffer band (this robot + a neighbour are both within
//          buffer_width_m of each other's distance at this cell)
//     0 -> another robot's exclusive territory
//    -1 -> unassigned (unreached by any robot yet)
nav_msgs::msg::OccupancyGrid build_mask(
    const PartitionFields& fields,
    int robot_index,
    double buffer_width_m);

// Consumer-side helper: true if (wx, wy) falls inside this robot's own
// territory according to `mask` (cell value 50 or 100). Cells outside the
// mask's spatial coverage are treated as unconstrained (true) — the mask
// only ever covers currently-known map area, and a candidate just beyond
// its edge should not be blocked purely because the mask hasn't caught up.
bool in_own_territory(const nav_msgs::msg::OccupancyGrid& mask,
                      double wx, double wy);

}  // namespace voronoi
