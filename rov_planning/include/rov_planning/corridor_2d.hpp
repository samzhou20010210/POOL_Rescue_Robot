// Copyright 2026 sam
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 2D safe flight corridor from a path and an occupancy grid (design
// section 3, item 3). No ROS.
//
// 0. Check: every segment of the path as given must keep clearance from the
//    blocked cells; otherwise the path is rejected (shortcuts below must not
//    hide a blocked waypoint).
// 1. Seeds: walk along the path in steps of seed_spacing (more if the path
//    would need more than max_polygons). Each seed is the straight segment
//    between two path points; it is shortened if it comes closer than
//    clearance to a blocked cell (e.g. where the path bends round a wall).
// 2. For each seed, GCOPTER's FIRI grows a large convex region around the
//    segment among the corners of the blocked cells (grown by clearance),
//    inside the seed's bounding box grown by bounding_margin and clipped to
//    the map. FIRI works in 3D, so it runs in a thin z slab with every point
//    at z = 0, and the polygon is the slice of the result at z = 0.
// 3. An exact check against the blocked cells (squares) then adds a
//    separating half-plane for every cell still closer than clearance, so
//    each polygon keeps at least clearance from every blocked cell. (FIRI
//    only separates points, not whole cells.)
// Consecutive seeds share an end point that lies strictly inside both
// polygons, so consecutive polygons overlap.
//
// Polygons use GCOPTER's half-space format for the (x, y, yaw) trajectory:
// each row [a_x, a_y, 0, d] means a_x x + a_y y + 0 * yaw + d <= 0, with
// (a_x, a_y) of unit length. The yaw coefficient is always zero.

#ifndef ROV_PLANNING__CORRIDOR_2D_HPP_
#define ROV_PLANNING__CORRIDOR_2D_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "Eigen/Dense"

namespace rov_planning
{

// Same layout as nav_msgs/OccupancyGrid, without ROS.
struct OccupancyGrid2D
{
  double resolution = 0.05;                            // m per cell
  Eigen::Vector2d origin = Eigen::Vector2d::Zero();    // m, outer corner of cell (0, 0)
  int width = 0;                                       // cells along x
  int height = 0;                                      // cells along y
  std::vector<int8_t> data;  // row-major (index y * width + x): 0 free, 100 occupied, -1 unknown
};

struct CorridorSettings
{
  double seed_spacing = 1.0;     // delta_s, m (design 6.5)
  int max_polygons = 30;         // N_poly (design 6.5)
  double clearance = 0.0;        // m, least distance from every polygon to every blocked cell
  double bounding_margin = 3.0;  // m, how far a polygon may reach beyond its seed
  int occupied_threshold = 65;   // a cell is blocked if value >= this; unknown (-1) is blocked
};

struct Corridor
{
  std::vector<Eigen::MatrixX4d> polygons;               // rows [a_x, a_y, 0, d]
  std::vector<std::array<Eigen::Vector2d, 2>> seeds;    // seed segment of each polygon
};

// The obstacle inflation used by BuildCorridor (the slow part on a large
// map): which cells are blocked, and which are blocked or within clearance
// of a blocked cell ("grown").
//
// It owns the immutable map snapshot it was computed from, and
// BuildCorridor() takes the map from it, so an inflation can never be paired
// with another map, even one with the same size and origin.
struct CorridorObstacles
{
  std::shared_ptr<const OccupancyGrid2D> grid;  // the snapshot this was computed from
  double clearance = -1.0;                       // inflation settings used
  int occupied_threshold = 0;
  std::vector<bool> blocked;
  std::vector<bool> grown;
};

// Compute the inflation of a map snapshot. Throws std::invalid_argument for
// a null or bad grid or a negative clearance.
std::shared_ptr<const CorridorObstacles> PrepareCorridorObstacles(
  std::shared_ptr<const OccupancyGrid2D> grid, const CorridorSettings & settings);

// Keeps the inflation of one map snapshot. Get() reuses it only for the very
// same snapshot object (pointer identity; the cache keeps the snapshot
// alive, so its address cannot be reused by another map) and the same
// inflation settings (clearance, occupied_threshold); otherwise it rebuilds.
// Not thread-safe: use it from one thread.
class CorridorObstacleCache
{
public:
  std::shared_ptr<const CorridorObstacles> Get(
    const std::shared_ptr<const OccupancyGrid2D> & grid, const CorridorSettings & settings);
  bool last_was_hit() const {return last_was_hit_;}
  void Clear() {cached_.reset();}

private:
  std::shared_ptr<const CorridorObstacles> cached_;
  bool last_was_hit_ = false;
};

// path: at least two points, in the grid's frame. Throws std::runtime_error
// if the path leaves the map, any path segment comes closer than clearance to
// a blocked cell, or the path needs more than max_polygons polygons;
// std::invalid_argument for bad settings or grid.
Corridor BuildCorridor(
  const std::vector<Eigen::Vector2d> & path, const OccupancyGrid2D & grid,
  const CorridorSettings & settings);

// The same with a precomputed inflation (for example from
// CorridorObstacleCache); the map is the inflation's own snapshot. Throws
// std::invalid_argument if the inflation was made with other inflation
// settings (clearance, occupied_threshold) than `settings`.
Corridor BuildCorridor(
  const std::vector<Eigen::Vector2d> & path, const CorridorSettings & settings,
  const CorridorObstacles & obstacles);

// Corners of a bounded 2D polygon given as rows [a_x, a_y, 0, d],
// counter-clockwise.
std::vector<Eigen::Vector2d> PolygonVertices(const Eigen::MatrixX4d & polygon);

// Distance between a convex polygon (its corners, counter-clockwise) and an
// axis-aligned square (centre, half side). 0 if they overlap.
double PolygonSquareDistance(
  const std::vector<Eigen::Vector2d> & vertices, const Eigen::Vector2d & center,
  double half_side);

}  // namespace rov_planning

#endif  // ROV_PLANNING__CORRIDOR_2D_HPP_
