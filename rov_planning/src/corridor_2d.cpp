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

#include "rov_planning/corridor_2d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "gcopter/firi.hpp"

namespace rov_planning
{

namespace
{

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Height of the thin 3D slab FIRI runs in (any positive value works).
constexpr double kSlabHalfHeight = 1.0;

std::string PointText(const Eigen::Vector2d & p)
{
  return "(" + std::to_string(p.x()) + ", " + std::to_string(p.y()) + ")";
}

// Blocked cells of the grid, with helpers to go between cells and metres.
class BlockedCells
{
public:
  // `blocked` has one entry per grid cell and must outlive this object.
  BlockedCells(const OccupancyGrid2D & grid, const std::vector<bool> & blocked)
  : grid_(grid), blocked_(blocked) {}

  bool Blocked(int x, int y) const {return blocked_[y * grid_.width + x];}
  Eigen::Vector2d Center(int x, int y) const
  {
    return grid_.origin + grid_.resolution * Eigen::Vector2d(x + 0.5, y + 0.5);
  }
  double half_side() const {return 0.5 * grid_.resolution;}

  // Cells (x0..x1, y0..y1, inclusive) covering a box in metres, clipped to the map.
  std::array<int, 4> CellRange(const Eigen::Vector2d & lo, const Eigen::Vector2d & hi) const
  {
    const Eigen::Vector2d a = (lo - grid_.origin) / grid_.resolution;
    const Eigen::Vector2d b = (hi - grid_.origin) / grid_.resolution;
    return {
      std::max(0, static_cast<int>(std::floor(a.x()))),
      std::max(0, static_cast<int>(std::floor(a.y()))),
      std::min(grid_.width - 1, static_cast<int>(std::floor(b.x()))),
      std::min(grid_.height - 1, static_cast<int>(std::floor(b.y())))};
  }

  Eigen::Vector2d map_min() const {return grid_.origin;}
  Eigen::Vector2d map_max() const
  {
    return grid_.origin + grid_.resolution * Eigen::Vector2d(grid_.width, grid_.height);
  }

private:
  const OccupancyGrid2D & grid_;
  const std::vector<bool> & blocked_;
};

void ValidateGridAndSettings(const OccupancyGrid2D & grid, const CorridorSettings & settings)
{
  if (!(grid.resolution > 0.0) || !grid.origin.allFinite() || grid.width <= 0 ||
    grid.height <= 0 || grid.data.size() != static_cast<std::size_t>(grid.width) * grid.height)
  {
    throw std::invalid_argument("the occupancy grid is empty or inconsistent");
  }
  if (!(settings.seed_spacing > 0.0) || settings.max_polygons < 1 ||
    !(settings.clearance >= 0.0) || !(settings.bounding_margin > 0.0))
  {
    throw std::invalid_argument("corridor settings must be positive");
  }
}

bool SameInflationSettings(const CorridorObstacles & o, const CorridorSettings & settings)
{
  return o.clearance == settings.clearance && o.occupied_threshold == settings.occupied_threshold;
}

Eigen::Vector2d ClosestOnSegment(
  const Eigen::Vector2d & p, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d ab = b - a;
  const double length_squared = ab.squaredNorm();
  if (length_squared < 1e-18) {
    return a;
  }
  const double t = std::clamp((p - a).dot(ab) / length_squared, 0.0, 1.0);
  return a + t * ab;
}

Eigen::Vector2d ClosestOnSquare(
  const Eigen::Vector2d & p, const Eigen::Vector2d & center, double half_side)
{
  return Eigen::Vector2d(
    std::clamp(p.x(), center.x() - half_side, center.x() + half_side),
    std::clamp(p.y(), center.y() - half_side, center.y() + half_side));
}

std::array<Eigen::Vector2d, 4> SquareCorners(const Eigen::Vector2d & center, double half_side)
{
  return {
    center + Eigen::Vector2d(-half_side, -half_side),
    center + Eigen::Vector2d(half_side, -half_side),
    center + Eigen::Vector2d(half_side, half_side),
    center + Eigen::Vector2d(-half_side, half_side)};
}

bool SegmentCrossesSquare(
  const Eigen::Vector2d & a, const Eigen::Vector2d & b, const Eigen::Vector2d & center,
  double half_side)
{
  // Separating axis test with the square's axes and the segment's normal.
  const Eigen::Vector2d lo = center.array() - half_side;
  const Eigen::Vector2d hi = center.array() + half_side;
  if (std::max(a.x(), b.x()) < lo.x() || std::min(a.x(), b.x()) > hi.x() ||
    std::max(a.y(), b.y()) < lo.y() || std::min(a.y(), b.y()) > hi.y())
  {
    return false;
  }
  const Eigen::Vector2d normal(a.y() - b.y(), b.x() - a.x());
  double min_side = kInfinity;
  double max_side = -kInfinity;
  for (const Eigen::Vector2d & corner : SquareCorners(center, half_side)) {
    const double side = normal.dot(corner - a);
    min_side = std::min(min_side, side);
    max_side = std::max(max_side, side);
  }
  return min_side <= 0.0 && max_side >= 0.0;
}

// Closest points (on the segment, on the square) of a segment and a square
// that do not touch. With both shapes convex, one of the two points is an
// end of the segment or a corner of the square.
std::pair<Eigen::Vector2d, Eigen::Vector2d> ClosestSegmentSquare(
  const Eigen::Vector2d & a, const Eigen::Vector2d & b, const Eigen::Vector2d & center,
  double half_side)
{
  std::pair<Eigen::Vector2d, Eigen::Vector2d> best;
  double best_distance = kInfinity;
  auto consider = [&](const Eigen::Vector2d & on_segment, const Eigen::Vector2d & on_square) {
      const double distance = (on_square - on_segment).norm();
      if (distance < best_distance) {
        best_distance = distance;
        best = {on_segment, on_square};
      }
    };
  consider(a, ClosestOnSquare(a, center, half_side));
  consider(b, ClosestOnSquare(b, center, half_side));
  for (const Eigen::Vector2d & corner : SquareCorners(center, half_side)) {
    consider(ClosestOnSegment(corner, a, b), corner);
  }
  return best;
}

double SegmentSquareDistance(
  const Eigen::Vector2d & a, const Eigen::Vector2d & b, const Eigen::Vector2d & center,
  double half_side)
{
  if (SegmentCrossesSquare(a, b, center, half_side)) {
    return 0.0;
  }
  const auto closest = ClosestSegmentSquare(a, b, center, half_side);
  return (closest.second - closest.first).norm();
}

// Least distance from the segment a-b to any blocked cell near it.
double SegmentClearance(
  const Eigen::Vector2d & a, const Eigen::Vector2d & b, const BlockedCells & cells,
  double search_radius)
{
  const Eigen::Vector2d lo = a.cwiseMin(b).array() - search_radius;
  const Eigen::Vector2d hi = a.cwiseMax(b).array() + search_radius;
  const std::array<int, 4> range = cells.CellRange(lo, hi);
  double clearance = kInfinity;
  for (int y = range[1]; y <= range[3]; ++y) {
    for (int x = range[0]; x <= range[2]; ++x) {
      if (cells.Blocked(x, y)) {
        clearance = std::min(
          clearance, SegmentSquareDistance(a, b, cells.Center(x, y), cells.half_side()));
      }
    }
  }
  return clearance;
}

// Point at arc length s along the path.
Eigen::Vector2d PointAt(
  const std::vector<Eigen::Vector2d> & path, const std::vector<double> & cumulative, double s)
{
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (s <= cumulative[i] || i + 1 == path.size()) {
      const double length = cumulative[i] - cumulative[i - 1];
      const double t = length > 0.0 ? std::clamp((s - cumulative[i - 1]) / length, 0.0, 1.0) : 0.0;
      return (1.0 - t) * path[i - 1] + t * path[i];
    }
  }
  return path.back();
}

// Cells that are blocked or within `radius` of a blocked cell (centre to
// centre). FIRI uses the corners of these "grown" cells as obstacle points.
std::vector<bool> GrowBlockedCells(
  const OccupancyGrid2D & grid, const BlockedCells & cells, double radius)
{
  std::vector<bool> grown(grid.data.size(), false);
  const int reach = static_cast<int>(std::ceil(radius / grid.resolution));
  const double reach_squared = std::pow(radius / grid.resolution + 1e-9, 2);
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      if (!cells.Blocked(x, y)) {
        continue;
      }
      for (int dy = -reach; dy <= reach; ++dy) {
        for (int dx = -reach; dx <= reach; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx >= 0 && ny >= 0 && nx < grid.width && ny < grid.height &&
            dx * dx + dy * dy <= reach_squared)
          {
            grown[ny * grid.width + nx] = true;
          }
        }
      }
    }
  }
  return grown;
}

// One polygon around the seed a-b.
Eigen::MatrixX4d BuildPolygon(
  const Eigen::Vector2d & a, const Eigen::Vector2d & b, const OccupancyGrid2D & grid,
  const BlockedCells & cells, const std::vector<bool> & grown,
  const CorridorSettings & settings)
{
  // Box around the seed, clipped to the map.
  const Eigen::Vector2d lo =
    (a.cwiseMin(b).array() - settings.bounding_margin).matrix().cwiseMax(cells.map_min());
  const Eigen::Vector2d hi =
    (a.cwiseMax(b).array() + settings.bounding_margin).matrix().cwiseMin(cells.map_max());

  // 1. FIRI in a thin 3D slab. Box rows [a, b, c, d]: a x + b y + c z + d <= 0.
  Eigen::MatrixX4d box(6, 4);
  box << -1.0, 0.0, 0.0, lo.x(),
    1.0, 0.0, 0.0, -hi.x(),
    0.0, -1.0, 0.0, lo.y(),
    0.0, 1.0, 0.0, -hi.y(),
    0.0, 0.0, -1.0, -kSlabHalfHeight,
    0.0, 0.0, 1.0, -kSlabHalfHeight;

  // Obstacle points: corners of grown cells on the edge of a grown region.
  const std::array<int, 4> range = cells.CellRange(lo, hi);
  std::vector<Eigen::Vector3d> points;
  for (int y = range[1]; y <= range[3]; ++y) {
    for (int x = range[0]; x <= range[2]; ++x) {
      if (!grown[y * grid.width + x]) {
        continue;
      }
      const bool edge =
        x == 0 || y == 0 || x == grid.width - 1 || y == grid.height - 1 ||
        !grown[y * grid.width + x - 1] || !grown[y * grid.width + x + 1] ||
        !grown[(y - 1) * grid.width + x] || !grown[(y + 1) * grid.width + x];
      if (edge) {
        for (const Eigen::Vector2d & c : SquareCorners(cells.Center(x, y), cells.half_side())) {
          points.emplace_back(c.x(), c.y(), 0.0);
        }
      }
    }
  }
  Eigen::Matrix3Xd point_cloud(3, points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    point_cloud.col(i) = points[i];
  }
  Eigen::MatrixX4d polytope;
  if (!firi::firi(
      box, point_cloud, Eigen::Vector3d(a.x(), a.y(), 0.0),
      Eigen::Vector3d(b.x(), b.y(), 0.0), polytope))
  {
    throw std::runtime_error("FIRI failed for the seed " + PointText(a) + " - " + PointText(b));
  }

  // 2. Slice at z = 0: drop the slab faces, keep [a_x, a_y, 0, d] normalized.
  std::vector<Eigen::Vector4d> rows;
  for (int i = 0; i < polytope.rows(); ++i) {
    const double planar = polytope.row(i).head<2>().norm();
    if (planar > 1e-9) {
      rows.emplace_back(
        polytope(i, 0) / planar, polytope(i, 1) / planar, 0.0, polytope(i, 3) / planar);
    }
  }
  auto to_matrix = [&rows]() {
      Eigen::MatrixX4d m(rows.size(), 4);
      for (std::size_t i = 0; i < rows.size(); ++i) {
        m.row(i) = rows[i].transpose();
      }
      return m;
    };

  // 3. Exact clearance: separate every blocked cell that is still too close.
  const double search = settings.clearance + 2.0 * grid.resolution;
  const std::array<int, 4> near = cells.CellRange(
    lo.array() - search, hi.array() + search);
  for (int round = 0; round < 1000; ++round) {
    const std::vector<Eigen::Vector2d> vertices = PolygonVertices(to_matrix());
    // The violating cell closest to the seed.
    double closest_to_seed = kInfinity;
    Eigen::Vector2d worst_center;
    for (int y = near[1]; y <= near[3]; ++y) {
      for (int x = near[0]; x <= near[2]; ++x) {
        if (!cells.Blocked(x, y)) {
          continue;
        }
        const Eigen::Vector2d center = cells.Center(x, y);
        if (PolygonSquareDistance(vertices, center, cells.half_side()) <
          settings.clearance - 1e-9)
        {
          const double to_seed = SegmentSquareDistance(a, b, center, cells.half_side());
          if (to_seed < closest_to_seed) {
            closest_to_seed = to_seed;
            worst_center = center;
          }
        }
      }
    }
    if (closest_to_seed == kInfinity) {
      return to_matrix();
    }
    // Half-plane through the cell's closest point, moved back by clearance:
    // n . x <= n . q - clearance, with n pointing from the seed to the cell.
    const auto closest = ClosestSegmentSquare(a, b, worst_center, cells.half_side());
    const Eigen::Vector2d n = (closest.second - closest.first).normalized();
    rows.emplace_back(n.x(), n.y(), 0.0, -(n.dot(closest.second) - settings.clearance));
  }
  throw std::runtime_error("corridor clearance check did not finish");
}

}  // namespace

std::vector<Eigen::Vector2d> PolygonVertices(const Eigen::MatrixX4d & polygon)
{
  // Every pair of edge lines meets in a candidate corner; keep the ones
  // inside all half-planes.
  std::vector<Eigen::Vector2d> vertices;
  const int n = static_cast<int>(polygon.rows());
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      Eigen::Matrix2d lines;
      lines << polygon(i, 0), polygon(i, 1), polygon(j, 0), polygon(j, 1);
      if (std::abs(lines.determinant()) < 1e-12) {
        continue;
      }
      const Eigen::Vector2d p = lines.inverse() * Eigen::Vector2d(-polygon(i, 3), -polygon(j, 3));
      bool inside = true;
      for (int k = 0; k < n && inside; ++k) {
        inside = polygon(k, 0) * p.x() + polygon(k, 1) * p.y() + polygon(k, 3) <= 1e-9;
      }
      const bool duplicate = std::any_of(
        vertices.begin(), vertices.end(),
        [&p](const Eigen::Vector2d & v) {return (v - p).norm() < 1e-9;});
      if (inside && !duplicate) {
        vertices.push_back(p);
      }
    }
  }
  // Counter-clockwise around the centre.
  Eigen::Vector2d centre = Eigen::Vector2d::Zero();
  for (const Eigen::Vector2d & v : vertices) {
    centre += v;
  }
  centre /= std::max<std::size_t>(vertices.size(), 1);
  std::sort(
    vertices.begin(), vertices.end(),
    [&centre](const Eigen::Vector2d & p, const Eigen::Vector2d & q) {
      return std::atan2(p.y() - centre.y(), p.x() - centre.x()) <
             std::atan2(q.y() - centre.y(), q.x() - centre.x());
    });
  return vertices;
}

double PolygonSquareDistance(
  const std::vector<Eigen::Vector2d> & vertices, const Eigen::Vector2d & center,
  double half_side)
{
  const std::array<Eigen::Vector2d, 4> corners = SquareCorners(center, half_side);
  const std::size_t n = vertices.size();
  // Separating axis test: the square's axes and the polygon's edge normals.
  bool separated = false;
  for (int axis = 0; axis < 2 && !separated; ++axis) {
    double low = kInfinity;
    double high = -kInfinity;
    for (const Eigen::Vector2d & v : vertices) {
      low = std::min(low, v(axis));
      high = std::max(high, v(axis));
    }
    separated = (high < center(axis) - half_side) || (low > center(axis) + half_side);
  }
  for (std::size_t i = 0; i < n && !separated; ++i) {
    const Eigen::Vector2d edge = vertices[(i + 1) % n] - vertices[i];
    const Eigen::Vector2d outward(edge.y(), -edge.x());  // CCW polygon
    double nearest_corner = kInfinity;
    for (const Eigen::Vector2d & c : corners) {
      nearest_corner = std::min(nearest_corner, outward.dot(c - vertices[i]));
    }
    separated = nearest_corner > 0.0;
  }
  if (!separated) {
    return 0.0;
  }
  double distance = kInfinity;
  for (std::size_t i = 0; i < n; ++i) {
    distance = std::min(
      distance, (ClosestOnSquare(vertices[i], center, half_side) - vertices[i]).norm());
    for (const Eigen::Vector2d & c : corners) {
      distance = std::min(
        distance, (ClosestOnSegment(c, vertices[i], vertices[(i + 1) % n]) - c).norm());
    }
  }
  return distance;
}

std::shared_ptr<const CorridorObstacles> PrepareCorridorObstacles(
  std::shared_ptr<const OccupancyGrid2D> grid, const CorridorSettings & settings)
{
  if (!grid) {
    throw std::invalid_argument("no occupancy grid");
  }
  ValidateGridAndSettings(*grid, settings);
  auto obstacles = std::make_shared<CorridorObstacles>();
  obstacles->clearance = settings.clearance;
  obstacles->occupied_threshold = settings.occupied_threshold;
  obstacles->blocked.assign(grid->data.size(), false);
  for (std::size_t i = 0; i < grid->data.size(); ++i) {
    obstacles->blocked[i] = grid->data[i] < 0 || grid->data[i] >= settings.occupied_threshold;
  }
  const BlockedCells cells(*grid, obstacles->blocked);
  obstacles->grown = GrowBlockedCells(*grid, cells, settings.clearance);
  obstacles->grid = std::move(grid);
  return obstacles;
}

std::shared_ptr<const CorridorObstacles> CorridorObstacleCache::Get(
  const std::shared_ptr<const OccupancyGrid2D> & grid, const CorridorSettings & settings)
{
  last_was_hit_ = cached_ && grid && cached_->grid == grid &&
    SameInflationSettings(*cached_, settings);
  if (!last_was_hit_) {
    cached_.reset();  // stays empty if Prepare throws
    cached_ = PrepareCorridorObstacles(grid, settings);
  }
  return cached_;
}

Corridor BuildCorridor(
  const std::vector<Eigen::Vector2d> & path, const OccupancyGrid2D & grid,
  const CorridorSettings & settings)
{
  return BuildCorridor(
    path, settings,
    *PrepareCorridorObstacles(std::make_shared<const OccupancyGrid2D>(grid), settings));
}

Corridor BuildCorridor(
  const std::vector<Eigen::Vector2d> & path, const CorridorSettings & settings,
  const CorridorObstacles & obstacles)
{
  if (path.size() < 2) {
    throw std::invalid_argument("the path needs at least two points");
  }
  if (!obstacles.grid) {
    throw std::invalid_argument("the corridor obstacles have no map snapshot");
  }
  // The map is always the inflation's own snapshot.
  const OccupancyGrid2D & grid = *obstacles.grid;
  ValidateGridAndSettings(grid, settings);
  if (!SameInflationSettings(obstacles, settings)) {
    throw std::invalid_argument(
            "the corridor obstacles were made with other inflation settings");
  }

  const BlockedCells cells(grid, obstacles.blocked);
  for (const Eigen::Vector2d & p : path) {
    if ((p.array() < cells.map_min().array()).any() ||
      (p.array() > cells.map_max().array()).any())
    {
      throw std::runtime_error("the path leaves the map at " + PointText(p));
    }
  }
  // Check the path as given, segment by segment, before any shortcut: a
  // seed is a straight shortcut between two path points, and a clear
  // shortcut must not hide a path that runs through a blocked cell.
  const double search = settings.clearance + 2.0 * grid.resolution;
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (SegmentClearance(path[i - 1], path[i], cells, search) <= settings.clearance + 1e-9) {
      throw std::runtime_error(
              "path segment " + std::to_string(i - 1) + " (" + PointText(path[i - 1]) + " - " +
              PointText(path[i]) + ") comes closer than clearance to a blocked cell");
    }
  }
  const std::vector<bool> & grown = obstacles.grown;

  std::vector<double> cumulative = {0.0};
  for (std::size_t i = 1; i < path.size(); ++i) {
    cumulative.push_back(cumulative.back() + (path[i] - path[i - 1]).norm());
  }
  const double length = cumulative.back();
  const double spacing = std::max(settings.seed_spacing, length / settings.max_polygons);

  Corridor corridor;
  double s_start = 0.0;
  Eigen::Vector2d start = path.front();
  while (s_start < length - 1e-9) {
    // Longest seed (up to spacing) whose straight segment keeps clearance.
    double step = spacing;
    double s_end = 0.0;
    Eigen::Vector2d end;
    while (true) {
      s_end = std::min(s_start + step, length);
      end = PointAt(path, cumulative, s_end);
      if (SegmentClearance(start, end, cells, search) > settings.clearance + 1e-9) {
        break;
      }
      step *= 0.5;
      if (step < 0.5 * grid.resolution) {
        throw std::runtime_error(
                "the path comes closer than clearance to a blocked cell near " +
                PointText(start));
      }
    }
    if (static_cast<int>(corridor.polygons.size()) == settings.max_polygons) {
      throw std::runtime_error(
              "the path needs more than " + std::to_string(settings.max_polygons) + " polygons");
    }
    corridor.polygons.push_back(BuildPolygon(start, end, grid, cells, grown, settings));
    corridor.seeds.push_back({start, end});
    s_start = s_end;
    start = end;
  }
  return corridor;
}

}  // namespace rov_planning
