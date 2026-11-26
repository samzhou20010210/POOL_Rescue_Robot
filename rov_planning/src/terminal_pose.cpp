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

#include "rov_planning/terminal_pose.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace rov_planning
{

namespace
{

// Closer than this to the victim, the bearing robot -> victim is undefined.
constexpr double kMinBearingDistance = 1e-3;  // m

double NormalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double ReadNumber(const YAML::Node & root, const std::string & key, const std::string & path)
{
  const YAML::Node node = root[key];
  if (!node || node.IsNull()) {
    throw std::runtime_error(
            path + ": '" + key + "' is missing or null; set an explicit (simulation) value");
  }
  return node.as<double>();
}

}  // namespace

TerminalPoseSettings LoadTerminalPoseSettings(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot read '" + yaml_path + "': " + error.what());
  }
  TerminalPoseSettings s;
  s.bow_offset = ReadNumber(root, "bow_offset", yaml_path);
  s.stern_offset = ReadNumber(root, "stern_offset", yaml_path);
  s.hull_half_width = ReadNumber(root, "hull_half_width", yaml_path);
  s.stowed_arm_protrusion = ReadNumber(root, "stowed_arm_protrusion", yaml_path);
  s.victim_body_radius = ReadNumber(root, "victim_body_radius", yaml_path);
  s.surface_gap = ReadNumber(root, "surface_gap", yaml_path);
  s.dive_gap = ReadNumber(root, "dive_gap", yaml_path);
  s.heading_hold_distance = ReadNumber(root, "heading_hold_distance", yaml_path);
  s.heading_reaim_angle = ReadNumber(root, "heading_reaim_angle", yaml_path);
  s.heading_reaim_victim_shift = ReadNumber(root, "heading_reaim_victim_shift", yaml_path);
  s.heading_hold_hysteresis = ReadNumber(root, "heading_hold_hysteresis", yaml_path);
  s.min_victim_gap = ReadNumber(root, "min_victim_gap", yaml_path);
  s.wall_clearance = ReadNumber(root, "wall_clearance", yaml_path);
  const std::string problem = ValidateTerminalPoseSettings(s);
  if (!problem.empty()) {
    throw std::runtime_error(yaml_path + ": " + problem);
  }
  return s;
}

std::string ValidateTerminalPoseSettings(const TerminalPoseSettings & s)
{
  if (std::isnan(s.stowed_arm_protrusion)) {
    return "stowed_arm_protrusion (L_arm) is not set; give an explicit simulation value";
  }
  const double values[] = {
    s.bow_offset, s.stern_offset, s.hull_half_width, s.stowed_arm_protrusion,
    s.victim_body_radius, s.surface_gap, s.dive_gap, s.heading_hold_distance,
    s.heading_reaim_angle, s.heading_reaim_victim_shift, s.heading_hold_hysteresis,
    s.min_victim_gap, s.wall_clearance};
  for (const double v : values) {
    if (!std::isfinite(v)) {
      return "every setting must be a finite number";
    }
  }
  if (s.bow_offset <= 0.0 || s.stern_offset <= 0.0 || s.hull_half_width <= 0.0 ||
    s.victim_body_radius <= 0.0)
  {
    return "bow_offset, stern_offset, hull_half_width and victim_body_radius must be > 0";
  }
  if (s.stowed_arm_protrusion < 0.0 || s.surface_gap < 0.0 || s.dive_gap < 0.0 ||
    s.heading_hold_distance < 0.0 || s.heading_hold_hysteresis < 0.0 ||
    s.heading_reaim_victim_shift < 0.0 || s.min_victim_gap < 0.0 || s.wall_clearance < 0.0)
  {
    return "protrusion, gaps (including min_victim_gap), hold distance, re-aim shift, "
           "hysteresis and wall clearance must be >= 0";
  }
  if (s.heading_reaim_angle <= 0.0 || s.heading_reaim_angle >= M_PI) {
    return "heading_reaim_angle must be in (0, pi)";
  }
  return "";
}

double Standoff(const TerminalPoseSettings & settings, VictimState state)
{
  const double gap = state == VictimState::kSubmerged ? settings.dive_gap : settings.surface_gap;
  return settings.bow_offset + settings.stowed_arm_protrusion + settings.victim_body_radius + gap;
}

TerminalPose BowFacingPose(const Eigen::Vector2d & victim, double approach_yaw, double standoff)
{
  TerminalPose pose;
  pose.yaw = NormalizeAngle(approach_yaw);
  pose.standoff = standoff;
  pose.position = victim - standoff * Eigen::Vector2d(std::cos(pose.yaw), std::sin(pose.yaw));
  return pose;
}

TerminalPoseGenerator::TerminalPoseGenerator(const TerminalPoseSettings & settings)
: settings_(settings)
{
  const std::string problem = ValidateTerminalPoseSettings(settings_);
  if (!problem.empty()) {
    throw std::invalid_argument(problem);
  }
}

TerminalPose TerminalPoseGenerator::Update(
  const Eigen::Vector2d & robot, const Eigen::Vector2d & victim, VictimState state)
{
  if (!robot.allFinite() || !victim.allFinite()) {
    throw std::invalid_argument("robot and victim positions must be finite");
  }
  const double standoff = Standoff(settings_, state);
  const Eigen::Vector2d to_victim = victim - robot;
  const double distance = to_victim.norm();
  const bool bearing_defined = distance >= kMinBearingDistance;
  const double bearing = bearing_defined ? std::atan2(to_victim.y(), to_victim.x()) : 0.0;
  const double hold_zone = standoff + settings_.heading_hold_distance;

  if (holding_) {
    if (bearing_defined) {
      if (distance > hold_zone + settings_.heading_hold_hysteresis) {
        holding_ = false;  // left the final-approach zone: follow the bearing again
      } else if ((victim - victim_at_lock_).norm() > settings_.heading_reaim_victim_shift &&
        std::abs(NormalizeAngle(bearing - held_yaw_)) > settings_.heading_reaim_angle)
      {
        // The victim itself moved and the bow no longer faces it: re-aim
        // once and keep holding. Robot motion alone never gets here.
        held_yaw_ = bearing;
        victim_at_lock_ = victim;
      }
    }
    // Without a defined bearing (robot at the victim) the held heading stays.
  } else {
    if (!bearing_defined) {
      throw std::invalid_argument(
              "the robot is at the victim and no heading is held: approach direction undefined");
    }
    if (distance <= hold_zone) {
      holding_ = true;  // entering the final approach: freeze the heading
      held_yaw_ = bearing;
      victim_at_lock_ = victim;
    }
  }

  TerminalPose pose = BowFacingPose(victim, holding_ ? held_yaw_ : bearing, standoff);
  pose.heading_held = holding_;
  return pose;
}

ClearanceReport CheckTerminalClearance(
  const Eigen::Vector3d & robot_pose, const Eigen::Vector2d & victim,
  const TerminalPoseSettings & settings, const OccupancyGrid2D * grid)
{
  ClearanceReport report;
  const std::string problem = ValidateTerminalPoseSettings(settings);
  if (problem.empty() && !(robot_pose.allFinite() && victim.allFinite())) {
    report.reason = "robot pose and victim position must be finite";
    return report;
  }
  if (!problem.empty()) {
    report.reason = problem;
    return report;
  }

  // Stowed outline in base_link: x from -stern to bow + arms, y +/- half beam.
  const double front = settings.bow_offset + settings.stowed_arm_protrusion;
  const double rear = -settings.stern_offset;
  const double half_width = settings.hull_half_width;
  const double c = std::cos(robot_pose(2));
  const double s = std::sin(robot_pose(2));
  const Eigen::Vector2d origin = robot_pose.head<2>();

  // Victim gap: distance from the victim centre to the outline, minus r_body.
  const Eigen::Vector2d relative = victim - origin;
  const Eigen::Vector2d local(c * relative.x() + s * relative.y(),
    -s * relative.x() + c * relative.y());
  const Eigen::Vector2d nearest(
    std::clamp(local.x(), rear, front), std::clamp(local.y(), -half_width, half_width));
  report.victim_gap = (local - nearest).norm() - settings.victim_body_radius;

  // Wall distance: outline polygon against blocked cells near it.
  report.wall_distance = std::numeric_limits<double>::infinity();
  if (grid != nullptr) {
    if (!(grid->resolution > 0.0) || !std::isfinite(grid->resolution) ||
      !grid->origin.allFinite() || grid->width <= 0 || grid->height <= 0 ||
      grid->data.size() != static_cast<std::size_t>(grid->width) * grid->height)
    {
      report.reason = "invalid occupancy grid (resolution, origin, size or data length)";
      return report;
    }
    std::vector<Eigen::Vector2d> corners;
    const double xs[] = {rear, front, front, rear};
    const double ys[] = {-half_width, -half_width, half_width, half_width};
    Eigen::Vector2d low = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector2d high = -low;
    for (int k = 0; k < 4; ++k) {  // counter-clockwise
      const Eigen::Vector2d corner =
        origin + Eigen::Vector2d(c * xs[k] - s * ys[k], s * xs[k] + c * ys[k]);
      corners.push_back(corner);
      low = low.cwiseMin(corner);
      high = high.cwiseMax(corner);
    }
    // The whole stowed outline must be on the map.
    const Eigen::Vector2d map_max =
      grid->origin + grid->resolution * Eigen::Vector2d(grid->width, grid->height);
    if ((low.array() < grid->origin.array()).any() || (high.array() > map_max.array()).any()) {
      report.reason = "the stowed outline extends outside the map";
      return report;
    }
    // Cells within reach of the outline's bounding box.
    const double reach = settings.wall_clearance + grid->resolution;
    auto cell = [grid](double metres, double origin) {
        return static_cast<int>(std::floor((metres - origin) / grid->resolution));
      };
    const int x0 = std::max(0, cell(low.x() - reach, grid->origin.x()));
    const int y0 = std::max(0, cell(low.y() - reach, grid->origin.y()));
    const int x1 = std::min(grid->width - 1, cell(high.x() + reach, grid->origin.x()));
    const int y1 = std::min(grid->height - 1, cell(high.y() + reach, grid->origin.y()));
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const int8_t value = grid->data[y * grid->width + x];
        if (value < 0 || value >= 65) {  // unknown or occupied (as the corridor)
          const Eigen::Vector2d center =
            grid->origin + grid->resolution * Eigen::Vector2d(x + 0.5, y + 0.5);
          report.wall_distance = std::min(
            report.wall_distance,
            PolygonSquareDistance(corners, center, 0.5 * grid->resolution));
        }
      }
    }
  }

  char text[160];
  if (report.victim_gap < settings.min_victim_gap) {
    std::snprintf(
      text, sizeof(text), "victim gap %.3f m below %.3f m; ", report.victim_gap,
      settings.min_victim_gap);
    report.reason += text;
  }
  if (report.wall_distance < settings.wall_clearance) {
    std::snprintf(
      text, sizeof(text), "wall distance %.3f m below %.3f m; ", report.wall_distance,
      settings.wall_clearance);
    report.reason += text;
  }
  report.ok = report.reason.empty();
  return report;
}

}  // namespace rov_planning
