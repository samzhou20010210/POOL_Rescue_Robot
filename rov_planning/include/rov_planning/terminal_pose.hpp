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

// Terminal pose of the surface approach (design section 3, item 1;
// implementation step 8A). No ROS.
//
// In both victim states the bow faces the victim, and base_link stops short
// of the victim by
//   d = x_bow + L_arm + r_body + g,   g = g_surface or g_dive,
// along the approach heading psi:
//   p_goal = p_v - d [cos psi, sin psi],  yaw_goal = psi.
// The pose is a staging point for the dive / grasp phase; whether the
// deployed arms reach the victim from it still needs the arm CAD.
//
// Approach heading psi (TerminalPoseGenerator):
//   - recomputed (bearing robot -> victim) at every update while the robot is
//     farther than d + d_hold from the victim;
//   - held once the robot is within d + d_hold; the goal position still
//     follows the victim along the held heading;
//   - re-aimed (recomputed once, held again) only if the victim has moved
//     more than the re-aim shift from where it was at the last lock or
//     re-aim AND the bearing differs from the held heading by more than
//     psi_reaim. Robot motion alone never re-aims;
//   - released beyond d + d_hold + d_hyst; Reset() forgets it.
//
// Clearance (CheckTerminalClearance) is checked separately from the pose
// acceptance limits: the stowed outline (hull + stowed arms) must keep
// g_min from the victim's body and the wall clearance from blocked cells,
// and must lie inside the map. A malformed grid fails the check.

#ifndef ROV_PLANNING__TERMINAL_POSE_HPP_
#define ROV_PLANNING__TERMINAL_POSE_HPP_

#include <limits>
#include <string>

#include "Eigen/Dense"
#include "rov_planning/corridor_2d.hpp"

namespace rov_planning
{

enum class VictimState
{
  kSurface,
  kSubmerged,
};

// Parameters (config/terminal_pose.yaml). The stowed-arm protrusion has no
// usable default: it is NaN until set explicitly, and is rejected as NaN.
struct TerminalPoseSettings
{
  double bow_offset = 0.50;              // x_bow, m: bow ahead of base_link
  double stern_offset = 0.50;            // m: stern behind base_link (outline only)
  double hull_half_width = 0.52;         // m: half of the 1.04 m beam (outline only)
  double stowed_arm_protrusion =         // L_arm, m: stowed arms beyond the bow
    std::numeric_limits<double>::quiet_NaN();
  double victim_body_radius = 0.25;      // r_body, m
  double surface_gap = 0.15;             // g_surface, m
  double dive_gap = 0.30;                // g_dive, m
  double heading_hold_distance = 2.0;    // d_hold, m
  double heading_reaim_angle = 0.15;     // psi_reaim, rad
  double heading_reaim_victim_shift = 0.3;  // m: victim motion needed to re-aim
  double heading_hold_hysteresis = 0.5;  // d_hyst, m
  double min_victim_gap = 0.0;           // g_min, m (clearance check), >= 0
  double wall_clearance = 0.5;           // m (clearance check)
};

// Read the settings. Throws std::runtime_error if a key is missing, null
// (in particular stowed_arm_protrusion) or invalid.
TerminalPoseSettings LoadTerminalPoseSettings(const std::string & yaml_path);

// Empty string if the settings are usable, otherwise the reason.
std::string ValidateTerminalPoseSettings(const TerminalPoseSettings & settings);

// d = x_bow + L_arm + r_body + g for the victim state.
double Standoff(const TerminalPoseSettings & settings, VictimState state);

struct TerminalPose
{
  Eigen::Vector2d position = Eigen::Vector2d::Zero();  // base_link goal, m
  double yaw = 0.0;                                    // rad, bow towards the victim
  double standoff = 0.0;                               // d, m
  bool heading_held = false;                           // heading frozen (final approach)
};

// The one bow-facing pose for both victim states: base_link `standoff`
// short of the victim along `approach_yaw`, facing the victim.
TerminalPose BowFacingPose(
  const Eigen::Vector2d & victim, double approach_yaw, double standoff);

class TerminalPoseGenerator
{
public:
  // Throws std::invalid_argument if the settings are not usable.
  explicit TerminalPoseGenerator(const TerminalPoseSettings & settings);

  // Terminal pose for the current robot and victim positions (same frame).
  // Throws std::invalid_argument for non-finite input, or if the robot is at
  // the victim and no heading is held (the approach direction is undefined).
  TerminalPose Update(
    const Eigen::Vector2d & robot, const Eigen::Vector2d & victim, VictimState state);

  // Forget the held heading (new mission).
  void Reset() {holding_ = false;}

  bool holding() const {return holding_;}
  const TerminalPoseSettings & settings() const {return settings_;}

private:
  TerminalPoseSettings settings_;
  bool holding_ = false;
  double held_yaw_ = 0.0;
  Eigen::Vector2d victim_at_lock_ = Eigen::Vector2d::Zero();  // at the last lock or re-aim
};

struct ClearanceReport
{
  double victim_gap = 0.0;     // m, stowed outline to the victim's body (< 0: overlap)
  double wall_distance = 0.0;  // m, outline to the nearest blocked cell (inf: none)
  bool ok = false;
  std::string reason;          // empty if ok
};

// Clearance at a (reached) robot pose [x, y, yaw]. grid may be null (no
// wall or map check). Independent of the 0.25 m / 0.15 rad acceptance
// limits. Not ok if the settings or grid are invalid, the stowed outline
// leaves the map, or a gap is below its minimum.
ClearanceReport CheckTerminalClearance(
  const Eigen::Vector3d & robot_pose, const Eigen::Vector2d & victim,
  const TerminalPoseSettings & settings, const OccupancyGrid2D * grid);

}  // namespace rov_planning

#endif  // ROV_PLANNING__TERMINAL_POSE_HPP_
