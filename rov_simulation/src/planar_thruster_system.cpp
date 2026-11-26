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

// PlanarThrusterSystem: Gazebo Harmonic system plugin that applies the V4
// thruster forces to the vehicle in the planar test setup.
//
// Input: gz.msgs.Double_V with 8 forces in N, in ThrusterCommand order
// (index i = thruster_0(i+1)); sent by gazebo_thruster_relay.
//
// Planar setup (documented in rov_bringup/models/rov_v4_thrust.sdf.template):
//   - Only the four horizontal channels (thruster_01-04) are used. Each force
//     follows its command with the first-order lag tau_m from
//     thrusters_v4.yaml.
//   - Their planar wrench [Fx, Fy, Mz] is computed from the positions and
//     directions in thrusters_v4.yaml (rows 0, 1, 5 of B) and applied at the
//     link origin (= centre of gravity). The 0.045 m height of the thrusters
//     is ignored, so no roll or pitch moment is applied; the vertical
//     channels (thruster_05-08) are ignored. This matches dynamics_sim_node.
//   - If no command arrives for <command_timeout> s of simulation time, the
//     commands become zero and the forces decay with the lag.
// Damping and added mass are NOT applied here: the Hydrodynamics system and
// the link's <fluid_added_mass> handle them.
//
// Hydrostatics (with world gravity on; optional <hydrostatics> element):
// the hull is a box from <bottom_below_origin> below the link origin, of
// height <hull_height>; the waterline is world z = 0. Buoyancy
//   F_z = rho g A_wp clamp(bottom_below_origin - z, 0, hull_height)
// acts upward at the link origin (r_b = r_g, provisional), with A_wp =
// <waterplane_area>, so the vehicle floats where rho A_wp draft = m.
// Heave damping F_z = <heave_damping> * w (world vertical velocity) and the
// roll / pitch restoring moment -<roll_pitch_restoring> * angle with damping
// <roll_pitch_damping> * rate keep it level (no metacentre is identified
// yet: provisional simulation values, rov_simulation/config/
// gazebo_hydrostatics.yaml).
//
// A gz.msgs.Empty on <reset_topic> puts the model back at the pose it was
// spawned at, at rest, with zero thrust. The pose and zero velocity are held
// for kResetHoldSec of simulation time so no motion leaks through the reset.
//
// SDF parameters:
//   <link_name>        link the wrench is applied to (default base_link)
//   <thruster_config>  path of thrusters_v4.yaml (required)
//   <command_timeout>  s of simulation time (required, > 0)
//   <command_topic>    default /model/<model>/thruster_forces
//   <reset_topic>      default /model/<model>/reset_vehicle
//   <hydrostatics>     optional: <water_density>, <gravity>, <waterplane_area>,
//                      <bottom_below_origin>, <hull_height>, <heave_damping>,
//                      <roll_pitch_restoring>, <roll_pitch_damping>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "gz/msgs/double_v.pb.h"
#include "gz/msgs/empty.pb.h"
#include "gz/plugin/Register.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/transport/Node.hh"
#include "rov_control/thrust_allocator.hpp"
#include "rov_simulation/planar_dynamics.hpp"

namespace rov_simulation
{

namespace
{

// Channels 0-3 (thruster_01-04) are the horizontal thrusters.
constexpr int kNumHorizontal = 4;

// How long a reset holds the model still (s of simulation time).
constexpr double kResetHoldSec = 0.05;

double Seconds(const std::chrono::steady_clock::duration & duration)
{
  return std::chrono::duration<double>(duration).count();
}

}  // namespace

class PlanarThrusterSystem
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm)) {
      gzerr << "PlanarThrusterSystem must be attached to a model.\n";
      return;
    }
    const std::string model_name = model_.Name(ecm);
    const std::string link_name = sdf->Get<std::string>("link_name", "base_link").first;
    link_ = gz::sim::Link(model_.LinkByName(ecm, link_name));
    if (!link_.Valid(ecm)) {
      gzerr << "PlanarThrusterSystem: no link named '" << link_name << "'.\n";
      return;
    }
    // Needed so the link's world pose is available in PreUpdate.
    link_.EnableVelocityChecks(ecm, true);

    const std::string thruster_config = sdf->Get<std::string>("thruster_config", "").first;
    const double command_timeout = sdf->Get<double>("command_timeout", -1.0).first;
    try {
      const std::vector<rov_control::Thruster> thrusters =
        rov_control::LoadThrusterLayout(thruster_config);
      for (int i = 0; i < kNumHorizontal; ++i) {
        if (std::abs(thrusters[i].direction.z()) > 1e-9) {
          throw std::runtime_error(thrusters[i].name + " is not horizontal");
        }
      }
      allocation_matrix_ = rov_control::BuildAllocationMatrix(thrusters);
      bank_ = std::make_unique<ThrusterBank>(
        LoadThrusterTimeConstant(thruster_config), command_timeout);
    } catch (const std::exception & error) {
      gzerr << "PlanarThrusterSystem: " << error.what() << "\n";
      return;
    }

    const std::string command_topic = sdf->Get<std::string>(
      "command_topic", "/model/" + model_name + "/thruster_forces").first;
    const std::string reset_topic = sdf->Get<std::string>(
      "reset_topic", "/model/" + model_name + "/reset_vehicle").first;
    if (sdf->HasElement("hydrostatics")) {
      const auto h = sdf->FindElement("hydrostatics");
      Hydrostatics hs;
      hs.density = h->Get<double>("water_density", hs.density).first;
      hs.gravity = h->Get<double>("gravity", hs.gravity).first;
      hs.waterplane_area = h->Get<double>("waterplane_area", hs.waterplane_area).first;
      hs.bottom_below_origin = h->Get<double>("bottom_below_origin", hs.bottom_below_origin).first;
      hs.hull_height = h->Get<double>("hull_height", hs.hull_height).first;
      hs.heave_damping = h->Get<double>("heave_damping", hs.heave_damping).first;
      hs.roll_pitch_restoring =
        h->Get<double>("roll_pitch_restoring", hs.roll_pitch_restoring).first;
      hs.roll_pitch_damping = h->Get<double>("roll_pitch_damping", hs.roll_pitch_damping).first;
      if (!(hs.density > 0.0) || !(hs.gravity > 0.0) || !(hs.waterplane_area > 0.0) ||
        !(hs.hull_height > 0.0) || hs.heave_damping > 0.0 || hs.roll_pitch_restoring < 0.0 ||
        hs.roll_pitch_damping > 0.0)
      {
        gzerr << "PlanarThrusterSystem: invalid <hydrostatics>\n";
        return;
      }
      hydrostatics_ = hs;
      gzmsg << "PlanarThrusterSystem: hydrostatics A_wp " << hs.waterplane_area
            << " m^2, hull bottom " << hs.bottom_below_origin << " m below the origin\n";
    }
    node_.Subscribe(command_topic, &PlanarThrusterSystem::OnCommand, this);
    node_.Subscribe(reset_topic, &PlanarThrusterSystem::OnReset, this);
    configured_ = true;
    gzmsg << "PlanarThrusterSystem: " << thruster_config << ", command timeout "
          << command_timeout << " s, commands on " << command_topic << ", reset on "
          << reset_topic << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (!configured_ || info.paused) {
      return;
    }
    const double time = Seconds(info.simTime);
    const double dt = Seconds(info.dt);

    // Remember the spawn pose on the first step. (In Configure the pose of a
    // model spawned by ros_gz_sim create is not set yet.)
    if (!has_initial_pose_) {
      initial_pose_ = gz::sim::worldPose(model_.Entity(), ecm);
      has_initial_pose_ = true;
      gzmsg << "PlanarThrusterSystem: reset pose " << initial_pose_ << "\n";
    }

    // Take over what the transport thread received since the last step.
    bool reset = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      reset = reset_requested_;
      reset_requested_ = false;
      if (has_pending_command_) {
        if (!bank_->SetCommand(pending_command_, time)) {
          gzwarn << "PlanarThrusterSystem: ignoring a command with NaN or infinite force\n";
        }
        has_pending_command_ = false;
      }
    }

    if (reset) {
      bank_->Reset();
      reset_hold_until_ = time + kResetHoldSec;
      holding_reset_ = true;
    }
    if (holding_reset_) {
      if (time < reset_hold_until_) {
        // Keep the model at the spawn pose with zero velocity, no thrust.
        model_.SetWorldPoseCmd(ecm, initial_pose_);
        link_.SetLinearVelocity(ecm, gz::math::Vector3d::Zero);
        link_.SetAngularVelocity(ecm, gz::math::Vector3d::Zero);
        return;
      }
      // Done: remove the velocity commands so they do not override the
      // thrust from now on.
      ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(link_.Entity());
      ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(link_.Entity());
      holding_reset_ = false;
    }

    // Lagged forces; only the horizontal channels are used.
    ThrustVector forces = bank_->Update(time, dt);
    forces.tail<kNumThrusters - kNumHorizontal>().setZero();
    const Eigen::Vector3d wrench = PlanarWrench(allocation_matrix_, forces);

    const std::optional<gz::math::Pose3d> pose = link_.WorldPose(ecm);
    if (!pose) {
      return;
    }
    // Body-frame wrench -> world frame, applied at the link origin.
    const gz::math::Vector3d force = pose->Rot().RotateVector(
      gz::math::Vector3d(wrench(0), wrench(1), 0.0));
    const gz::math::Vector3d torque = pose->Rot().RotateVector(
      gz::math::Vector3d(0.0, 0.0, wrench(2)));
    gz::math::Vector3d total_force = force;
    gz::math::Vector3d total_torque = torque;
    if (hydrostatics_) {
      const Hydrostatics & h = *hydrostatics_;
      const double draft =
        std::clamp(h.bottom_below_origin - pose->Pos().Z(), 0.0, h.hull_height);
      const gz::math::Vector3d velocity =
        link_.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero);
      const gz::math::Vector3d rate =
        link_.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
      total_force.Z() += h.density * h.gravity * h.waterplane_area * draft +
        h.heave_damping * velocity.Z();
      // Small-angle restoring moment and damping about the world x and y axes.
      total_torque.X() += -h.roll_pitch_restoring * pose->Rot().Roll() +
        h.roll_pitch_damping * rate.X();
      total_torque.Y() += -h.roll_pitch_restoring * pose->Rot().Pitch() +
        h.roll_pitch_damping * rate.Y();
    }
    link_.AddWorldWrench(ecm, total_force, total_torque);
  }

private:
  void OnCommand(const gz::msgs::Double_V & message)
  {
    if (message.data_size() != kNumThrusters) {
      gzwarn << "PlanarThrusterSystem: expected " << kNumThrusters << " forces, got "
             << message.data_size() << "\n";
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < kNumThrusters; ++i) {
      pending_command_(i) = message.data(i);
    }
    has_pending_command_ = true;
  }

  void OnReset(const gz::msgs::Empty &)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reset_requested_ = true;
    has_pending_command_ = false;
  }

  struct Hydrostatics
  {
    double density = 997.0;             // rho, kg/m^3
    double gravity = 9.81;              // g, m/s^2
    double waterplane_area = 0.25;      // A_wp, m^2
    double bottom_below_origin = 0.15;  // m
    double hull_height = 0.3;           // m
    double heave_damping = 0.0;         // N s/m, <= 0
    double roll_pitch_restoring = 0.0;  // N m/rad, >= 0
    double roll_pitch_damping = 0.0;    // N m s/rad, <= 0
  };

  bool configured_ = false;
  std::optional<Hydrostatics> hydrostatics_;
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};
  AllocationMatrix allocation_matrix_ = AllocationMatrix::Zero();
  std::unique_ptr<ThrusterBank> bank_;
  gz::math::Pose3d initial_pose_;
  bool has_initial_pose_ = false;
  bool holding_reset_ = false;
  double reset_hold_until_ = 0.0;

  // Filled by the transport thread, read in PreUpdate.
  gz::transport::Node node_;
  std::mutex mutex_;
  ThrustVector pending_command_ = ThrustVector::Zero();
  bool has_pending_command_ = false;
  bool reset_requested_ = false;
};

}  // namespace rov_simulation

GZ_ADD_PLUGIN(
  rov_simulation::PlanarThrusterSystem, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
