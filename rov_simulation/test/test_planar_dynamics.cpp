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

// Unit tests for the planar dynamics, using the real V4 parameter files
// from rov_description (paths set by CMake).

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "rov_control/thrust_allocator.hpp"
#include "rov_simulation/planar_dynamics.hpp"
#include "yaml-cpp/yaml.h"

namespace rov_simulation
{
namespace
{

constexpr double kMaxStep = 1.0 / 200.0;     // s, 200 Hz integration
constexpr double kCommandPeriod = 1.0 / 50.0;  // s, 50 Hz commands
constexpr double kTimeout = 0.2;             // s

// Steady speed where thrust equals drag: force = a * s + b * s^2, with
// a = -(linear coefficient) and b = -(quadratic coefficient).
double SteadySpeed(double force, double linear, double quadratic)
{
  const double a = -linear;
  const double b = -quadratic;
  return (-a + std::sqrt(a * a + 4.0 * b * force)) / (2.0 * b);
}

class PlanarDynamicsTest : public ::testing::Test
{
protected:
  PlanarDynamicsTest()
  : params_(LoadHydrodynamicParams(HYDRODYNAMICS_V4_YAML)),
    allocation_matrix_(rov_control::BuildAllocationMatrix(
        rov_control::LoadThrusterLayout(THRUSTERS_V4_YAML))),
    time_constant_(LoadThrusterTimeConstant(THRUSTERS_V4_YAML)),
    dynamics_(params_, allocation_matrix_, time_constant_)
  {
  }

  // Minimum-norm thruster forces that give exactly [Fx, Fy, Mz]. The
  // vertical thrusters have no planar effect, so they get zero force.
  ThrustVector ForcesFor(double fx, double fy, double mz) const
  {
    Eigen::Matrix<double, 3, kNumThrusters> planar;
    planar << allocation_matrix_.row(0), allocation_matrix_.row(1), allocation_matrix_.row(5);
    const Eigen::Vector3d wrench(fx, fy, mz);
    return planar.transpose() * (planar * planar.transpose()).ldlt().solve(wrench);
  }

  // Send the same command at 50 Hz for "duration" seconds.
  static void HoldCommand(PlanarSimulator & simulator, const ThrustVector & forces, double duration)
  {
    const double end = simulator.time() + duration;
    while (simulator.time() < end - 1e-9) {
      simulator.SetCommand(forces, simulator.time());
      simulator.AdvanceTo(simulator.time() + kCommandPeriod);
    }
  }

  HydrodynamicParams params_;
  AllocationMatrix allocation_matrix_;
  double time_constant_;
  PlanarDynamics dynamics_;
};

TEST_F(PlanarDynamicsTest, LoadsDesignValuesFromYaml)
{
  EXPECT_DOUBLE_EQ(params_.mass, 50.0);
  EXPECT_DOUBLE_EQ(params_.inertia_z, 8.67);
  EXPECT_DOUBLE_EQ(params_.x_udot, -15.0);
  EXPECT_DOUBLE_EQ(params_.y_vdot, -15.0);
  EXPECT_DOUBLE_EQ(params_.n_rdot, -1.5);
  EXPECT_DOUBLE_EQ(params_.x_u, -10.0);
  EXPECT_DOUBLE_EQ(params_.y_v, -10.0);
  EXPECT_DOUBLE_EQ(params_.n_r, -5.0);
  EXPECT_DOUBLE_EQ(params_.x_uu, -100.0);
  EXPECT_DOUBLE_EQ(params_.y_vv, -100.0);
  EXPECT_DOUBLE_EQ(params_.n_rr, -30.0);
  EXPECT_DOUBLE_EQ(time_constant_, 0.1);
}

// All 4 horizontal thrusters at f_plan (from thrusters_v4.yaml): pure surge.
TEST_F(PlanarDynamicsTest, ForwardThrustReachesPredictedSteadySpeed)
{
  const double f_plan =
    YAML::LoadFile(THRUSTERS_V4_YAML)["dynamics"]["planning_force_limit"].as<double>();
  ThrustVector forces = ThrustVector::Zero();
  forces.head<4>().setConstant(f_plan);
  const Eigen::Vector3d wrench = dynamics_.PlanarWrench(forces);
  EXPECT_NEAR(wrench(1), 0.0, 1e-9);  // no sway force
  EXPECT_NEAR(wrench(2), 0.0, 1e-9);  // no yaw moment

  PlanarSimulator simulator(dynamics_, kMaxStep, kTimeout, 0.0);
  HoldCommand(simulator, forces, 20.0);
  const double predicted = SteadySpeed(wrench(0), params_.x_u, params_.x_uu);
  const PlanarState & state = simulator.state();
  EXPECT_NEAR(state.velocity(0), predicted, 0.01 * predicted);
  EXPECT_NEAR(state.velocity(1), 0.0, 1e-9);
  EXPECT_NEAR(state.velocity(2), 0.0, 1e-9);
  EXPECT_GT(state.pose(0), 20.0);
  EXPECT_NEAR(state.pose(1), 0.0, 1e-9);
  EXPECT_NEAR(state.pose(2), 0.0, 1e-9);
}

TEST_F(PlanarDynamicsTest, LateralThrustReachesPredictedSwaySpeedWithoutTurning)
{
  const double fy = 150.0;
  PlanarSimulator simulator(dynamics_, kMaxStep, kTimeout, 0.0);
  HoldCommand(simulator, ForcesFor(0.0, fy, 0.0), 20.0);
  const double predicted = SteadySpeed(fy, params_.y_v, params_.y_vv);
  const PlanarState & state = simulator.state();
  EXPECT_NEAR(state.velocity(1), predicted, 0.01 * predicted);
  EXPECT_NEAR(state.velocity(0), 0.0, 1e-6);
  EXPECT_NEAR(state.velocity(2), 0.0, 1e-6);
  EXPECT_NEAR(state.pose(2), 0.0, 1e-6);  // heading unchanged
  EXPECT_GT(state.pose(1), 15.0);         // moved to the left
  EXPECT_NEAR(state.pose(0), 0.0, 1e-6);
}

TEST_F(PlanarDynamicsTest, PureYawTurnsInPlaceAtPredictedRate)
{
  const double mz = 20.0;
  PlanarSimulator simulator(dynamics_, kMaxStep, kTimeout, 0.0);
  HoldCommand(simulator, ForcesFor(0.0, 0.0, mz), 10.0);
  const double predicted = SteadySpeed(mz, params_.n_r, params_.n_rr);
  const PlanarState & state = simulator.state();
  EXPECT_NEAR(state.velocity(2), predicted, 0.01 * predicted);
  EXPECT_NEAR(state.velocity(0), 0.0, 1e-6);
  EXPECT_NEAR(state.velocity(1), 0.0, 1e-6);
  EXPECT_NEAR(state.pose.head<2>().norm(), 0.0, 1e-6);  // stays in place
}

// One command at t = 0 and nothing after: the thrust must stay on until the
// timeout, then decay with the thruster lag, and drag must stop the vehicle.
TEST_F(PlanarDynamicsTest, CommandTimeoutCutsThrustAndVehicleCoastsToStop)
{
  ThrustVector forces = ThrustVector::Zero();
  forces.head<4>().setConstant(110.0);
  PlanarSimulator simulator(dynamics_, kMaxStep, kTimeout, 0.0);
  ASSERT_TRUE(simulator.SetCommand(forces, 0.0));

  simulator.AdvanceTo(0.19);
  EXPECT_TRUE(simulator.CommandIsFresh(0.19));
  const double thrust_before_timeout = simulator.state().thrust(0);
  EXPECT_GT(thrust_before_timeout, 0.8 * 110.0);  // spun up (lag 0.1 s)

  simulator.AdvanceTo(0.21);
  EXPECT_FALSE(simulator.CommandIsFresh(0.21));

  simulator.AdvanceTo(1.5);  // 13 time constants after the cut
  EXPECT_LT(simulator.state().thrust.cwiseAbs().maxCoeff(), 1e-3);
  const double speed_after_cut = simulator.state().velocity(0);
  EXPECT_GT(speed_after_cut, 0.0);  // still coasting

  simulator.AdvanceTo(30.0);
  EXPECT_LT(simulator.state().velocity(0), 0.01 * speed_after_cut);
  EXPECT_GE(simulator.state().velocity(0), 0.0);  // drag never pushes it backwards
}

TEST_F(PlanarDynamicsTest, NonFiniteCommandIsIgnored)
{
  PlanarSimulator simulator(dynamics_, kMaxStep, kTimeout, 0.0);
  ThrustVector forces = ThrustVector::Zero();
  forces(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(simulator.SetCommand(forces, 0.0));
  EXPECT_FALSE(simulator.CommandIsFresh(0.0));
  simulator.AdvanceTo(1.0);
  EXPECT_TRUE(simulator.state().velocity.isZero(0.0));
  EXPECT_TRUE(simulator.state().pose.allFinite());
}

// From 1.5 m/s with no thrust, drag alone stops the vehicle. Integrating
// (m - X_udot) du/dt = X_u u + X_uu u^2 gives the distance from u0 to u1:
//   d = (m_eff / b) * ln((a + b u0) / (a + b u1)),  a = -X_u, b = -X_uu.
TEST_F(PlanarDynamicsTest, CoastDownDistanceMatchesClosedForm)
{
  const double start_speed = 1.5;
  const double stop_speed = 0.05;
  PlanarSimulator simulator(dynamics_, kMaxStep, kTimeout, 0.0);
  PlanarState state;
  state.velocity(0) = start_speed;
  simulator.set_state(state);
  while (simulator.state().velocity(0) > stop_speed) {
    simulator.AdvanceTo(simulator.time() + kMaxStep);
  }
  const double a = -params_.x_u;
  const double b = -params_.x_uu;
  const double effective_mass = params_.mass - params_.x_udot;
  const double predicted =
    effective_mass / b * std::log((a + b * start_speed) / (a + b * stop_speed));
  EXPECT_NEAR(simulator.state().pose(0), predicted, 0.01 * predicted);
}

TEST_F(PlanarDynamicsTest, WaterCurrentCarriesVehicleAtRest)
{
  const PlanarDynamics with_current(
    params_, allocation_matrix_, time_constant_, Eigen::Vector2d(0.2, 0.0));
  PlanarSimulator simulator(with_current, kMaxStep, kTimeout, 0.0);
  simulator.AdvanceTo(10.0);
  EXPECT_NEAR(simulator.state().pose(0), 2.0, 1e-9);
  EXPECT_NEAR(with_current.GroundVelocity(simulator.state())(0), 0.2, 1e-12);
  EXPECT_NEAR(simulator.state().velocity(0), 0.0, 1e-12);  // no motion through the water
}

TEST_F(PlanarDynamicsTest, ResetReturnsToStateAndClearsCommand)
{
  ThrustVector forces = ThrustVector::Zero();
  forces.head<4>().setConstant(110.0);
  PlanarSimulator simulator(dynamics_, kMaxStep, kTimeout, 0.0);
  HoldCommand(simulator, forces, 3.0);
  ASSERT_GT(simulator.state().velocity(0), 1.0);

  PlanarState start;
  start.pose = Eigen::Vector3d(5.0, 5.0, 0.3);
  simulator.Reset(start);
  EXPECT_FALSE(simulator.CommandIsFresh(simulator.time()));
  EXPECT_TRUE(simulator.state().pose.isApprox(start.pose));
  EXPECT_TRUE(simulator.state().velocity.isZero(0.0));
  EXPECT_TRUE(simulator.state().thrust.isZero(0.0));
  simulator.AdvanceTo(simulator.time() + 1.0);  // no command -> stays put
  EXPECT_TRUE(simulator.state().pose.isApprox(start.pose));
}

// The Gazebo plugin's lag must match the RK4 lag of the light simulator:
// a step command reaches 1 - exp(-t / tau) of its value after t.
TEST(ThrusterBankTest, LagFollowsFirstOrderStepResponse)
{
  const double tau = 0.1;
  ThrusterBank bank(tau, kTimeout);
  ThrustVector command = ThrustVector::Zero();
  command(0) = 100.0;
  command(5) = -40.0;
  const double dt = 0.001;  // Gazebo physics step
  double time = 0.0;
  for (int i = 0; i < 100; ++i) {  // 0.1 s, refreshing the command
    bank.SetCommand(command, time);
    bank.Update(time, dt);
    time += dt;
  }
  const double expected = 1.0 - std::exp(-0.1 / tau);
  EXPECT_NEAR(bank.forces()(0), 100.0 * expected, 1e-9);
  EXPECT_NEAR(bank.forces()(5), -40.0 * expected, 1e-9);
  EXPECT_NEAR(bank.forces()(1), 0.0, 1e-12);
}

TEST(ThrusterBankTest, CommandTimeoutDecaysForcesToZero)
{
  ThrusterBank bank(0.1, kTimeout);
  ThrustVector command = ThrustVector::Constant(50.0);
  ASSERT_TRUE(bank.SetCommand(command, 0.0));
  double time = 0.0;
  const double dt = 0.001;
  while (time < 0.19) {
    bank.Update(time, dt);
    time += dt;
  }
  EXPECT_TRUE(bank.CommandIsFresh(time));
  const double before_timeout = bank.forces()(0);
  EXPECT_GT(before_timeout, 0.8 * 50.0);
  while (time < 1.5) {
    bank.Update(time, dt);
    time += dt;
  }
  EXPECT_FALSE(bank.CommandIsFresh(time));
  EXPECT_LT(bank.forces().cwiseAbs().maxCoeff(), 1e-3);
}

TEST(ThrusterBankTest, RejectsNonFiniteCommandAndResets)
{
  ThrusterBank bank(0.1, kTimeout);
  ThrustVector command = ThrustVector::Constant(50.0);
  command(3) = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(bank.SetCommand(command, 0.0));
  EXPECT_FALSE(bank.CommandIsFresh(0.0));
  ASSERT_TRUE(bank.SetCommand(ThrustVector::Constant(50.0), 0.0));
  bank.Update(0.0, 0.05);
  EXPECT_GT(bank.forces()(0), 0.0);
  bank.Reset();
  EXPECT_TRUE(bank.forces().isZero(0.0));
  EXPECT_FALSE(bank.CommandIsFresh(0.0));
  EXPECT_THROW(ThrusterBank(0.0, 0.2), std::invalid_argument);
}

TEST(HydrodynamicParamsTest, MissingFileThrows)
{
  EXPECT_THROW(LoadHydrodynamicParams("/nonexistent/hydro.yaml"), std::runtime_error);
}

}  // namespace
}  // namespace rov_simulation
