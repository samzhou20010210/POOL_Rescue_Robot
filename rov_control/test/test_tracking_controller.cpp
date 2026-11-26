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

// Unit tests for the planar model, trajectory sampling, the tracking
// controller and the path-to-trajectory converter.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "rov_control/planar_model.hpp"
#include "rov_control/tracking_controller.hpp"

namespace rov_control
{
namespace
{

PlanarModel V4Model() {return PlanarModel(LoadHydrodynamicParams(HYDRODYNAMICS_V4_YAML));}

// L-shaped trajectory: 5 m along +x, a quarter circle of radius 1 m, 3 m
// along +y (ends at (6, 4), heading +y), the heading along the path, at up
// to 1 m/s with smooth 1.5 s speed ramps; samples every 0.01 s.
std::vector<TrajectorySample> LTrajectory()
{
  const double radius = 1.0;
  const double length = 5.0 + M_PI / 2.0 * radius + 3.0;
  const double speed = 1.0;
  const double ramp = 1.5;
  auto geometry = [&](double s, Eigen::Vector2d * p, double * heading) {
      if (s <= 5.0) {
        *p = Eigen::Vector2d(s, 0.0);
        *heading = 0.0;
      } else if (s <= 5.0 + M_PI / 2.0 * radius) {
        const double a = (s - 5.0) / radius;
        *p = Eigen::Vector2d(5.0 + radius * std::sin(a), radius - radius * std::cos(a));
        *heading = a;
      } else {
        *p = Eigen::Vector2d(6.0, 1.0 + (s - 5.0 - M_PI / 2.0 * radius));
        *heading = M_PI / 2.0;
      }
    };
  auto smooth = [](double x) {
      x = std::clamp(x, 0.0, 1.0);
      return x * x * (3.0 - 2.0 * x);
    };
  const double cruise = (length - speed * ramp) / speed;
  const double total = 2.0 * ramp + cruise;
  const double dt = 0.01;
  std::vector<TrajectorySample> samples;
  double s = 0.0;
  for (double t = 0.0; t <= total + 1e-9; t += dt) {
    TrajectorySample sample;
    sample.time = t;
    double heading = 0.0;
    geometry(std::min(s, length), &sample.position, &heading);
    sample.yaw = heading;
    samples.push_back(sample);
    const double v = speed * std::min(
      smooth((t + 0.5 * dt) / ramp), smooth((total - t - 0.5 * dt) / ramp));
    s += v * dt;
  }
  // Velocities, accelerations and yaw rates by central differences.
  for (std::size_t k = 0; k < samples.size(); ++k) {
    const std::size_t a = k == 0 ? 0 : k - 1;
    const std::size_t b = std::min(k + 1, samples.size() - 1);
    const double span = samples[b].time - samples[a].time;
    samples[k].velocity = (samples[b].position - samples[a].position) / span;
    samples[k].yaw_rate = (samples[b].yaw - samples[a].yaw) / span;
  }
  for (std::size_t k = 0; k < samples.size(); ++k) {
    const std::size_t a = k == 0 ? 0 : k - 1;
    const std::size_t b = std::min(k + 1, samples.size() - 1);
    samples[k].acceleration =
      (samples[b].velocity - samples[a].velocity) / (samples[b].time - samples[a].time);
  }
  samples.back().velocity.setZero();
  samples.back().acceleration.setZero();
  samples.back().yaw_rate = 0.0;
  return samples;
}

TEST(PlanarModelTest, InverseAndForwardDynamicsAgree)
{
  const PlanarModel model = V4Model();
  const Eigen::Vector3d nu(1.2, -0.4, 0.3);
  const Eigen::Vector3d nu_dot(0.5, 0.2, -0.1);
  const Eigen::Vector3d tau = model.RequiredWrench(nu, nu_dot);
  EXPECT_TRUE(model.Acceleration(nu, tau).isApprox(nu_dot, 1e-12));
  // Steady surge: the wrench is the damping force -X_u u - X_u|u| u|u|.
  const Eigen::Vector3d steady = model.RequiredWrench(
    Eigen::Vector3d(1.5, 0.0, 0.0), Eigen::Vector3d::Zero());
  EXPECT_NEAR(steady(0), 10.0 * 1.5 + 100.0 * 1.5 * 1.5, 1e-9);
  EXPECT_NEAR(steady(1), 0.0, 1e-12);
}

TEST(TrajectorySamplingTest, ValidatesAndInterpolates)
{
  std::vector<TrajectorySample> samples(2);
  samples[1].time = 2.0;
  samples[1].position = Eigen::Vector2d(2.0, 0.0);
  samples[0].velocity = Eigen::Vector2d(1.0, 0.0);
  samples[1].velocity = Eigen::Vector2d(1.0, 0.0);
  samples[1].yaw_rate = 0.4;
  EXPECT_EQ(ValidateTrajectory(samples), "");

  const Reference middle = SampleTrajectory(samples, 0.5);
  EXPECT_TRUE(middle.position.isApprox(Eigen::Vector2d(0.5, 0.0)));
  EXPECT_NEAR(middle.yaw_rate, 0.1, 1e-12);
  EXPECT_NEAR(middle.yaw_acceleration, 0.2, 1e-12);

  const Reference after = SampleTrajectory(samples, 5.0);  // hold the end still
  EXPECT_TRUE(after.position.isApprox(Eigen::Vector2d(2.0, 0.0)));
  EXPECT_TRUE(after.velocity.isZero(0.0));
  EXPECT_EQ(after.yaw_rate, 0.0);

  EXPECT_NE(ValidateTrajectory({}), "");
  std::vector<TrajectorySample> bad = samples;
  bad[1].time = 0.0;  // not increasing
  EXPECT_NE(ValidateTrajectory(bad), "");
  bad = samples;
  bad[1].velocity.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(ValidateTrajectory(bad), "");
}

TEST(TrackingControllerTest, ErrorsAreInTheBodyFrame)
{
  TrackingGains gains;
  gains.use_feedforward = false;
  const TrackingController controller(V4Model(), gains);
  Reference reference;
  reference.position = Eigen::Vector2d(1.0, 0.0);  // 1 m ahead in world +x
  // Robot at the origin facing +y: world +x is its right side (body -y).
  const TrackingOutput output = controller.Compute(
    Eigen::Vector3d(0.0, 0.0, M_PI / 2.0), Eigen::Vector3d::Zero(), reference);
  EXPECT_NEAR(output.position_error.x(), 0.0, 1e-12);
  EXPECT_NEAR(output.position_error.y(), -1.0, 1e-12);
  EXPECT_NEAR(output.wrench(1), -200.0, 1e-9);
  EXPECT_NEAR(output.yaw_error, -M_PI / 2.0, 1e-12);
}

TEST(TrackingControllerTest, FeedforwardAloneHoldsCruiseWithoutError)
{
  const TrackingController controller(V4Model(), TrackingGains());
  Reference reference;
  reference.position = Eigen::Vector2d(3.0, 2.0);
  reference.velocity = Eigen::Vector2d(0.0, 1.5);  // moving along world +y
  reference.yaw = M_PI / 2.0;                      // facing +y: pure surge
  const TrackingOutput output = controller.Compute(
    Eigen::Vector3d(3.0, 2.0, M_PI / 2.0), Eigen::Vector3d(1.5, 0.0, 0.0), reference);
  EXPECT_NEAR(output.wrench(0), 240.0, 1e-9);  // damping at 1.5 m/s
  EXPECT_NEAR(output.wrench(1), 0.0, 1e-9);
  EXPECT_NEAR(output.wrench(2), 0.0, 1e-9);
}

// Closed loop without ROS: the controller drives the shared model directly
// (no thrust limits or lag) around the L-shaped path.
TEST(TrackingControllerTest, ClosedLoopFollowsLPath)
{
  const PlanarModel model = V4Model();
  const TrackingController controller(model, TrackingGains());
  const std::vector<TrajectorySample> samples = LTrajectory();
  ASSERT_TRUE(ValidateTrajectory(samples).empty()) << ValidateTrajectory(samples);
  Eigen::Vector3d pose = Eigen::Vector3d::Zero();
  Eigen::Vector3d nu = Eigen::Vector3d::Zero();
  const double dt = 0.002;
  double max_error = 0.0;
  for (double t = 0.0; t < samples.back().time + 3.0; t += dt) {
    const TrackingOutput output = controller.Compute(pose, nu, SampleTrajectory(samples, t));
    max_error = std::max(max_error, output.position_error.norm());
    nu += model.Acceleration(nu, output.wrench) * dt;
    const double c = std::cos(pose(2));
    const double s = std::sin(pose(2));
    pose += Eigen::Vector3d(c * nu(0) - s * nu(1), s * nu(0) + c * nu(1), nu(2)) * dt;
  }
  EXPECT_LT(max_error, 0.05);  // the feedforward does almost all the work
  EXPECT_LT((pose.head<2>() - Eigen::Vector2d(6.0, 4.0)).norm(), 0.01);
  EXPECT_NEAR(pose(2), M_PI / 2.0, 0.01);
  EXPECT_LT(nu.norm(), 0.01);
}

}  // namespace
}  // namespace rov_control
