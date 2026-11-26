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

// Unit tests for the ROS-free thrust allocator, using the real V4 layout
// from rov_description/config/thrusters_v4.yaml (path set by CMake).

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "rov_control/thrust_allocator.hpp"

namespace rov_control
{
namespace
{

constexpr double kForwardLimit = 140.0;  // N
constexpr double kReverseLimit = 100.0;  // N

class ThrustAllocatorTest : public ::testing::Test
{
protected:
  ThrustAllocatorTest()
  : thrusters_(LoadThrusterLayout(THRUSTERS_V4_YAML)),
    allocation_matrix_(BuildAllocationMatrix(thrusters_)),
    allocator_(allocation_matrix_, MakeSettings())
  {
  }

  static AllocatorSettings MakeSettings()
  {
    AllocatorSettings settings;
    settings.max_forward_force = kForwardLimit;
    settings.max_reverse_force = kReverseLimit;
    return settings;
  }

  static WrenchVector Wrench(double fx, double fy, double fz, double mx, double my, double mz)
  {
    WrenchVector wrench;
    wrench << fx, fy, fz, mx, my, mz;
    return wrench;
  }

  // Every force must stay inside [-kReverseLimit, kForwardLimit].
  static void ExpectWithinLimits(const AllocationResult & result)
  {
    for (int i = 0; i < kNumThrusters; ++i) {
      EXPECT_LE(result.forces(i), kForwardLimit + 1e-9) << "thruster index " << i;
      EXPECT_GE(result.forces(i), -kReverseLimit - 1e-9) << "thruster index " << i;
    }
  }

  // A feasible request must be reproduced within 1% (whole 6-axis wrench).
  void ExpectFeasible(const WrenchVector & requested)
  {
    const AllocationResult result = allocator_.Allocate(requested);
    EXPECT_TRUE(result.converged);
    EXPECT_FALSE(result.saturated);
    ExpectWithinLimits(result);
    const WrenchVector achieved = allocation_matrix_ * result.forces;
    EXPECT_LE((achieved - requested).norm(), 0.01 * requested.norm())
      << "requested " << requested.transpose() << "\nachieved " << achieved.transpose();
    EXPECT_LT((result.achieved - achieved).norm(), 1e-9);
    EXPECT_LT((result.residual - (requested - achieved)).norm(), 1e-9);
  }

  std::vector<Thruster> thrusters_;
  AllocationMatrix allocation_matrix_;
  ThrustAllocator allocator_;
};

TEST_F(ThrustAllocatorTest, LoadsThrustersInDocumentedOrder)
{
  ASSERT_EQ(thrusters_.size(), static_cast<std::size_t>(kNumThrusters));
  const char * expected[] = {
    "thruster_01", "thruster_02", "thruster_03", "thruster_04",
    "thruster_05", "thruster_06", "thruster_07", "thruster_08"};
  for (int i = 0; i < kNumThrusters; ++i) {
    EXPECT_EQ(thrusters_[i].name, expected[i]);
  }
}

// Columns of B worked out by hand from thrusters_v4.yaml.
//   column = [d ; r x d], r x d = (ry dz - rz dy, rz dx - rx dz, rx dy - ry dx)
TEST_F(ThrustAllocatorTest, AllocationMatrixMatchesHandComputedColumns)
{
  const double a = std::sqrt(0.5);
  // thruster_01: r = (-0.32, -0.415, 0.045), d = (a, -a, 0)
  //   r x d = (0.045 a, 0.045 a, 0.32 a + 0.415 a)
  WrenchVector thruster_01;
  thruster_01 << a, -a, 0.0, 0.045 * a, 0.045 * a, 0.735 * a;
  // thruster_05: r = (-0.38, -0.26, 0.045), d = (0, 0, 1)
  //   r x d = (-0.26, 0.38, 0)
  WrenchVector thruster_05;
  thruster_05 << 0.0, 0.0, 1.0, -0.26, 0.38, 0.0;
  EXPECT_TRUE(allocation_matrix_.col(0).isApprox(thruster_01, 1e-12))
    << allocation_matrix_.col(0).transpose();
  EXPECT_TRUE(allocation_matrix_.col(4).isApprox(thruster_05, 1e-12))
    << allocation_matrix_.col(4).transpose();
}

TEST_F(ThrustAllocatorTest, ZeroRequestGivesZeroForces)
{
  const AllocationResult result = allocator_.Allocate(WrenchVector::Zero());
  EXPECT_TRUE(result.forces.isZero(1e-12));
  EXPECT_FALSE(result.saturated);
}

TEST_F(ThrustAllocatorTest, PureSurgeIsReproducedWithinOnePercent)
{
  ExpectFeasible(Wrench(200.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  ExpectFeasible(Wrench(-200.0, 0.0, 0.0, 0.0, 0.0, 0.0));
}

TEST_F(ThrustAllocatorTest, PureSwayIsReproducedWithinOnePercent)
{
  ExpectFeasible(Wrench(0.0, 200.0, 0.0, 0.0, 0.0, 0.0));
  ExpectFeasible(Wrench(0.0, -200.0, 0.0, 0.0, 0.0, 0.0));
}

TEST_F(ThrustAllocatorTest, PureYawIsReproducedWithinOnePercent)
{
  ExpectFeasible(Wrench(0.0, 0.0, 0.0, 0.0, 0.0, 50.0));
  ExpectFeasible(Wrench(0.0, 0.0, 0.0, 0.0, 0.0, -50.0));
}

// 1000 N of surge is far beyond what 4 horizontal thrusters can give
// (about 4 * 0.707 * 140 = 396 N).
TEST_F(ThrustAllocatorTest, InfeasibleRequestRespectsLimitsAndReportsResidual)
{
  const WrenchVector requested = Wrench(1000.0, 0.0, 0.0, 0.0, 0.0, 300.0);
  const AllocationResult result = allocator_.Allocate(requested);
  EXPECT_TRUE(result.converged);
  ExpectWithinLimits(result);
  EXPECT_TRUE(result.saturated);
  EXPECT_GT(result.num_saturated, 0);

  const WrenchVector achieved = allocation_matrix_ * result.forces;
  EXPECT_LT((result.achieved - achieved).norm(), 1e-9);
  EXPECT_LT((result.residual - (requested - achieved)).norm(), 1e-9);
  EXPECT_GT(result.residual.norm(), 100.0);  // a large part is not reachable
  EXPECT_GT(result.achieved(0), 300.0);      // but it still pushes forward hard
}

// Full surge in reverse needs about 106 N per horizontal thruster, above the
// 100 N reverse limit but below the 140 N forward limit. The reverse limit
// must be the one that applies.
TEST_F(ThrustAllocatorTest, ReverseLimitIsSeparateFromForwardLimit)
{
  const WrenchVector requested = Wrench(-300.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const AllocationResult result = allocator_.Allocate(requested);
  ExpectWithinLimits(result);
  EXPECT_TRUE(result.saturated);
  EXPECT_NEAR(result.forces.minCoeff(), -kReverseLimit, 1e-9);
  // Achieved surge is about -283 N (4 * 0.707 * -100 N), less than the
  // -300 N requested, so the residual (requested - achieved) is negative.
  EXPECT_GT(result.achieved(0), -300.0);
  EXPECT_LT(result.residual(0), -1.0);
}

// The answer must be the true optimum of the QP (KKT conditions): a free
// thruster has zero gradient, a thruster at its upper limit could only gain
// by going higher, and one at its lower limit only by going lower.
TEST_F(ThrustAllocatorTest, InfeasibleAnswerIsOptimal)
{
  const AllocatorSettings settings = MakeSettings();
  const WrenchVector requested = Wrench(350.0, -250.0, 40.0, 5.0, -5.0, 150.0);
  const AllocationResult result = allocator_.Allocate(requested);
  ASSERT_TRUE(result.converged);

  Eigen::Matrix<double, kNumThrusters, kNumThrusters> hessian =
    allocation_matrix_.transpose() * settings.weights.asDiagonal() * allocation_matrix_;
  hessian.diagonal().array() += settings.regularization;
  const ThrustVector gradient = hessian * result.forces -
    allocation_matrix_.transpose() * settings.weights.asDiagonal() * requested;
  for (int i = 0; i < kNumThrusters; ++i) {
    const double force = result.forces(i);
    if (std::abs(force - kForwardLimit) < 1e-9) {
      EXPECT_LE(gradient(i), 1e-6) << "thruster index " << i;
    } else if (std::abs(force + kReverseLimit) < 1e-9) {
      EXPECT_GE(gradient(i), -1e-6) << "thruster index " << i;
    } else {
      EXPECT_NEAR(gradient(i), 0.0, 1e-6) << "thruster index " << i;
    }
  }
}

// Regression: a NaN or infinite request must give zero thrust and a fault,
// never forces at a limit.
TEST_F(ThrustAllocatorTest, NonFiniteRequestGivesZeroThrustAndFault)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const WrenchVector bad_requests[] = {
    Wrench(nan, 0.0, 0.0, 0.0, 0.0, 0.0),
    Wrench(200.0, 0.0, 0.0, 0.0, 0.0, nan),
    Wrench(inf, 0.0, 0.0, 0.0, 0.0, 0.0),
    Wrench(0.0, -inf, 0.0, 0.0, 0.0, 0.0),
  };
  for (const WrenchVector & requested : bad_requests) {
    const AllocationResult result = allocator_.Allocate(requested);
    EXPECT_EQ(result.fault, AllocationFault::kNonFiniteRequest) << requested.transpose();
    EXPECT_TRUE(result.forces.isZero(0.0)) << result.forces.transpose();
    EXPECT_TRUE(result.achieved.isZero(0.0));
    EXPECT_FALSE(result.saturated);
  }
}

// Regression: a finite request so large that the solver overflows
// (W * tau = 10 * 1e308 = infinity) must also give zero thrust and a fault.
// Before the fix, the NaN solution was clamped into limit values.
TEST_F(ThrustAllocatorTest, OverflowingSolverResultGivesZeroThrustAndFault)
{
  const WrenchVector requested = Wrench(0.0, 0.0, 0.0, 1e308, 0.0, 0.0);
  ASSERT_TRUE(requested.allFinite());
  const AllocationResult result = allocator_.Allocate(requested);
  EXPECT_NE(result.fault, AllocationFault::kNone);
  EXPECT_TRUE(result.forces.isZero(0.0)) << result.forces.transpose();
  EXPECT_FALSE(result.saturated);
  EXPECT_TRUE(result.residual == requested);
}

TEST(ThrustAllocatorSettingsTest, RejectsNonFiniteMatrixAndSettings)
{
  AllocationMatrix matrix = AllocationMatrix::Identity();
  matrix(2, 3) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ThrustAllocator(matrix, AllocatorSettings()), std::invalid_argument);

  AllocatorSettings settings;
  settings.max_forward_force = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ThrustAllocator(AllocationMatrix::Identity(), settings), std::invalid_argument);
  settings = AllocatorSettings();
  settings.regularization = std::numeric_limits<double>::infinity();
  EXPECT_THROW(ThrustAllocator(AllocationMatrix::Identity(), settings), std::invalid_argument);
}

TEST(ThrustAllocatorSettingsTest, RejectsInvalidSettings)
{
  const AllocationMatrix matrix = AllocationMatrix::Identity();
  AllocatorSettings settings;
  settings.max_reverse_force = 0.0;
  EXPECT_THROW(ThrustAllocator(matrix, settings), std::invalid_argument);
  settings = AllocatorSettings();
  settings.weights(3) = -1.0;
  EXPECT_THROW(ThrustAllocator(matrix, settings), std::invalid_argument);
}

TEST(ThrusterLayoutTest, MissingFileThrows)
{
  EXPECT_THROW(LoadThrusterLayout("/nonexistent/thrusters.yaml"), std::runtime_error);
}

}  // namespace
}  // namespace rov_control
