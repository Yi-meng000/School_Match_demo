#include "robot_control/mpc_controller.hpp"
#include "robot_control/tracing_adapter.hpp"
#include "robot_control/translational_trajectory.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace rt = robot_control::trajectory;

TEST(TracingPipelineTest, StationaryStartupCommandGrowsWithinAccelerationLimit)
{
  rt::MotionLimits limits;
  limits.cruise_speed = 1.2;
  limits.max_accel = 2.0;
  limits.normal_decel = 2.0;
  limits.emergency_decel = 3.0;
  limits.max_lateral_accel = 3.0;
  limits.terminal_speed = 0.0;
  limits.accel_fraction = 0.20;
  limits.decel_fraction = 0.25;

  rt::PathGeometry path;
  rt::PathBuildOptions path_options;
  path_options.rdp_epsilon = 0.001;
  std::string error;
  ASSERT_TRUE(path.build({{0.0, 0.0}, {4.0, 0.0}}, path_options, &error)) << error;

  rt::TrajectoryGenerator generator(limits);
  const rt::MotionState2D stationary{{0.0, 0.0}, {0.0, 0.0}};
  ASSERT_TRUE(generator.activatePath(path, stationary, &error)) << error;

  constexpr int kHorizon = 20;
  constexpr double kDt = 0.05;
  robot_control::MpcController mpc(kHorizon, kDt);
  Eigen::Matrix<double, 6, 1> q_diag;
  q_diag << 120.0, 120.0, 90.0, 20.0, 20.0, 2.0;
  const Eigen::Vector3d r_diag(1.5, 1.5, 0.8);
  const Eigen::Vector3d acceleration_limits(2.0, 3.0, 4.0);
  mpc.setErrorLimits(0.30, 0.5236);
  mpc.setWeights(q_diag, r_diag, acceleration_limits);
  mpc.setReferenceRelativeSpeedLimit(0.05, limits.normal_decel);

  robot_control::State mpc_state;
  std::vector<double> vx_commands;
  for (int tick = 0; tick < 12; ++tick) {
    std::vector<rt::ReferencePoint> reference;
    ASSERT_EQ(
      generator.makeHorizon(stationary, kDt, kHorizon, &reference),
      rt::TrajectoryStatus::Ready);
    ASSERT_EQ(reference.size(), static_cast<std::size_t>(kHorizon));

    std::vector<robot_control::TrajectoryPoint> mpc_reference;
    mpc_reference.reserve(reference.size());
    for (const rt::ReferencePoint & point : reference) {
      const rt::Vector2 body_velocity =
        robot_control::tracing::worldVelocityToBody(0.0, point.velocity);
      robot_control::TrajectoryPoint mpc_point;
      mpc_point.x = point.position.x;
      mpc_point.y = point.position.y;
      mpc_point.yaw = 0.0;
      mpc_point.vx = body_velocity.x;
      mpc_point.vy = body_velocity.y;
      mpc_reference.push_back(mpc_point);
    }

    robot_control::ControlCmd command;
    ASSERT_TRUE(mpc.solveMPC(mpc_state, mpc_reference, command));
    vx_commands.push_back(command.vx);
    EXPECT_NEAR(command.vy, 0.0, 1e-6);
    EXPECT_NEAR(command.vw, 0.0, 1e-6);
    // OSQP absolute tolerance is 1e-4, so do not require a tighter numerical
    // bound from the returned floating-point solution.
    EXPECT_LE(command.vx, limits.max_accel * kDt + 1e-3);
  }

  ASSERT_FALSE(vx_commands.empty());
  EXPECT_GT(vx_commands.back(), vx_commands.front() + 0.05);
  EXPECT_GT(*std::max_element(vx_commands.begin(), vx_commands.end()), 0.08);
}

TEST(MpcControllerTest, ReferenceRelativeSpeedLimitPreventsPositionCatchupOverspeed)
{
  constexpr int kHorizon = 20;
  constexpr double kDt = 0.05;
  robot_control::MpcController mpc(kHorizon, kDt);
  Eigen::Matrix<double, 6, 1> q_diag;
  // Deliberately make position catch-up attractive.  The hard speed constraint
  // must still win over this cost preference.
  q_diag << 500.0, 500.0, 90.0, 1.0, 1.0, 2.0;
  const Eigen::Vector3d r_diag(1.5, 1.5, 0.8);
  const Eigen::Vector3d acceleration_limits(2.0, 2.0, 4.0);
  mpc.setErrorLimits(0.30, 0.5236);
  mpc.setWeights(q_diag, r_diag, acceleration_limits);
  mpc.setReferenceRelativeSpeedLimit(0.05, 2.0);

  robot_control::State state;
  state.x = -10.0;  // Keep the saturated position error trying to accelerate.
  state.vx = 0.60;
  std::vector<robot_control::TrajectoryPoint> reference(kHorizon);
  for (int i = 0; i < kHorizon; ++i) {
    reference[static_cast<std::size_t>(i)].x = 0.20 * kDt * static_cast<double>(i + 1);
    reference[static_cast<std::size_t>(i)].vx = 0.20;
  }

  robot_control::ControlCmd command;
  ASSERT_TRUE(mpc.solveMPC(state, reference, command));

  // The reference-related target is 0.20 + 0.05 m/s, but the initial
  // 0.60 m/s can only reduce by 2.0 * 0.05 = 0.10 m/s in one step.
  // Thus the first feasible cap is 0.50 m/s, not an instantaneous 0.25 m/s.
  EXPECT_LE(std::hypot(command.vx, command.vy), 0.50 + 2e-3);
  EXPECT_LT(command.vx, state.vx);
}
