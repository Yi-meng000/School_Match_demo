#include "robot_control/mpc_controller.hpp"
#include "robot_control/tracing_adapter.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace rt = robot_control::trajectory;

namespace
{

std::vector<robot_control::TrajectoryPoint> stationaryReference(
  int horizon, double x, double y, double yaw = 0.0)
{
  std::vector<robot_control::TrajectoryPoint> reference(
    static_cast<std::size_t>(horizon));
  for (auto& point : reference) {
    point.x = x;
    point.y = y;
    point.yaw = yaw;
  }
  return reference;
}

std::vector<robot_control::TrajectoryPoint> constantVelocityReference(
  int horizon, double dt, double x0, double y0, double yaw,
  double vx, double vy, double vw = 0.0)
{
  std::vector<robot_control::TrajectoryPoint> reference(
    static_cast<std::size_t>(horizon));
  for (int i = 0; i < horizon; ++i) {
    const double time = dt * static_cast<double>(i + 1);
    auto& point = reference[static_cast<std::size_t>(i)];
    point.x = x0 + vx * time;
    point.y = y0 + vy * time;
    point.yaw = yaw + vw * time;
    point.vx = vx;
    point.vy = vy;
    point.vw = vw;
  }
  return reference;
}

std::vector<robot_control::TrajectoryPoint> acceleratingStraightReference(
  int horizon, double dt, double acceleration)
{
  std::vector<robot_control::TrajectoryPoint> reference(
    static_cast<std::size_t>(horizon));
  for (int i = 0; i < horizon; ++i) {
    const double time = dt * static_cast<double>(i + 1);
    auto& point = reference[static_cast<std::size_t>(i)];
    point.x = 0.5 * acceleration * time * time;
    point.vx = acceleration * time;
    point.ax = acceleration;
  }
  return reference;
}

void advanceFirstOrder(
  robot_control::State* state,
  const robot_control::ControlCmd& command,
  double dt,
  const Eigen::Vector3d& tau)
{
  const Eigen::Vector3d decay =
    (-Eigen::Vector3d::Constant(dt).cwiseQuotient(tau)).array().exp().matrix();
  const Eigen::Vector3d velocity_from_state =
    tau.cwiseProduct(Eigen::Vector3d::Ones() - decay);
  const Eigen::Vector3d position_from_command =
    Eigen::Vector3d::Constant(dt) - velocity_from_state;
  const Eigen::Vector3d position(state->x, state->y, state->yaw);
  const Eigen::Vector3d velocity(state->vx, state->vy, state->vw);
  const Eigen::Vector3d input(command.vx, command.vy, command.vw);
  const Eigen::Vector3d next_position =
    position + velocity_from_state.cwiseProduct(velocity) +
    position_from_command.cwiseProduct(input);
  const Eigen::Vector3d next_velocity =
    decay.cwiseProduct(velocity) +
    (Eigen::Vector3d::Ones() - decay).cwiseProduct(input);
  state->x = next_position.x();
  state->y = next_position.y();
  state->yaw = next_position.z();
  state->vx = next_velocity.x();
  state->vy = next_velocity.y();
  state->vw = next_velocity.z();
}

}  // namespace

TEST(MpcControllerTest, RejectsInvalidConfigurationAndWeights)
{
  EXPECT_THROW(
    robot_control::MpcController(
      20, 0.05, Eigen::Vector3d(0.1, 0.0, 0.1)),
    std::invalid_argument);

  robot_control::MpcController mpc(20, 0.05);
  Eigen::Matrix<double, 6, 1> q =
    Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Vector3d correction = Eigen::Vector3d::Ones();
  Eigen::Vector3d r = Eigen::Vector3d::Ones();
  std::string error;
  q(2) = -1.0;
  EXPECT_FALSE(mpc.setWeights(q, correction, r, &error));
  q(2) = 1.0;
  correction(1) = -1.0;
  EXPECT_FALSE(mpc.setWeights(q, correction, r, &error));
  correction(1) = 1.0;
  r(0) = 0.0;
  EXPECT_FALSE(mpc.setWeights(q, correction, r, &error));
}

TEST(MpcControllerTest, PlanarVelocityContractIsWorldFrameAtNonzeroYaw)
{
  constexpr int kHorizon = 20;
  constexpr double kDt = 0.05;
  constexpr double kYaw = 1.57079632679489661923;
  const rt::Vector2 measured_world =
    robot_control::tracing::bodyVelocityToWorld(kYaw, 0.0, -0.4);

  robot_control::State state;
  state.yaw = kYaw;
  state.vx = measured_world.x;
  state.vy = measured_world.y;
  const auto reference = constantVelocityReference(
    kHorizon, kDt, state.x, state.y, kYaw, 0.4, 0.0);
  robot_control::ControlCmd previous;
  previous.vx = 0.4;

  robot_control::MpcController mpc(kHorizon, kDt);
  robot_control::ControlCmd command;
  ASSERT_TRUE(mpc.solveMPC(state, reference, previous, command));
  EXPECT_NEAR(command.vx, 0.4, 1e-10);
  EXPECT_NEAR(command.vy, 0.0, 1e-10);

  const rt::Vector2 command_body =
    robot_control::tracing::worldVelocityToBody(
    kYaw, {command.vx, command.vy});
  EXPECT_NEAR(command_body.x, 0.0, 1e-10);
  EXPECT_NEAR(command_body.y, -0.4, 1e-10);
}

TEST(MpcControllerTest, StraightStartupFirstCommandPointsAlongReference)
{
  constexpr int kHorizon = 20;
  constexpr double kDt = 0.05;
  robot_control::MpcController mpc(kHorizon, kDt);
  robot_control::State state;
  robot_control::ControlCmd previous;
  robot_control::ControlCmd command;
  const auto reference =
    acceleratingStraightReference(kHorizon, kDt, 1.0);

  ASSERT_TRUE(mpc.solveMPC(state, reference, previous, command));
  EXPECT_GT(command.vx, 0.0);
  EXPECT_NEAR(command.vy, 0.0, 1e-10);

  robot_control::ControlCmd repeated;
  ASSERT_TRUE(mpc.solveMPC(state, reference, previous, repeated));
  EXPECT_GT(repeated.vx, 0.0);
  EXPECT_NEAR(repeated.vx, command.vx, 1e-12);
}

TEST(MpcControllerTest, RealTrajectoryGeneratorCannotRepeatAReverseStartup)
{
  constexpr int kHorizon = 20;
  constexpr double kDt = 0.05;
  rt::PathGeometry path;
  std::string error;
  ASSERT_TRUE(path.build({{0.0, 0.0}, {5.0, 0.0}}, &error)) << error;

  rt::SpeedProfileOptions speed;
  speed.cruise_speed = 1.8;
  speed.nominal_accel = 4.0;
  speed.nominal_decel = 4.0;
  speed.accel_fraction = 0.20;
  speed.decel_fraction = 0.25;
  rt::TrajectoryGenerator generator(speed);
  rt::MotionState2D measured_state;
  ASSERT_TRUE(generator.activatePath(path, measured_state, &error)) << error;

  const rt::HeadingProvider fixed_yaw = [](double, double) {
      return rt::HeadingReference{true, 0.0, 0.0, 0.0};
    };
  robot_control::MpcController mpc(kHorizon, kDt);
  robot_control::ControlCmd previous;
  for (int repetition = 0; repetition < 3; ++repetition) {
    std::vector<rt::ReferencePoint> horizon;
    ASSERT_EQ(
      generator.makeHorizon(
        measured_state, kDt, kHorizon, &horizon, fixed_yaw),
      rt::TrajectoryStatus::Ready);
    ASSERT_EQ(horizon.size(), static_cast<std::size_t>(kHorizon));

    std::vector<robot_control::TrajectoryPoint> reference;
    reference.reserve(horizon.size());
    for (const auto& point : horizon) {
      robot_control::TrajectoryPoint mpc_point;
      mpc_point.x = point.position.x;
      mpc_point.y = point.position.y;
      mpc_point.yaw = point.heading.yaw;
      mpc_point.vx = point.velocity.x;
      mpc_point.vy = point.velocity.y;
      mpc_point.vw = point.heading.angular_velocity;
      mpc_point.ax = point.acceleration.x;
      mpc_point.ay = point.acceleration.y;
      mpc_point.aw = point.heading.angular_acceleration;
      reference.push_back(mpc_point);
    }

    robot_control::ControlCmd command;
    robot_control::State state;
    ASSERT_TRUE(mpc.solveMPC(state, reference, previous, command));
    EXPECT_GT(command.vx, 0.0);
    EXPECT_NEAR(command.vy, 0.0, 1e-10);
    // Leave the measured state unchanged, but keep the command history exactly
    // as tracing_node does. This reproduces a chassis that has not yet broken
    // static friction or its command deadzone.
    previous = command;
  }
}

TEST(MpcControllerTest, FuturePositionsChangeTheFirstCommand)
{
  constexpr int kHorizon = 20;
  robot_control::MpcController mpc(kHorizon, 0.05);
  robot_control::State state;
  robot_control::ControlCmd previous;
  robot_control::ControlCmd stationary_command;
  robot_control::ControlCmd future_command;
  const auto stationary = stationaryReference(kHorizon, 0.0, 0.0);
  auto future = stationary;
  for (int i = 5; i < kHorizon; ++i) {
    future[static_cast<std::size_t>(i)].x =
      0.02 * static_cast<double>(i - 4);
  }

  ASSERT_TRUE(mpc.solveMPC(
    state, stationary, previous, stationary_command));
  ASSERT_TRUE(mpc.solveMPC(
    state, future, previous, future_command));
  EXPECT_NEAR(stationary_command.vx, 0.0, 1e-12);
  EXPECT_GT(future_command.vx, stationary_command.vx + 1e-6);
}

TEST(MpcControllerTest, CorrectionWeightKeepsCommandCloserToFeedforward)
{
  constexpr int kHorizon = 20;
  const auto reference = stationaryReference(kHorizon, 1.0, 0.0);
  robot_control::State state;
  robot_control::ControlCmd previous;
  Eigen::Matrix<double, 6, 1> q;
  q << 120.0, 120.0, 90.0, 20.0, 20.0, 2.0;
  const Eigen::Vector3d r(1.5, 1.5, 0.8);
  std::string error;

  robot_control::MpcController weak_correction_penalty(kHorizon, 0.05);
  robot_control::MpcController strong_correction_penalty(kHorizon, 0.05);
  ASSERT_TRUE(weak_correction_penalty.setWeights(
    q, Eigen::Vector3d::Zero(), r, &error)) << error;
  ASSERT_TRUE(strong_correction_penalty.setWeights(
    q, Eigen::Vector3d::Constant(1000.0), r, &error)) << error;

  robot_control::ControlCmd weak_command;
  robot_control::ControlCmd strong_command;
  ASSERT_TRUE(weak_correction_penalty.solveMPC(
    state, reference, previous, weak_command));
  ASSERT_TRUE(strong_correction_penalty.solveMPC(
    state, reference, previous, strong_command));
  EXPECT_GT(std::fabs(weak_command.vx), 1e-6);
  EXPECT_LT(std::fabs(strong_command.vx), std::fabs(weak_command.vx));
}

TEST(MpcControllerTest, FixedYawErrorProducesCorrectYawRateDirection)
{
  robot_control::State state;
  state.yaw = 0.20;
  robot_control::MpcController mpc(20, 0.05);
  robot_control::ControlCmd previous;
  robot_control::ControlCmd command;
  ASSERT_TRUE(mpc.solveMPC(
    state, stationaryReference(20, 0.0, 0.0, 0.0),
    previous, command));
  EXPECT_NEAR(command.vx, 0.0, 1e-10);
  EXPECT_NEAR(command.vy, 0.0, 1e-10);
  EXPECT_LT(command.vw, 0.0);
}

TEST(MpcControllerTest, LargePositionErrorIsNotClipped)
{
  robot_control::MpcController mpc(20, 0.05);
  robot_control::State state;
  robot_control::ControlCmd previous;
  robot_control::ControlCmd small;
  robot_control::ControlCmd large;
  ASSERT_TRUE(mpc.solveMPC(
    state, stationaryReference(20, 1.0, 0.0), previous, small));
  ASSERT_TRUE(mpc.solveMPC(
    state, stationaryReference(20, 10.0, 0.0), previous, large));
  ASSERT_GT(std::fabs(small.vx), 1e-9);
  EXPECT_NEAR(large.vx / small.vx, 10.0, 1e-8);
}

TEST(MpcControllerTest, PlanarSolutionHasNoOctagonalDirectionLimit)
{
  robot_control::MpcController mpc(20, 0.05);
  robot_control::State state;
  robot_control::ControlCmd previous;
  robot_control::ControlCmd axis;
  robot_control::ControlCmd diagonal;
  ASSERT_TRUE(mpc.solveMPC(
    state, stationaryReference(20, 2.0, 0.0), previous, axis));
  const double component = std::sqrt(2.0);
  ASSERT_TRUE(mpc.solveMPC(
    state, stationaryReference(20, component, component),
    previous, diagonal));
  EXPECT_NEAR(
    std::hypot(diagonal.vx, diagonal.vy), std::fabs(axis.vx), 1e-8);
  EXPECT_NEAR(diagonal.vx, diagonal.vy, 1e-10);
}

TEST(MpcControllerTest, LdltSolutionSatisfiesFirstOrderOptimality)
{
  robot_control::MpcController mpc(20, 0.05);
  robot_control::State state;
  state.x = -0.7;
  state.y = 0.2;
  state.yaw = -0.3;
  state.vx = 0.4;
  state.vy = -0.1;
  std::vector<robot_control::TrajectoryPoint> reference(20);
  for (int i = 0; i < 20; ++i) {
    auto& point = reference[static_cast<std::size_t>(i)];
    point.x = 0.03 * static_cast<double>(i + 1);
    point.y = 0.01 * static_cast<double>(i + 1);
    point.yaw = 0.1;
    point.vx = 0.6 + 0.01 * static_cast<double>(i);
    point.vy = 0.2;
    point.ax = 0.2;
  }

  robot_control::ControlCmd previous;
  previous.vx = 0.3;
  robot_control::ControlCmd command;
  ASSERT_TRUE(mpc.solveMPC(state, reference, previous, command));
  EXPECT_LT(mpc.lastOptimalityResidual(), 1e-8);
  EXPECT_TRUE(std::isfinite(command.vx));
  EXPECT_TRUE(std::isfinite(command.vy));
  EXPECT_TRUE(std::isfinite(command.vw));
}

TEST(MpcControllerTest, RepeatedClosedLoopSolvesRemainFinite)
{
  constexpr int kHorizon = 20;
  constexpr double kDt = 0.05;
  const Eigen::Vector3d tau(0.10, 0.10, 0.10);
  robot_control::MpcController mpc(kHorizon, kDt, tau);
  robot_control::State state;
  robot_control::ControlCmd previous;
  for (int iteration = 0; iteration < 100; ++iteration) {
    const auto reference = constantVelocityReference(
      kHorizon, kDt, state.x, state.y, 0.2, 0.5, -0.2);
    robot_control::ControlCmd command;
    ASSERT_TRUE(mpc.solveMPC(state, reference, previous, command));
    ASSERT_TRUE(std::isfinite(command.vx));
    ASSERT_TRUE(std::isfinite(command.vy));
    ASSERT_TRUE(std::isfinite(command.vw));
    advanceFirstOrder(&state, command, kDt, tau);
    previous = command;
  }
}
