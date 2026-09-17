#include "robot_control/translational_trajectory.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace rt = robot_control::trajectory;

namespace
{

rt::MotionLimits limits()
{
  rt::MotionLimits value;
  value.cruise_speed = 1.0;
  value.max_accel = 1.5;
  value.normal_decel = 1.5;
  value.emergency_decel = 5.0;
  value.max_lateral_accel = 1.0;
  value.terminal_speed = 0.0;
  value.accel_fraction = 0.20;
  value.decel_fraction = 0.25;
  return value;
}

rt::PathBuildOptions pathOptions()
{
  rt::PathBuildOptions options;
  options.sample_spacing = 0.02;
  options.rdp_epsilon = 0.005;
  options.smooth_half_window = 0.08;
  options.max_smooth_deviation = 0.15;
  options.max_smoothing_attempts = 6;
  return options;
}

rt::PathGeometry buildPath(
  const std::vector<rt::Point2> & points,
  const rt::PathBuildOptions & options = pathOptions())
{
  rt::PathGeometry path;
  std::string error;
  EXPECT_TRUE(path.build(points, options, &error)) << error;
  return path;
}

rt::MotionState2D stoppedState()
{
  return {{0.0, 0.0}, {0.0, 0.0}};
}

}  // namespace

TEST(MotionLimitsTest, RequiresAllPhysicalLimitsAndIndependentEmergencyBrake)
{
  rt::MotionLimits value;
  std::string error;
  EXPECT_FALSE(value.isValid(&error));
  EXPECT_FALSE(error.empty());

  value = limits();
  value.emergency_decel = 1.0;
  EXPECT_FALSE(value.isValid(&error));

  value = limits();
  EXPECT_TRUE(value.isValid(&error)) << error;
}

TEST(PathGeometryTest, RejectsInvalidAndDegenerateInput)
{
  rt::PathGeometry path;
  std::string error;
  EXPECT_FALSE(path.build({{0.0, 0.0}, {0.0, 0.0}}, pathOptions(), &error));
  EXPECT_FALSE(error.empty());

  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(path.build({{0.0, 0.0}, {nan, 1.0}}, pathOptions(), &error));
}

TEST(PathGeometryTest, RemovesConsecutiveDuplicatesAndHonorsProjectionHint)
{
  rt::PathGeometry duplicate_path;
  std::string error;
  ASSERT_TRUE(
    duplicate_path.build(
      {{0.0, 0.0}, {0.0, 0.0}, {0.5, 0.0},
        {0.5, 0.0}, {1.0, 0.0}}, pathOptions(), &error)) << error;
  EXPECT_NEAR(duplicate_path.length(), 1.0, 1e-6);

  rt::PathBuildOptions crossing_options = pathOptions();
  crossing_options.rdp_epsilon = 0.0;
  crossing_options.smooth_half_window = 0.0;
  crossing_options.max_smooth_deviation = 0.02;
  const rt::PathGeometry crossing = buildPath(
    {{0.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, {1.0, 0.0}}, crossing_options);
  const rt::Projection first_branch = crossing.project({0.5, 0.5}, 0.1, 0.2, 0.8);
  const rt::Projection last_branch = crossing.project({0.5, 0.5}, 3.0, 0.8, 0.8);
  EXPECT_LT(first_branch.arc_length, 1.0);
  EXPECT_GT(last_branch.arc_length, 2.5);
}

TEST(PathGeometryTest, PreservesEndpointsAndBoundsDeviation)
{
  const std::vector<rt::Point2> stair{{0.0, 0.0}, {0.025, 0.0}, {0.025, 0.025},
    {0.05, 0.025}, {0.05, 0.05}, {0.40, 0.05}};
  rt::PathBuildOptions options = pathOptions();
  options.rdp_epsilon = 0.001;
  options.max_smooth_deviation = 0.10;
  const rt::PathGeometry path = buildPath(stair, options);

  const rt::PathSample first = path.sample(0.0);
  const rt::PathSample last = path.sample(path.length());
  EXPECT_NEAR(first.position.x, stair.front().x, 1e-9);
  EXPECT_NEAR(first.position.y, stair.front().y, 1e-9);
  EXPECT_NEAR(last.position.x, stair.back().x, 1e-9);
  EXPECT_NEAR(last.position.y, stair.back().y, 1e-9);
  EXPECT_LE(path.maxSmoothDeviation(), options.max_smooth_deviation + 1e-9);
}

TEST(TrajectoryGeneratorTest, StraightPathStopsWithBoundedAcceleration)
{
  const rt::PathGeometry path = buildPath({{0.0, 0.0}, {4.0, 0.0}});
  rt::TrajectoryGenerator generator(limits());
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, stoppedState(), &error)) << error;

  std::vector<rt::ReferencePoint> reference;
  EXPECT_EQ(
    generator.makeHorizon(stoppedState(), 0.02, 700, &reference),
    rt::TrajectoryStatus::Ready);
  ASSERT_FALSE(reference.empty());
  EXPECT_FALSE(reference.front().heading.valid);

  double previous_speed = 0.0;
  for (const rt::ReferencePoint & point : reference) {
    EXPECT_LE(std::fabs(point.tangential_acceleration), limits().max_accel + 1e-6);
    EXPECT_NEAR(point.velocity.y, 0.0, 1e-8);
    EXPECT_NEAR(point.acceleration.y, 0.0, 1e-6);
    EXPECT_GE(point.speed, -1e-9);
    EXPECT_LE(std::fabs(point.speed - previous_speed), limits().max_accel * 0.02 + 1e-4);
    previous_speed = point.speed;
  }
  EXPECT_NEAR(reference.back().position.x, 4.0, 2e-3);
  EXPECT_NEAR(reference.back().speed, 0.0, 2e-3);
}

TEST(TrajectoryGeneratorTest, PersistentNominalPhaseEscapesStationaryStartup)
{
  const rt::PathGeometry path = buildPath({{0.0, 0.0}, {4.0, 0.0}});
  rt::GeneratorOptions options;
  options.max_reference_lead = 0.05;
  rt::TrajectoryGenerator generator(limits(), options);
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, stoppedState(), &error)) << error;

  std::vector<double> first_speeds;
  std::vector<rt::ReferencePoint> reference;
  for (int tick = 0; tick < 12; ++tick) {
    ASSERT_EQ(
      generator.makeHorizon(stoppedState(), 0.05, 1, &reference),
      rt::TrajectoryStatus::Ready);
    ASSERT_EQ(reference.size(), 1u);
    first_speeds.push_back(reference.front().speed);
    // Position is re-anchored to the measured projection every cycle instead
    // of accumulating the nominal profile's lead while the vehicle is stuck.
    EXPECT_LT(reference.front().arc_length, 0.01);
  }

  EXPECT_GT(first_speeds[5], first_speeds.front() + 0.05);
  EXPECT_NEAR(first_speeds.back(), limits().max_accel * 0.05, 2e-3);
  EXPECT_NEAR(generator.diagnostics().progress, 0.0, 1e-12);

  for (int tick = 0; tick < 200; ++tick) {
    ASSERT_EQ(
      generator.makeHorizon(stoppedState(), 0.05, 1, &reference),
      rt::TrajectoryStatus::Ready);
    ASSERT_EQ(reference.size(), 1u);
    EXPECT_LT(reference.front().arc_length, 0.01);
    EXPECT_LE(reference.front().speed, limits().max_accel * 0.05 + 1e-6);
  }
}

TEST(TrajectoryGeneratorTest, OverspeedAboveCruiseBrakesWithoutFalseEmergencyStop)
{
  const rt::PathGeometry path = buildPath({{0.0, 0.0}, {2.0, 0.0}});
  rt::TrajectoryGenerator generator(limits());
  // The vehicle is only 15% above its 1.0 m/s cruise target and has ample
  // straight distance to decelerate.  This is not an emergency-infeasible
  // condition, even though the measured start speed exceeds cruise_speed.
  const rt::MotionState2D overspeed{{0.0, 0.0}, {1.15, 0.0}};
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, overspeed, &error)) << error;

  std::vector<rt::ReferencePoint> reference;
  EXPECT_EQ(
    generator.makeHorizon(overspeed, 0.02, 20, &reference),
    rt::TrajectoryStatus::Ready);
  ASSERT_FALSE(reference.empty());
  EXPECT_LT(reference.back().speed, overspeed.velocity.x);
  EXPECT_GT(reference.back().speed, 0.0);
}

TEST(TrajectoryGeneratorTest, RecedingHorizonReachesCruiseAndStops)
{
  const rt::PathGeometry path = buildPath({{0.0, 0.0}, {4.0, 0.0}});
  rt::TrajectoryGenerator generator(limits());
  rt::MotionState2D state = stoppedState();
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, state, &error)) << error;

  double maximum_speed = 0.0;
  bool reached_end = false;
  std::vector<rt::ReferencePoint> reference;
  for (int tick = 0; tick < 1000; ++tick) {
    const rt::TrajectoryStatus status = generator.makeHorizon(state, 0.02, 20, &reference);
    ASSERT_NE(status, rt::TrajectoryStatus::EmergencyInfeasible)
      << "normal_stop=" << generator.diagnostics().normal_stop_distance
      << " emergency_stop=" << generator.diagnostics().emergency_stop_distance
      << " deficit=" << generator.diagnostics().stop_deficit
      << " terminal=" << generator.diagnostics().terminal_speed_if_unstoppable;
    ASSERT_FALSE(reference.empty());
    state.position = reference.front().position;
    state.velocity = reference.front().velocity;
    maximum_speed = std::max(maximum_speed, reference.front().speed);
    if (generator.diagnostics().remaining_length < 1e-3 && reference.front().speed < 1e-3) {
      reached_end = true;
      break;
    }
  }

  EXPECT_TRUE(reached_end);
  EXPECT_GT(maximum_speed, 0.90);
  EXPECT_NEAR(state.position.x, 4.0, 2e-3);
  EXPECT_NEAR(std::hypot(state.velocity.x, state.velocity.y), 0.0, 2e-3);
}

TEST(TrajectoryGeneratorTest, CurvatureCapLimitsSpeed)
{
  rt::PathBuildOptions options = pathOptions();
  options.smooth_half_window = 0.12;
  options.max_smooth_deviation = 0.20;
  const rt::PathGeometry path =
    buildPath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}}, options);
  rt::TrajectoryGenerator generator(limits());
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, stoppedState(), &error)) << error;

  std::vector<rt::ReferencePoint> reference;
  ASSERT_EQ(
    generator.makeHorizon(stoppedState(), 0.02, 700, &reference),
    rt::TrajectoryStatus::Ready);

  bool observed_curve = false;
  for (const rt::ReferencePoint & point : reference) {
    const double curvature = std::fabs(point.curvature);
    if (curvature > 0.10) {
      observed_curve = true;
      const double cap = std::sqrt(limits().max_lateral_accel / curvature);
      EXPECT_LE(point.speed, cap + 2e-3);
      const double normal_accel = std::fabs(point.curvature) * point.speed * point.speed;
      EXPECT_LE(normal_accel, limits().max_lateral_accel + 2e-3);
    }
  }
  EXPECT_TRUE(observed_curve);
}

TEST(TrajectoryGeneratorTest, RecedingHorizonKeepsCurvatureAccelerationBounded)
{
  rt::PathBuildOptions options = pathOptions();
  options.smooth_half_window = 0.12;
  options.max_smooth_deviation = 0.20;
  const rt::PathGeometry path =
    buildPath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}}, options);
  rt::TrajectoryGenerator generator(limits());
  rt::MotionState2D state = stoppedState();
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, state, &error)) << error;

  bool observed_curve = false;
  std::vector<rt::ReferencePoint> reference;
  for (int tick = 0; tick < 800; ++tick) {
    SCOPED_TRACE(::testing::Message()
      << "tick=" << tick
      << " x=" << state.position.x
      << " y=" << state.position.y
      << " vx=" << state.velocity.x
      << " vy=" << state.velocity.y
      << " progress=" << generator.diagnostics().progress
      << " remaining=" << generator.diagnostics().remaining_length);
    const rt::TrajectoryStatus status = generator.makeHorizon(state, 0.02, 20, &reference);
    ASSERT_NE(status, rt::TrajectoryStatus::EmergencyInfeasible)
      << "normal_stop=" << generator.diagnostics().normal_stop_distance
      << " emergency_stop=" << generator.diagnostics().emergency_stop_distance
      << " deficit=" << generator.diagnostics().stop_deficit
      << " terminal=" << generator.diagnostics().terminal_speed_if_unstoppable;
    ASSERT_FALSE(reference.empty());
    for (const rt::ReferencePoint & point : reference) {
      if (std::fabs(point.curvature) > 0.10) {
        observed_curve = true;
      }
      EXPECT_LE(
        std::fabs(point.curvature) * point.speed * point.speed,
        limits().max_lateral_accel + 3e-3);
    }
    state.position = reference.front().position;
    state.velocity = reference.front().velocity;
    if (generator.diagnostics().remaining_length < 1e-3 && reference.front().speed < 1e-3) {
      break;
    }
  }
  EXPECT_TRUE(observed_curve);
}

TEST(TrajectoryGeneratorTest, HandlesNormalAndEmergencyBrakingSeparately)
{
  const rt::MotionState2D moving{{0.0, 0.0}, {1.0, 0.0}};
  const rt::PathGeometry recoverable = buildPath({{0.0, 0.0}, {0.40, 0.0}});
  rt::TrajectoryGenerator generator(limits());
  std::string error;
  ASSERT_TRUE(generator.activatePath(recoverable, moving, &error)) << error;
  std::vector<rt::ReferencePoint> reference;
  EXPECT_EQ(
    generator.makeHorizon(moving, 0.02, 100, &reference),
    rt::TrajectoryStatus::EmergencyBraking);
  EXPECT_GT(generator.diagnostics().stop_deficit, 0.0);

  const rt::PathGeometry impossible = buildPath({{0.0, 0.0}, {0.05, 0.0}});
  ASSERT_TRUE(generator.activatePath(impossible, moving, &error)) << error;
  EXPECT_EQ(
    generator.makeHorizon(moving, 0.02, 100, &reference),
    rt::TrajectoryStatus::EmergencyInfeasible);
  EXPECT_GT(generator.diagnostics().terminal_speed_if_unstoppable, 0.0);
}

TEST(TrajectoryGeneratorTest, RejectsFarReplanAndResetsAcceptedProgress)
{
  const rt::PathGeometry first = buildPath({{0.0, 0.0}, {2.0, 0.0}});
  const rt::PathGeometry nearby_replan = buildPath({{0.0, 0.0}, {0.0, 2.0}});
  const rt::PathGeometry far_replan = buildPath({{2.0, 2.0}, {3.0, 2.0}});
  const rt::MotionState2D state{{0.10, 0.0}, {0.0, 0.0}};
  rt::TrajectoryGenerator generator(limits());
  std::string error;
  ASSERT_TRUE(generator.activatePath(first, state, &error)) << error;
  std::vector<rt::ReferencePoint> reference;
  generator.makeHorizon(state, 0.05, 4, &reference);

  ASSERT_TRUE(generator.activatePath(nearby_replan, state, &error)) << error;
  EXPECT_NEAR(generator.diagnostics().progress, 0.0, 1e-8);
  EXPECT_FALSE(generator.activatePath(far_replan, state, &error));
  EXPECT_EQ(generator.diagnostics().status, rt::TrajectoryStatus::PathRejected);
}

TEST(TrajectoryGeneratorTest, TransparentlyForwardsExternalHeading)
{
  const rt::PathGeometry path = buildPath({{0.0, 0.0}, {2.0, 0.0}});
  rt::TrajectoryGenerator generator(limits());
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, stoppedState(), &error)) << error;

  const rt::HeadingProvider heading = [](double time_s, double arc_length) {
      return rt::HeadingReference{true, 0.2 + time_s, arc_length, -0.5};
    };
  std::vector<rt::ReferencePoint> reference;
  ASSERT_EQ(
    generator.makeHorizon(stoppedState(), 0.05, 3, &reference, heading),
    rt::TrajectoryStatus::Ready);
  ASSERT_EQ(reference.size(), 3u);
  EXPECT_TRUE(reference[0].heading.valid);
  EXPECT_NEAR(reference[0].heading.yaw, 0.25, 1e-9);
  EXPECT_NEAR(reference[0].heading.angular_velocity, reference[0].arc_length, 1e-9);
  EXPECT_NEAR(reference[0].heading.angular_acceleration, -0.5, 1e-9);
}
