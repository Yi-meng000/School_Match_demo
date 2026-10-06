#include "robot_control/translational_trajectory.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace rt = robot_control::trajectory;

namespace
{

rt::SpeedProfileOptions speedOptions()
{
  rt::SpeedProfileOptions options;
  options.cruise_speed = 1.0;
  options.nominal_accel = 2.0;
  options.nominal_decel = 2.0;
  options.accel_fraction = 0.20;
  options.decel_fraction = 0.25;
  return options;
}

rt::PathGeometry buildPath(const std::vector<rt::Point2>& points)
{
  rt::PathGeometry path;
  std::string error;
  EXPECT_TRUE(path.build(points, &error)) << error;
  return path;
}

rt::MotionState2D stopped(double x = 0.0, double y = 0.0)
{
  return {{x, y}, {0.0, 0.0}};
}

}  // namespace

TEST(SpeedProfileOptionsTest, RejectsInvalidValues)
{
  rt::SpeedProfileOptions options = speedOptions();
  EXPECT_TRUE(options.isValid());
  options.nominal_decel = 0.0;
  EXPECT_FALSE(options.isValid());
  options = speedOptions();
  options.accel_fraction = -0.1;
  EXPECT_FALSE(options.isValid());
}

TEST(PathGeometryTest, RejectsNonFiniteAndCoincidentPaths)
{
  rt::PathGeometry path;
  std::string error;
  EXPECT_FALSE(path.build({{0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 1.0}}, &error));
  EXPECT_FALSE(path.build({{1.0, 2.0}, {1.0, 2.0}, {1.0, 2.0}}, &error));
  EXPECT_FALSE(path.valid());
}

TEST(PathGeometryTest, RemovesOnlyConsecutiveDuplicatesAndPreservesPlannerPoints)
{
  rt::PathGeometry path;
  std::string error;
  ASSERT_TRUE(path.build(
    {{0.0, 0.0}, {0.0, 0.0}, {0.10, 0.0}, {0.45, 0.20},
      {0.50, 0.80}, {1.30, 1.00}}, &error)) << error;

  ASSERT_EQ(path.size(), 5u);
  const auto& points = path.points();
  EXPECT_DOUBLE_EQ(points[0].x, 0.0);
  EXPECT_DOUBLE_EQ(points[1].x, 0.10);
  EXPECT_DOUBLE_EQ(points[2].x, 0.45);
  EXPECT_DOUBLE_EQ(points[2].y, 0.20);
  EXPECT_DOUBLE_EQ(points[3].x, 0.50);
  EXPECT_DOUBLE_EQ(points.back().x, 1.30);
  EXPECT_DOUBLE_EQ(points.back().y, 1.00);
}

TEST(PathGeometryTest, SamplesArcLengthWithoutMovingEndpoints)
{
  const rt::PathGeometry path = buildPath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 2.0}});
  EXPECT_NEAR(path.length(), 3.0, 1e-12);
  const rt::PathSample start = path.sample(-1.0);
  const rt::PathSample middle = path.sample(1.5);
  const rt::PathSample end = path.sample(10.0);
  EXPECT_NEAR(start.position.x, 0.0, 1e-12);
  EXPECT_NEAR(start.position.y, 0.0, 1e-12);
  EXPECT_NEAR(middle.position.x, 1.0, 1e-12);
  EXPECT_NEAR(middle.position.y, 0.5, 1e-12);
  EXPECT_NEAR(end.position.x, 1.0, 1e-12);
  EXPECT_NEAR(end.position.y, 2.0, 1e-12);
  EXPECT_GT(std::fabs(path.sample(1.0).curvature), 0.1);
}

TEST(PathGeometryTest, LocalProjectionHintKeepsTheRequestedSelfIntersectionBranch)
{
  const rt::PathGeometry path = buildPath(
    {{0.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {2.0, 0.0}});
  const rt::Projection global = path.project({1.0, 1.0});
  const rt::Projection later_branch = path.project({1.0, 1.0}, 6.2, 0.6, 0.6);
  EXPECT_LT(global.arc_length, 2.0);
  EXPECT_GT(later_branch.arc_length, 5.5);
  EXPECT_NEAR(global.distance, 0.0, 1e-12);
  EXPECT_NEAR(later_branch.distance, 0.0, 1e-12);
}

TEST(TrajectoryGeneratorTest, LongPathAcceleratesCruisesAndStopsAtEndpoint)
{
  const rt::PathGeometry path = buildPath({{0.0, 0.0}, {10.0, 0.0}});
  rt::TrajectoryGenerator generator(speedOptions());
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, stopped(), &error)) << error;

  EXPECT_NEAR(generator.desiredSpeed(0.0), 0.0, 1e-10);
  EXPECT_NEAR(generator.desiredSpeed(5.0), 1.0, 1e-8);
  EXPECT_NEAR(generator.desiredSpeed(10.0), 0.0, 1e-10);

  std::vector<rt::ReferencePoint> horizon;
  ASSERT_EQ(
    generator.makeHorizon(stopped(), 0.02, 1000, &horizon),
    rt::TrajectoryStatus::Ready);
  ASSERT_FALSE(horizon.empty());
  EXPECT_NEAR(horizon.back().position.x, 10.0, 1e-10);
  EXPECT_NEAR(horizon.back().speed, 0.0, 1e-10);
}

TEST(TrajectoryGeneratorTest, ShortPathUsesTriangularSineProfile)
{
  const rt::PathGeometry path = buildPath({{0.0, 0.0}, {0.20, 0.0}});
  rt::TrajectoryGenerator generator(speedOptions());
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, stopped(), &error)) << error;

  double maximum_speed = 0.0;
  for (int i = 0; i <= 100; ++i) {
    maximum_speed = std::max(
      maximum_speed, generator.desiredSpeed(0.002 * static_cast<double>(i)));
  }
  EXPECT_GT(maximum_speed, 0.1);
  EXPECT_LT(maximum_speed, speedOptions().cruise_speed);
  EXPECT_NEAR(generator.desiredSpeed(path.length()), 0.0, 1e-10);
}

TEST(TrajectoryGeneratorTest, HighInitialSpeedUsesWholePathAndOnlyReportsAggressiveDeceleration)
{
  const rt::PathGeometry path = buildPath({{0.0, 0.0}, {0.05, 0.0}});
  rt::TrajectoryGenerator generator(speedOptions());
  const rt::MotionState2D moving{{0.0, 0.0}, {1.0, 0.0}};
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, moving, &error)) << error;

  EXPECT_EQ(generator.diagnostics().status, rt::TrajectoryStatus::Ready);
  EXPECT_TRUE(generator.diagnostics().nominal_decel_exceeded);
  EXPECT_GT(
    generator.diagnostics().required_peak_deceleration,
    speedOptions().nominal_decel);
  EXPECT_NEAR(generator.desiredSpeed(0.0), 1.0, 1e-9);
  EXPECT_NEAR(generator.desiredSpeed(0.05), 0.0, 1e-9);

  std::vector<rt::ReferencePoint> horizon;
  EXPECT_EQ(
    generator.makeHorizon(moving, 0.01, 20, &horizon),
    rt::TrajectoryStatus::Ready);
  ASSERT_FALSE(horizon.empty());
  EXPECT_NEAR(horizon.back().speed, 0.0, 1e-9);
}

TEST(TrajectoryGeneratorTest, DesiredSpeedAtAnArcLengthNeverDependsOnLaterFeedback)
{
  const rt::PathGeometry path = buildPath({{0.0, 0.0}, {4.0, 0.0}});
  rt::TrajectoryGenerator generator(speedOptions());
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, stopped(), &error)) << error;
  const double expected = generator.desiredSpeed(1.5);

  std::vector<rt::ReferencePoint> horizon;
  const rt::MotionState2D too_fast{{1.5, 0.0}, {5.0, 0.0}};
  ASSERT_EQ(
    generator.makeHorizon(too_fast, 0.02, 20, &horizon),
    rt::TrajectoryStatus::Ready);
  EXPECT_NEAR(generator.diagnostics().speed_at_progress, expected, 1e-10);
  EXPECT_NEAR(generator.desiredSpeed(1.5), expected, 1e-10);

  const rt::MotionState2D stopped_same_place{{1.5, 0.0}, {0.0, 0.0}};
  ASSERT_EQ(
    generator.makeHorizon(stopped_same_place, 0.02, 20, &horizon),
    rt::TrajectoryStatus::Ready);
  EXPECT_NEAR(generator.desiredSpeed(1.5), expected, 1e-10);
}

TEST(TrajectoryGeneratorTest, ProgressIsMonotonic)
{
  const rt::PathGeometry path = buildPath({{0.0, 0.0}, {4.0, 0.0}});
  rt::TrajectoryGenerator generator(speedOptions());
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, stopped(1.0, 0.0), &error)) << error;

  std::vector<rt::ReferencePoint> horizon;
  ASSERT_EQ(
    generator.makeHorizon(stopped(2.0, 0.0), 0.05, 5, &horizon),
    rt::TrajectoryStatus::Ready);
  EXPECT_NEAR(generator.diagnostics().progress, 2.0, 1e-9);
  ASSERT_EQ(
    generator.makeHorizon(stopped(1.5, 0.0), 0.05, 5, &horizon),
    rt::TrajectoryStatus::Ready);
  EXPECT_NEAR(generator.diagnostics().progress, 2.0, 1e-9);
}

TEST(TrajectoryGeneratorTest, NewActivationUsesNewProjectionAndMeasuredTangentSpeed)
{
  const rt::PathGeometry first = buildPath({{0.0, 0.0}, {4.0, 0.0}});
  const rt::PathGeometry second = buildPath({{0.5, 0.0}, {0.5, 3.0}});
  rt::TrajectoryGenerator generator(speedOptions());
  std::string error;
  ASSERT_TRUE(generator.activatePath(first, stopped(), &error)) << error;

  const rt::MotionState2D state{{0.5, 0.4}, {0.0, 0.7}};
  ASSERT_TRUE(generator.activatePath(second, state, &error)) << error;
  EXPECT_NEAR(generator.profileStartArcLength(), 0.4, 1e-9);
  EXPECT_NEAR(generator.desiredSpeed(0.4), 0.7, 1e-9);
}

TEST(TrajectoryGeneratorTest, RejectsPathTooFarFromVehicle)
{
  const rt::PathGeometry path = buildPath({{2.0, 0.0}, {3.0, 0.0}});
  rt::TrajectoryGenerator generator(speedOptions());
  std::string error;
  EXPECT_FALSE(generator.activatePath(path, stopped(), &error));
  EXPECT_EQ(generator.diagnostics().status, rt::TrajectoryStatus::PathRejected);
}

TEST(TrajectoryGeneratorTest, TransparentlyForwardsExternalHeading)
{
  const rt::PathGeometry path = buildPath({{0.0, 0.0}, {2.0, 0.0}});
  rt::TrajectoryGenerator generator(speedOptions());
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, stopped(), &error)) << error;

  const rt::HeadingProvider heading = [](double time, double arc_length) {
      return rt::HeadingReference{true, 0.2 + time, arc_length, -0.5};
    };
  std::vector<rt::ReferencePoint> horizon;
  ASSERT_EQ(
    generator.makeHorizon(stopped(), 0.05, 3, &horizon, heading),
    rt::TrajectoryStatus::Ready);
  ASSERT_EQ(horizon.size(), 3u);
  EXPECT_TRUE(horizon[0].heading.valid);
  EXPECT_NEAR(horizon[0].heading.yaw, 0.25, 1e-12);
  EXPECT_NEAR(horizon[0].heading.angular_velocity, horizon[0].arc_length, 1e-12);
  EXPECT_NEAR(horizon[0].heading.angular_acceleration, -0.5, 1e-12);
}

TEST(TrajectoryGeneratorTest, AccelerationContainsTangentialAndNormalTerms)
{
  const rt::PathGeometry path = buildPath(
    {{0.0, 0.0}, {1.0, 0.0}, {1.7, 0.3}, {2.0, 1.0}, {2.0, 3.0}});
  rt::TrajectoryGenerator generator(speedOptions());
  std::string error;
  ASSERT_TRUE(generator.activatePath(path, stopped(), &error)) << error;

  std::vector<rt::ReferencePoint> horizon;
  ASSERT_EQ(
    generator.makeHorizon(stopped(), 0.02, 500, &horizon),
    rt::TrajectoryStatus::Ready);
  const auto curved = std::find_if(
    horizon.begin(), horizon.end(), [](const rt::ReferencePoint& point) {
      return std::fabs(point.curvature) > 0.05 && point.speed > 0.1;
    });
  ASSERT_NE(curved, horizon.end());
  const rt::PathSample sample = path.sample(curved->arc_length);
  const rt::Vector2 normal{-sample.tangent.y, sample.tangent.x};
  const double normal_component =
    curved->acceleration.x * normal.x + curved->acceleration.y * normal.y;
  EXPECT_NEAR(
    normal_component, curved->curvature * curved->speed * curved->speed,
    1e-8);
}
