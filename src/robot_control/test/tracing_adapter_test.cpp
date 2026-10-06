#include "robot_control/tracing_adapter.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace rt = robot_control::trajectory;
namespace tracing = robot_control::tracing;

TEST(TracingAdapterTest, RotatesBodyVelocityIntoWorldFrame)
{
  const rt::Vector2 velocity = tracing::bodyVelocityToWorld(
    1.57079632679489661923, 1.5, -0.25);
  EXPECT_NEAR(velocity.x, 0.25, 1e-12);
  EXPECT_NEAR(velocity.y, 1.5, 1e-12);
}

TEST(TracingAdapterTest, RotatesWorldCommandIntoCurrentBodyFrame)
{
  const rt::Vector2 body = tracing::worldVelocityToBody(
    1.57079632679489661923, {0.25, 1.5});
  EXPECT_NEAR(body.x, 1.5, 1e-12);
  EXPECT_NEAR(body.y, -0.25, 1e-12);
}

TEST(TracingAdapterTest, ExtractsPlanarYawFromQuaternion)
{
  const double half_yaw = 3.14159265358979323846 / 4.0;
  EXPECT_NEAR(
    tracing::yawFromQuaternion(
      0.0, 0.0, std::sin(half_yaw), std::cos(half_yaw)),
    1.57079632679489661923, 1e-12);
}

TEST(TracingAdapterTest, GoalRequiresArcEndpointAndSpeedConditionsTogether)
{
  EXPECT_TRUE(tracing::goalReached(0.02, 0.03, 0.04, 0.05, 0.05));
  EXPECT_FALSE(tracing::goalReached(0.06, 0.03, 0.04, 0.05, 0.05));
  EXPECT_FALSE(tracing::goalReached(0.02, 0.20, 0.04, 0.05, 0.05));
  EXPECT_FALSE(tracing::goalReached(0.02, 0.03, 0.10, 0.05, 0.05));
}
