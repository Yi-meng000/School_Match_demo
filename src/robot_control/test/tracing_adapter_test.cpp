#include "robot_control/tracing_adapter.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace rt = robot_control::trajectory;
namespace tracing = robot_control::tracing;

TEST(TracingAdapterTest, RotatesBodyVelocityIntoWorldFrame)
{
  const rt::Vector2 velocity = tracing::bodyVelocityToWorld(M_PI_2, 1.5, -0.25);
  EXPECT_NEAR(velocity.x, 0.25, 1e-12);
  EXPECT_NEAR(velocity.y, 1.5, 1e-12);
}

TEST(TracingAdapterTest, RotatesWorldReferenceIntoReferenceBodyFrame)
{
  const rt::Vector2 world_velocity{0.25, 1.5};
  const rt::Vector2 body_velocity = tracing::worldVelocityToBody(M_PI_2, world_velocity);
  EXPECT_NEAR(body_velocity.x, 1.5, 1e-12);
  EXPECT_NEAR(body_velocity.y, -0.25, 1e-12);
}

TEST(TracingAdapterTest, ExtractsPlanarYawFromQuaternion)
{
  const double half_yaw = M_PI / 4.0;
  EXPECT_NEAR(
    tracing::yawFromQuaternion(0.0, 0.0, std::sin(half_yaw), std::cos(half_yaw)),
    M_PI_2, 1e-12);
}
