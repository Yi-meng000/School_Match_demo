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

TEST(TracingAdapterTest, AppliesCurrentLidarYawAlignmentOffset)
{
  const double raw_yaw = tracing::yawFromQuaternion(
    0.0051228022112386916,
    0.0022843233499106738,
    0.7119072488514794,
    0.7022511002462406);
  const double corrected_yaw = tracing::normalizeYaw(raw_yaw - M_PI_2);
  const rt::Vector2 body_reference =
    tracing::worldVelocityToBody(corrected_yaw, rt::Vector2{1.0, 0.0});

  EXPECT_NEAR(raw_yaw * 180.0 / M_PI, 90.781218, 1e-6);
  EXPECT_NEAR(corrected_yaw * 180.0 / M_PI, 0.781218, 1e-6);
  EXPECT_GT(body_reference.x, 0.999);
  EXPECT_NEAR(body_reference.y, -std::sin(corrected_yaw), 1e-12);
}
