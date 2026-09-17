#pragma once

#include <cmath>

#include "robot_control/translational_trajectory.hpp"

namespace robot_control
{
namespace tracing
{

// Rotate a planar vector counter-clockwise by angle.  This is used to express
// an odometry twist in the physical chassis frame when its reported child
// frame has a fixed yaw alignment offset.
inline trajectory::Vector2 rotatePlanarVelocity(
  double angle, double velocity_x, double velocity_y)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {c * velocity_x - s * velocity_y,
    s * velocity_x + c * velocity_y};
}

// Convert an Odometry twist expressed in the chassis frame to the shared world
// frame used by translational_trajectory.
inline trajectory::Vector2 bodyVelocityToWorld(
  double yaw, double velocity_x_body, double velocity_y_body)
{
  return rotatePlanarVelocity(yaw, velocity_x_body, velocity_y_body);
}

// MpcController's velocity reference is expressed in the reference chassis
// frame.  This is the inverse conversion of bodyVelocityToWorld.
inline trajectory::Vector2 worldVelocityToBody(
  double reference_yaw, const trajectory::Vector2 & velocity_world)
{
  const double c = std::cos(reference_yaw);
  const double s = std::sin(reference_yaw);
  return {c * velocity_world.x + s * velocity_world.y,
    -s * velocity_world.x + c * velocity_world.y};
}

inline double yawFromQuaternion(double x, double y, double z, double w)
{
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}

// Keep corrected yaw in the principal interval after applying a fixed sensor
// or frame-alignment offset.
inline double normalizeYaw(double yaw)
{
  constexpr double kTwoPi = 6.28318530717958647692;
  return std::remainder(yaw, kTwoPi);
}

}  // namespace tracing
}  // namespace robot_control
