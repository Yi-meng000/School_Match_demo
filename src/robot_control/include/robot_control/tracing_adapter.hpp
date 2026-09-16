#pragma once

#include <cmath>

#include "robot_control/translational_trajectory.hpp"

namespace robot_control
{
namespace tracing
{

// Convert an Odometry twist expressed in the chassis frame to the shared world
// frame used by translational_trajectory.
inline trajectory::Vector2 bodyVelocityToWorld(
  double yaw, double velocity_x_body, double velocity_y_body)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * velocity_x_body - s * velocity_y_body,
    s * velocity_x_body + c * velocity_y_body};
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

}  // namespace tracing
}  // namespace robot_control
