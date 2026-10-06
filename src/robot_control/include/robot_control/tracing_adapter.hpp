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

// Convert a world-frame planar velocity command to the current chassis frame
// at the ROS/chassis boundary.  MpcController itself remains world-frame-only
// for planar position, velocity and velocity increments.
inline trajectory::Vector2 worldVelocityToBody(
  double current_yaw, const trajectory::Vector2 & velocity_world)
{
  const double c = std::cos(current_yaw);
  const double s = std::sin(current_yaw);
  return {c * velocity_world.x + s * velocity_world.y,
    -s * velocity_world.x + c * velocity_world.y};
}

inline double yawFromQuaternion(double x, double y, double z, double w)
{
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}

inline double planarDistance(
  const trajectory::Point2 & first, const trajectory::Point2 & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

inline bool goalReached(
  double remaining_distance, double endpoint_distance, double measured_speed,
  double position_tolerance, double speed_tolerance)
{
  return remaining_distance <= position_tolerance &&
         endpoint_distance <= position_tolerance &&
         measured_speed <= speed_tolerance;
}

}  // namespace tracing
}  // namespace robot_control
