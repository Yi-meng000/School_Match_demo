/*
 * ROS boundary for translational trajectory tracking.
 *
 * The path, odometry pose, MPC state/reference and debug telemetry use the
 * shared map/odom world convention.  Odometry twist arrives in base_frame and
 * /cmd_track leaves in the current chassis frame, so those two rotations live
 * here instead of inside MpcController.
 */

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigation/msg/plan_meta.hpp"
#include "robot_interfaces/msg/controller_cmd.hpp"
#include "robot_interfaces/msg/tracking_debug.hpp"
#include "robot_interfaces/msg/tracking_status.hpp"
#include "std_msgs/msg/header.hpp"

#include "robot_control/mpc_controller.hpp"
#include "robot_control/tracing_adapter.hpp"
#include "robot_control/translational_trajectory.hpp"

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rt = robot_control::trajectory;

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

class TracingNode : public rclcpp::Node
{
public:
  explicit TracingNode(const std::string& node_name)
  : Node(node_name)
  {
    declareParameters();

    std::string error;
    if (!speed_options_.isValid(&error)) {
      throw std::invalid_argument("Invalid speed profile options: " + error);
    }
    if (!generator_options_.isValid(&error)) {
      throw std::invalid_argument("Invalid trajectory generator options: " + error);
    }

    generator_ = std::make_unique<rt::TrajectoryGenerator>(
      speed_options_, generator_options_);
    mpc_ = std::make_unique<robot_control::MpcController>(
      horizon_, dt_, mpc_response_time_constants_);
    if (!mpc_->setWeights(q_diag_, correction_diag_, r_diag_, &error)) {
      throw std::invalid_argument("Invalid MPC weights: " + error);
    }

    rclcpp::QoS plan_qos(rclcpp::KeepLast(1));
    plan_qos.reliable().durability_volatile();
    sub_plan_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic_, plan_qos,
      std::bind(&TracingNode::planCallback, this, std::placeholders::_1));
    sub_plan_meta_ = create_subscription<navigation::msg::PlanMeta>(
      plan_meta_topic_, plan_qos,
      std::bind(&TracingNode::planMetaCallback, this, std::placeholders::_1));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(1).best_effort(),
      std::bind(&TracingNode::odomCallback, this, std::placeholders::_1));
    sub_controller_ = create_subscription<robot_interfaces::msg::ControllerCmd>(
      controller_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&TracingNode::controllerCallback, this, std::placeholders::_1));

    pub_cmd_track_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_track_topic_, rclcpp::QoS(1).reliable());
    rclcpp::QoS status_qos(rclcpp::KeepLast(1));
    status_qos.reliable().transient_local();
    pub_status_ = create_publisher<robot_interfaces::msg::TrackingStatus>(
      status_topic_, status_qos);
    pub_debug_ = create_publisher<robot_interfaces::msg::TrackingDebug>(
      debug_topic_, rclcpp::QoS(10).reliable());

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&TracingNode::controlTick, this));

    RCLCPP_INFO(
      get_logger(),
      "tracing_node: N=%d dt=%.3f cruise=%.2f m/s tau=(%.3f, %.3f, %.3f) s; "
      "/plan=%s /plan_meta=%s /odom=%s /cmd_track=%s; full-reference "
      "unconstrained Eigen LDLT MPC; map and odom are numerically identical",
      horizon_, dt_, speed_options_.cruise_speed,
      mpc_response_time_constants_.x(), mpc_response_time_constants_.y(),
      mpc_response_time_constants_.z(), plan_topic_.c_str(),
      plan_meta_topic_.c_str(), odom_topic_.c_str(), cmd_track_topic_.c_str());
  }

private:
  void declareParameters()
  {
    horizon_ = declare_parameter<int>("N", 20);
    dt_ = declare_parameter<double>("dt", 0.05);
    if (horizon_ < 1 || !std::isfinite(dt_) || dt_ <= 0.0) {
      throw std::invalid_argument("N and dt must be positive");
    }

    speed_options_.cruise_speed =
      declare_parameter<double>("cruise_speed", 4.0);
    speed_options_.nominal_accel =
      declare_parameter<double>("nominal_accel", 8.0);
    speed_options_.nominal_decel =
      declare_parameter<double>("nominal_decel", 8.0);
    speed_options_.accel_fraction =
      declare_parameter<double>("accel_fraction", 0.40);
    speed_options_.decel_fraction =
      declare_parameter<double>("decel_fraction", 0.40);

    generator_options_.max_activation_offset =
      declare_parameter<double>("max_activation_offset", 0.50);
    generator_options_.local_projection_backtrack =
      declare_parameter<double>("local_projection_backtrack", 1.0);
    generator_options_.local_projection_lookahead =
      declare_parameter<double>("local_projection_lookahead", 3.0);

    q_diag_ <<
      declare_parameter<double>("q_ex", 120.0),
      declare_parameter<double>("q_ey", 80.0),
      declare_parameter<double>("q_eyaw", 90.0),
      declare_parameter<double>("q_vx", 20.0),
      declare_parameter<double>("q_vy", 20.0),
      declare_parameter<double>("q_vw", 6.0);
    correction_diag_ <<
      declare_parameter<double>("s_correction_x", 2.0),
      declare_parameter<double>("s_correction_y", 6.0),
      declare_parameter<double>("s_correction_w", 2.0);
    r_diag_ <<
      declare_parameter<double>("r_du_x", 8.0),
      declare_parameter<double>("r_du_y", 20.0),
      declare_parameter<double>("r_du_w", 2.0);
    mpc_response_time_constants_ <<
      declare_parameter<double>("mpc_tau_x", 0.18),
      declare_parameter<double>("mpc_tau_y", 0.18),
      declare_parameter<double>("mpc_tau_w", 0.10);
    if (!mpc_response_time_constants_.allFinite() ||
        (mpc_response_time_constants_.array() <= 0.0).any())
    {
      throw std::invalid_argument(
        "mpc_tau_x, mpc_tau_y and mpc_tau_w must be finite and positive");
    }

    goal_position_tolerance_ =
      declare_parameter<double>("goal_position_tolerance", 0.05);
    goal_speed_tolerance_ =
      declare_parameter<double>("goal_speed_tolerance", 0.05);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.30);
    if (!std::isfinite(goal_position_tolerance_) ||
        !std::isfinite(goal_speed_tolerance_) ||
        !std::isfinite(odom_timeout_) || goal_position_tolerance_ < 0.0 ||
        goal_speed_tolerance_ < 0.0 || odom_timeout_ <= 0.0)
    {
      throw std::invalid_argument(
        "goal tolerances must be non-negative and odom_timeout must be positive");
    }

    plan_topic_ = declare_parameter<std::string>("plan_topic", "plan");
    plan_meta_topic_ = declare_parameter<std::string>(
      "plan_meta_topic", "/terrain_minco/plan_meta");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "OdometryHighFreq");
    controller_topic_ = declare_parameter<std::string>(
      "controller_topic", "cmd_controller");
    cmd_track_topic_ = declare_parameter<std::string>(
      "cmd_track_topic", "cmd_track");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "tracking_status");
    debug_topic_ = declare_parameter<std::string>(
      "debug_topic", "tracking_debug");
    path_frame_ = declare_parameter<std::string>("path_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link_hf");
  }

  bool validPlanFrame(const std::string& frame_id) const
  {
    return frame_id == path_frame_;
  }

  bool validOdomFrames(const nav_msgs::msg::Odometry& message) const
  {
    return message.header.frame_id == odom_frame_ &&
           message.child_frame_id == base_frame_;
  }

  static bool finiteOdometry(const nav_msgs::msg::Odometry& message)
  {
    const auto& position = message.pose.pose.position;
    const auto& orientation = message.pose.pose.orientation;
    const auto& linear = message.twist.twist.linear;
    return std::isfinite(position.x) && std::isfinite(position.y) &&
           std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
           std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
           std::isfinite(linear.x) && std::isfinite(linear.y) &&
           std::isfinite(message.twist.twist.angular.z);
  }

  void planCallback(const nav_msgs::msg::Path::SharedPtr message)
  {
    pending_plan_ = message;
    tryAcceptPendingPlan();
  }

  static bool sameHeader(
    const std_msgs::msg::Header& lhs, const std_msgs::msg::Header& rhs)
  {
    return lhs.stamp.sec == rhs.stamp.sec &&
           lhs.stamp.nanosec == rhs.stamp.nanosec &&
           lhs.frame_id == rhs.frame_id;
  }

  static bool headerIsOlder(
    const std_msgs::msg::Header& lhs, const std_msgs::msg::Header& rhs)
  {
    return lhs.stamp.sec < rhs.stamp.sec ||
      (lhs.stamp.sec == rhs.stamp.sec && lhs.stamp.nanosec < rhs.stamp.nanosec);
  }

  void planMetaCallback(const navigation::msg::PlanMeta::SharedPtr message)
  {
    pending_plan_meta_ = message;
    tryAcceptPendingPlan();
  }

  void tryAcceptPendingPlan()
  {
    if (!pending_plan_ || !pending_plan_meta_) {
      return;
    }
    if (!sameHeader(pending_plan_->header, pending_plan_meta_->header)) {
      if (headerIsOlder(pending_plan_->header, pending_plan_meta_->header)) {
        pending_plan_.reset();
      } else {
        pending_plan_meta_.reset();
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "discarded unmatched /plan and PlanMeta headers while waiting for a pair");
      return;
    }

    const uint64_t path_id = pending_plan_meta_->path_id;
    const nav_msgs::msg::Path::SharedPtr path = std::move(pending_plan_);
    pending_plan_meta_.reset();

    if (have_seen_path_id_ && path_id <= latest_path_id_) {
      if (path_id == latest_path_id_) {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ignoring duplicate path_id=%lu",
          static_cast<unsigned long>(path_id));
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ignoring stale path_id=%lu; latest=%lu",
          static_cast<unsigned long>(path_id),
          static_cast<unsigned long>(latest_path_id_));
      }
      return;
    }

    have_seen_path_id_ = true;
    latest_path_id_ = path_id;
    acceptPlan(*path, path_id);
  }

  void acceptPlan(const nav_msgs::msg::Path& message, uint64_t path_id)
  {
    if (!validPlanFrame(message.header.frame_id)) {
      rejectPath(
        "path_id=" + std::to_string(path_id) + ": plan frame '" +
        message.header.frame_id + "' does not match expected '" + path_frame_ + "'");
      return;
    }

    std::vector<rt::Point2> points;
    points.reserve(message.poses.size());
    for (const auto& pose : message.poses) {
      points.push_back({pose.pose.position.x, pose.pose.position.y});
    }

    rt::PathGeometry candidate;
    std::string error;
    if (!candidate.build(points, &error)) {
      rejectPath(
        "path_id=" + std::to_string(path_id) + ": cannot build path: " + error);
      return;
    }

    cached_path_ = std::move(candidate);
    have_cached_path_ = true;
    goal_reached_ = false;
    path_activation_rejected_ = false;

    RCLCPP_INFO(
      get_logger(), "accepted path_id=%lu: %zu planner points, %.3f m",
      static_cast<unsigned long>(path_id), cached_path_.size(), cached_path_.length());

    if (tracking_enabled_ && have_odom_) {
      if (!activateCachedPath(&error)) {
        rejectActivatedPath(error);
      }
    } else if (tracking_enabled_) {
      publishStatus(
        robot_interfaces::msg::TrackingStatus::WAITING_FOR_ODOMETRY,
        "path cached; waiting for valid odometry");
    } else {
      publishStatus(
        robot_interfaces::msg::TrackingStatus::DISABLED,
        "path cached; tracking disabled");
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!validOdomFrames(*message) || !finiteOdometry(*message)) {
      have_odom_ = false;
      last_odom_error_ =
        "invalid odometry values or frames; expected '" + odom_frame_ +
        "' and '" + base_frame_ + "', received '" +
        message->header.frame_id + "' and '" + message->child_frame_id + "'";
      generator_->clearPath();
      publishZero();
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "%s", last_odom_error_.c_str());
      return;
    }

    last_odom_t_ = now();
    last_odom_error_.clear();
    mpc_state_.x = message->pose.pose.position.x;
    mpc_state_.y = message->pose.pose.position.y;
    const auto& quaternion = message->pose.pose.orientation;
    mpc_state_.yaw = robot_control::tracing::yawFromQuaternion(
      quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    const rt::Vector2 world_velocity =
      robot_control::tracing::bodyVelocityToWorld(
      mpc_state_.yaw, message->twist.twist.linear.x,
      message->twist.twist.linear.y);
    mpc_state_.vx = world_velocity.x;
    mpc_state_.vy = world_velocity.y;
    mpc_state_.vw = message->twist.twist.angular.z;
    motion_state_.position = {mpc_state_.x, mpc_state_.y};
    motion_state_.velocity = world_velocity;
    have_odom_ = true;

    RCLCPP_INFO_ONCE(
      get_logger(),
      "odometry uses raw corrected lidar convention: yaw=%.3f rad (%.1f deg); "
      "child-frame twist is rotated directly into the world frame",
      mpc_state_.yaw, mpc_state_.yaw * 180.0 / kPi);

    if (tracking_enabled_ && have_cached_path_ &&
        !generator_->hasActivePath() && !goal_reached_ &&
        !path_activation_rejected_)
    {
      std::string error;
      if (!activateCachedPath(&error)) {
        rejectActivatedPath(error);
      }
    }
  }

  void controllerCallback(
    const robot_interfaces::msg::ControllerCmd::SharedPtr message)
  {
    const bool requested_enabled = message->trajectory != 0U;
    if (requested_enabled == tracking_enabled_) {
      return;
    }

    tracking_enabled_ = requested_enabled;
    goal_reached_ = false;
    path_activation_rejected_ = false;
    if (!tracking_enabled_) {
      generator_->clearPath();
      have_locked_yaw_ = false;
      diagnostics_ = {};
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::DISABLED,
        "tracking switch disabled");
      return;
    }

    if (have_odom_) {
      locked_yaw_ = mpc_state_.yaw;
      have_locked_yaw_ = true;
      previous_world_command_.vx = mpc_state_.vx;
      previous_world_command_.vy = mpc_state_.vy;
      previous_world_command_.vw = mpc_state_.vw;
    } else {
      have_locked_yaw_ = false;
      previous_world_command_ = {};
    }

    std::string error;
    if (have_cached_path_ && have_odom_ && !activateCachedPath(&error)) {
      rejectActivatedPath(error);
    }
  }

  bool activateCachedPath(std::string* error)
  {
    if (!tracking_enabled_ || !have_cached_path_ || !have_odom_) {
      if (error != nullptr) {
        *error = "tracking, path, or odometry is not ready";
      }
      return false;
    }
    if (!have_locked_yaw_) {
      locked_yaw_ = mpc_state_.yaw;
      have_locked_yaw_ = true;
    }

    generator_->clearPath();
    if (!generator_->activatePath(cached_path_, motion_state_, error)) {
      return false;
    }
    diagnostics_ = generator_->diagnostics();
    path_activation_rejected_ = false;
    if (diagnostics_.nominal_decel_exceeded) {
      RCLCPP_WARN(
        get_logger(),
        "path_id=%lu needs %.3f m/s^2 peak sine deceleration, above nominal_decel=%.3f; "
        "the fixed zero-terminal-speed profile is kept and tracking is not stopped",
        static_cast<unsigned long>(latest_path_id_),
        diagnostics_.required_peak_deceleration,
        speed_options_.nominal_decel);
    }
    return true;
  }

  void rejectPath(const std::string& detail)
  {
    have_cached_path_ = false;
    goal_reached_ = false;
    generator_->clearPath();
    diagnostics_ = {};
    path_activation_rejected_ = true;
    publishZero();
    publishStatus(robot_interfaces::msg::TrackingStatus::PATH_REJECTED, detail);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "%s", detail.c_str());
  }

  void rejectActivatedPath(const std::string& detail)
  {
    generator_->clearPath();
    diagnostics_ = {};
    path_activation_rejected_ = true;
    publishZero();
    publishStatus(robot_interfaces::msg::TrackingStatus::PATH_REJECTED, detail);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "path rejected: %s", detail.c_str());
  }

  double endpointDistance() const
  {
    if (!have_cached_path_ || !cached_path_.valid() || !have_odom_) {
      return 0.0;
    }
    return robot_control::tracing::planarDistance(
      motion_state_.position,
      cached_path_.sample(cached_path_.length()).position);
  }

  void controlTick()
  {
    const rclcpp::Time current_time = now();

    if (!tracking_enabled_) {
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::DISABLED,
        "tracking switch disabled");
      return;
    }
    if (!have_odom_) {
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::WAITING_FOR_ODOMETRY,
        last_odom_error_.empty() ?
        "waiting for valid odometry" : last_odom_error_);
      return;
    }
    if ((current_time - last_odom_t_).seconds() > odom_timeout_) {
      generator_->clearPath();
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::ODOMETRY_TIMEOUT,
        "odometry timeout");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "odometry timeout -> zero command");
      return;
    }
    if (!have_cached_path_) {
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::WAITING_FOR_PATH,
        "waiting for valid path");
      return;
    }
    if (goal_reached_) {
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::GOAL_REACHED,
        "goal reached");
      return;
    }
    if (path_activation_rejected_) {
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::PATH_REJECTED,
        "latest path could not be activated; waiting for a new path or switch cycle");
      return;
    }
    if (!generator_->hasActivePath()) {
      std::string error;
      if (!activateCachedPath(&error)) {
        rejectActivatedPath(error);
        return;
      }
    }

    const rt::HeadingProvider heading_provider = [this](double, double) {
        return rt::HeadingReference{true, locked_yaw_, 0.0, 0.0};
      };
    std::vector<rt::ReferencePoint> reference;
    const rt::TrajectoryStatus trajectory_status = generator_->makeHorizon(
      motion_state_, dt_, static_cast<std::size_t>(horizon_),
      &reference, heading_provider);
    diagnostics_ = generator_->diagnostics();
    if (trajectory_status != rt::TrajectoryStatus::Ready ||
        reference.size() < static_cast<std::size_t>(horizon_))
    {
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::PATH_REJECTED,
        "trajectory horizon unavailable");
      return;
    }

    const double measured_speed = std::hypot(
      motion_state_.velocity.x, motion_state_.velocity.y);
    const double endpoint_distance = endpointDistance();
    if (robot_control::tracing::goalReached(
        diagnostics_.remaining_length, endpoint_distance, measured_speed,
        goal_position_tolerance_, goal_speed_tolerance_))
    {
      goal_reached_ = true;
      generator_->clearPath();
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::GOAL_REACHED,
        "goal reached");
      return;
    }

    std::vector<robot_control::TrajectoryPoint> mpc_reference;
    mpc_reference.reserve(reference.size());
    for (const rt::ReferencePoint& point : reference) {
      robot_control::TrajectoryPoint mpc_point;
      mpc_point.x = point.position.x;
      mpc_point.y = point.position.y;
      mpc_point.yaw = point.heading.valid ? point.heading.yaw : locked_yaw_;
      mpc_point.vx = point.velocity.x;
      mpc_point.vy = point.velocity.y;
      mpc_point.vw = point.heading.valid ? point.heading.angular_velocity : 0.0;
      mpc_point.ax = point.acceleration.x;
      mpc_point.ay = point.acceleration.y;
      mpc_point.aw =
        point.heading.valid ? point.heading.angular_acceleration : 0.0;
      mpc_reference.push_back(mpc_point);
    }

    robot_control::ControlCmd command;
    if (!mpc_->solveMPC(
        mpc_state_, mpc_reference, previous_world_command_, command))
    {
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::MPC_FAILURE,
        "MPC solve failed");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MPC solve failed -> zero command");
      return;
    }

    const rt::Vector2 command_body =
      robot_control::tracing::worldVelocityToBody(
      mpc_state_.yaw, {command.vx, command.vy});
    geometry_msgs::msg::Twist output;
    output.linear.x = command_body.x;
    output.linear.y = command_body.y;
    output.angular.z = command.vw;
    pub_cmd_track_->publish(output);
    previous_world_command_ = command;
    publishDebug(reference.front(), command, endpoint_distance);
    publishStatus(
      robot_interfaces::msg::TrackingStatus::TRACKING,
      "tracking");
  }

  void publishZero()
  {
    pub_cmd_track_->publish(geometry_msgs::msg::Twist{});
    previous_world_command_ = {};
  }

  void publishStatus(uint8_t state, const std::string& detail)
  {
    robot_interfaces::msg::TrackingStatus status;
    status.header.stamp = now();
    status.header.frame_id = path_frame_;
    status.has_path = have_cached_path_;
    status.path_id = have_seen_path_id_ ? latest_path_id_ : 0U;
    status.state = state;
    status.progress = diagnostics_.progress;
    status.remaining_distance = diagnostics_.remaining_length;
    status.endpoint_distance = endpointDistance();
    status.detail = detail;
    pub_status_->publish(status);
  }

  void publishDebug(
    const rt::ReferencePoint& point,
    const robot_control::ControlCmd& command,
    double endpoint_distance)
  {
    robot_interfaces::msg::TrackingDebug debug;
    debug.header.stamp = now();
    debug.header.frame_id = path_frame_;
    debug.path_id = have_seen_path_id_ ? latest_path_id_ : 0U;
    debug.progress = diagnostics_.progress;
    debug.remaining_distance = diagnostics_.remaining_length;
    debug.endpoint_distance = endpoint_distance;
    debug.speed_at_progress = diagnostics_.speed_at_progress;
    debug.reference_arc_length = point.arc_length;
    debug.reference_speed = point.speed;
    debug.reference.linear.x = point.velocity.x;
    debug.reference.linear.y = point.velocity.y;
    debug.reference.angular.z =
      point.heading.valid ? point.heading.angular_velocity : 0.0;
    debug.command.linear.x = command.vx;
    debug.command.linear.y = command.vy;
    debug.command.angular.z = command.vw;
    debug.measured.linear.x = mpc_state_.vx;
    debug.measured.linear.y = mpc_state_.vy;
    debug.measured.angular.z = mpc_state_.vw;
    pub_debug_->publish(debug);
  }

  int horizon_{20};
  double dt_{0.05};
  rt::SpeedProfileOptions speed_options_;
  rt::GeneratorOptions generator_options_;
  Eigen::Matrix<double, 6, 1> q_diag_{};
  Eigen::Vector3d correction_diag_{};
  Eigen::Vector3d r_diag_{};
  Eigen::Vector3d mpc_response_time_constants_{0.18, 0.18, 0.10};
  double goal_position_tolerance_{0.05};
  double goal_speed_tolerance_{0.05};
  double odom_timeout_{0.30};

  std::string plan_topic_;
  std::string plan_meta_topic_;
  std::string odom_topic_;
  std::string controller_topic_;
  std::string cmd_track_topic_;
  std::string status_topic_;
  std::string debug_topic_;
  std::string path_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  std::unique_ptr<robot_control::MpcController> mpc_;
  std::unique_ptr<rt::TrajectoryGenerator> generator_;
  rt::PathGeometry cached_path_;
  rt::MotionState2D motion_state_;
  rt::TrajectoryDiagnostics diagnostics_;
  robot_control::State mpc_state_;
  robot_control::ControlCmd previous_world_command_;

  bool tracking_enabled_{false};
  bool have_odom_{false};
  bool have_cached_path_{false};
  bool have_seen_path_id_{false};
  bool have_locked_yaw_{false};
  bool goal_reached_{false};
  bool path_activation_rejected_{false};
  double locked_yaw_{0.0};
  uint64_t latest_path_id_{0U};
  std::string last_odom_error_;
  rclcpp::Time last_odom_t_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_track_;
  rclcpp::Publisher<robot_interfaces::msg::TrackingDebug>::SharedPtr pub_debug_;
  rclcpp::Publisher<robot_interfaces::msg::TrackingStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_plan_;
  rclcpp::Subscription<navigation::msg::PlanMeta>::SharedPtr sub_plan_meta_;
  nav_msgs::msg::Path::SharedPtr pending_plan_;
  navigation::msg::PlanMeta::SharedPtr pending_plan_meta_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<robot_interfaces::msg::ControllerCmd>::SharedPtr sub_controller_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TracingNode>("tracing_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
