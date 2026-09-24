/*
 * Translational path-tracking ROS adapter.
 *
 * Inputs:
 *   /plan             nav_msgs/Path, map frame
 *   /OdometryHighFreq nav_msgs/Odometry, odom pose + base_link_hf twist
 *   /cmd_controller   ControllerCmd.trajectory is the tracking enable switch
 * Outputs:
 *   /cmd_track        geometry_msgs/Twist in the chassis frame
 *   /tracking_status  current lifecycle and safety state
 *   /tracking_debug   temporary nominal/reference/command/feedback telemetry
 *
 * `map` and `odom` are deliberately treated as numerically identical in this
 * first deployment.  Do not use this adapter with a drifting map->odom
 * transform; introduce tf2 first when that assumption stops being true.
 */

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"
#include "robot_interfaces/msg/controller_cmd.hpp"
#include "robot_interfaces/msg/tracking_debug.hpp"
#include "robot_interfaces/msg/tracking_status.hpp"

#ifdef ROBOT_CONTROL_HAS_PLAN_META
#include "navigation/msg/plan_meta.hpp"
#endif

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
  explicit TracingNode(const std::string & node_name)
  : Node(node_name)
  {
    declareParameters();

    std::string error;
    if (!motion_limits_.isValid(&error)) {
      throw std::invalid_argument("Invalid trajectory motion limits: " + error);
    }

    generator_ = std::make_unique<rt::TrajectoryGenerator>(motion_limits_, generator_options_);
    mpc_ = std::make_unique<robot_control::MpcController>(N_, dt_);
    mpc_->setErrorLimits(e_xy_max_, e_yaw_max_);
    mpc_->setWeights(q_diag_, r_diag_, normal_mpc_accel_);
    mpc_->setReferenceRelativeSpeedLimit(reference_speed_margin_, reference_speed_braking_decel_);

    rclcpp::QoS plan_qos(rclcpp::KeepLast(1));
    // nav_msgs/Path publishers normally use volatile durability.  Requesting
    // transient_local here would make this subscriber incompatible with them.
    plan_qos.reliable().durability_volatile();
    sub_plan_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic_, plan_qos,
      std::bind(&TracingNode::planCallback, this, std::placeholders::_1));
#ifdef ROBOT_CONTROL_HAS_PLAN_META
    sub_plan_meta_ = create_subscription<navigation::msg::PlanMeta>(
      plan_meta_topic_, plan_qos,
      std::bind(&TracingNode::planMetaCallback, this, std::placeholders::_1));
#else
    RCLCPP_WARN(
      get_logger(),
      "navigation/msg/PlanMeta is not available in this build; /plan will wait for a PlanMeta-capable rebuild");
#endif
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
      "tracing_node: N=%d dt=%.3f, /plan=%s /odom=%s /cmd_track=%s; "
      "curve_speed_limit=%s dynamic_safety_envelope=%s; "
      "MPC delta limits=(%.1f, %.1f) m/s^2, %.1f rad/s^2; "
      "map and odom are configured as numerically identical; odom yaw offset=%.3f rad (%.1f deg)",
      N_, dt_, plan_topic_.c_str(), odom_topic_.c_str(), cmd_track_topic_.c_str(),
      generator_options_.enforce_curve_speed_limit ? "on" : "off",
      generator_options_.enforce_dynamic_safety_envelope ? "on" : "off",
      normal_mpc_accel_.x(), normal_mpc_accel_.y(), normal_mpc_accel_.z(),
      odom_yaw_offset_, odom_yaw_offset_ * 180.0 / kPi);
  }

private:
  void declareParameters()
  {
    N_ = declare_parameter<int>("N", 20);
    dt_ = declare_parameter<double>("dt", 0.05);
    if (N_ < 1 || dt_ <= 0.0) {
      throw std::invalid_argument("N must be positive and dt must be positive");
    }

    motion_limits_.cruise_speed = declare_parameter<double>("cruise_speed", 1.8); // 巡航速度
    // Temporary high-performance test profile.  These remain ROS parameters
    // so the validated vehicle limits can replace them without recompiling.
    motion_limits_.max_accel = declare_parameter<double>("max_accel", 4.0);
    motion_limits_.normal_decel = declare_parameter<double>("normal_decel", 4.0);
    // MotionLimits requires emergency_decel >= normal_decel.  Keep the
    // minimum valid value until a separately calibrated emergency value is
    // supplied through the parameter.
    motion_limits_.emergency_decel = declare_parameter<double>("emergency_decel", 6.0);
    motion_limits_.max_lateral_accel = declare_parameter<double>("max_lateral_accel", 3.0);
    motion_limits_.terminal_speed = declare_parameter<double>("terminal_speed", 0.0);
    motion_limits_.accel_fraction = declare_parameter<double>("accel_fraction", 0.20);  // 加速段占比
    motion_limits_.decel_fraction = declare_parameter<double>("decel_fraction", 0.25);  // 减速段占比

    path_options_.sample_spacing = declare_parameter<double>("sample_spacing", 0.05);  // 采样距离
    path_options_.rdp_epsilon = declare_parameter<double>("rdp_epsilon", 0.0);  // rdp抽稀容差
    path_options_.smooth_half_window = declare_parameter<double>("smooth_half_window", 0.0);  // 平滑窗口半窗大小
    path_options_.max_smooth_deviation = declare_parameter<double>("max_smooth_deviation", 0.05);
    path_options_.max_smoothing_attempts =
      declare_parameter<int>("max_smoothing_attempts", 5);

    generator_options_.max_activation_offset =
      declare_parameter<double>("max_activation_offset", 0.50);
    generator_options_.local_projection_backtrack =
      declare_parameter<double>("local_projection_backtrack", 1.0);
    generator_options_.local_projection_lookahead =
      declare_parameter<double>("local_projection_lookahead", 3.0);
    generator_options_.profile_spacing = declare_parameter<double>("profile_spacing", 0.02);
    generator_options_.minimum_speed_for_time =
      declare_parameter<double>("minimum_speed_for_time", 1e-4);
    generator_options_.max_reference_lead =
      declare_parameter<double>("max_reference_lead", 0.10);
    // Experimental defaults: characterise the chassis against the fixed
    // v_des(s) first. Restore both to true after the lidar velocity and
    // vehicle limits are calibrated.
    generator_options_.enforce_curve_speed_limit =
      declare_parameter<bool>("enforce_curve_speed_limit", false);
    generator_options_.enforce_dynamic_safety_envelope =
      declare_parameter<bool>("enforce_dynamic_safety_envelope", false);

    q_diag_ << declare_parameter<double>("q_ex", 120.0),
      declare_parameter<double>("q_ey", 120.0),
      declare_parameter<double>("q_eyaw", 90.0),
      declare_parameter<double>("q_vx", 20.0),
      declare_parameter<double>("q_vy", 20.0),
      declare_parameter<double>("q_vw", 2.0);
    r_diag_ << declare_parameter<double>("r_du_x", 1.5),
      declare_parameter<double>("r_du_y", 1.5),
      declare_parameter<double>("r_du_w", 0.8);

    // Temporary tracking experiment: at dt=0.05 s these high limits allow
    // a 5 m/s change in each planar axis and a 5 rad/s change in yaw rate
    // in one step. The reference-relative planar speed cap remains active.
    // Restore measured chassis limits after characterisation.
    normal_mpc_accel_ << declare_parameter<double>("a_max_x", 100.0),
      declare_parameter<double>("a_max_y", 100.0),
      declare_parameter<double>("a_max_w", 100.0);
    emergency_mpc_accel_ <<
      declare_parameter<double>("emergency_a_max_x", normal_mpc_accel_.x()),
      declare_parameter<double>("emergency_a_max_y", normal_mpc_accel_.y()),
      declare_parameter<double>("emergency_a_max_w", normal_mpc_accel_(2));

    reference_speed_margin_ = declare_parameter<double>("reference_speed_margin", 0.05);
    reference_speed_braking_decel_ =
      declare_parameter<double>("reference_speed_braking_decel", motion_limits_.normal_decel);
    if (!std::isfinite(reference_speed_margin_) || reference_speed_margin_ < 0.0 ||
      !std::isfinite(reference_speed_braking_decel_) || reference_speed_braking_decel_ <= 0.0)
    {
      throw std::invalid_argument(
              "reference_speed_margin must be non-negative and reference_speed_braking_decel must be positive");
    }

    e_xy_max_ = declare_parameter<double>("e_xy_max", 1000.0);
    e_yaw_max_ = declare_parameter<double>("e_yaw_max", kPi);
    goal_position_tolerance_ = declare_parameter<double>("goal_position_tolerance", 0.05);
    goal_speed_tolerance_ = declare_parameter<double>("goal_speed_tolerance", 0.05);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.30);
    plan_timeout_ = declare_parameter<double>("plan_timeout", 0.0);
    // The lidar odometry now publishes quaternion yaw and child-frame twist in
    // the physical chassis convention, so no correction is applied by default.
    // Keep this parameter only as a deliberate compatibility escape hatch for
    // an old or differently aligned odometry publisher.
    odom_yaw_offset_ = declare_parameter<double>("odom_yaw_offset", 0.0);

    plan_topic_ = declare_parameter<std::string>("plan_topic", "plan");
    plan_meta_topic_ = declare_parameter<std::string>("plan_meta_topic", "/terrain_minco/plan_meta");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "OdometryHighFreq");
    controller_topic_ = declare_parameter<std::string>("controller_topic", "cmd_controller");
    cmd_track_topic_ = declare_parameter<std::string>("cmd_track_topic", "cmd_track");
    status_topic_ = declare_parameter<std::string>("status_topic", "tracking_status");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "tracking_debug");
    path_frame_ = declare_parameter<std::string>("path_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link_hf");
  }

  bool validPlanFrame(const std::string & frame_id) const
  {
    return frame_id == path_frame_;
  }

  bool validOdomFrames(const nav_msgs::msg::Odometry & msg) const
  {
    return msg.header.frame_id == odom_frame_ && msg.child_frame_id == base_frame_;
  }

  void planCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // /plan deliberately remains a standard nav_msgs/Path. Its paired
    // navigation/PlanMeta has an identical header and supplies path_id.
#ifdef ROBOT_CONTROL_HAS_PLAN_META
    pending_plan_ = msg;
    tryAcceptPendingPlan();
#else
    (void)msg;
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "received /plan but this tracing_node was built without navigation/msg/PlanMeta");
#endif
  }

#ifdef ROBOT_CONTROL_HAS_PLAN_META
  static bool sameHeader(const std_msgs::msg::Header & lhs, const std_msgs::msg::Header & rhs)
  {
    return lhs.stamp.sec == rhs.stamp.sec && lhs.stamp.nanosec == rhs.stamp.nanosec &&
           lhs.frame_id == rhs.frame_id;
  }

  static bool headerIsOlder(const std_msgs::msg::Header & lhs, const std_msgs::msg::Header & rhs)
  {
    return lhs.stamp.sec < rhs.stamp.sec ||
           (lhs.stamp.sec == rhs.stamp.sec && lhs.stamp.nanosec < rhs.stamp.nanosec);
  }

  void planMetaCallback(const navigation::msg::PlanMeta::SharedPtr msg)
  {
    pending_plan_meta_ = msg;
    tryAcceptPendingPlan();
  }

  void tryAcceptPendingPlan()
  {
    if (!pending_plan_ || !pending_plan_meta_) {
      return;
    }

    if (!sameHeader(pending_plan_->header, pending_plan_meta_->header)) {
      // The two topics have independent DDS delivery order. Drop only the
      // older unmatched item, so a following counterpart can still form a pair.
      if (headerIsOlder(pending_plan_->header, pending_plan_meta_->header)) {
        pending_plan_.reset();
      } else {
        pending_plan_meta_.reset();
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "discarded an unmatched /plan or /terrain_minco/plan_meta header while waiting for a pair");
      return;
    }

    const uint64_t path_id = pending_plan_meta_->path_id;
    const nav_msgs::msg::Path::SharedPtr path = std::move(pending_plan_);
    pending_plan_meta_.reset();

    if (have_seen_path_id_ && path_id <= latest_path_id_) {
      if (path_id == latest_path_id_) {
        // publish_seq and path_start_s intentionally do not affect trajectory
        // state. A same-ID pair is only a heartbeat for plan_timeout.
        last_valid_plan_t_ = now();
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ignoring duplicate path_id=%lu", static_cast<unsigned long>(path_id));
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ignoring stale path_id=%lu; latest seen path_id=%lu",
          static_cast<unsigned long>(path_id), static_cast<unsigned long>(latest_path_id_));
      }
      return;
    }

    have_seen_path_id_ = true;
    latest_path_id_ = path_id;
    acceptPlan(*path, path_id);
  }
#endif

  void acceptPlan(const nav_msgs::msg::Path & msg, uint64_t path_id)
  {
    if (!validPlanFrame(msg.header.frame_id)) {
      rejectPath(
        "path_id=" + std::to_string(path_id) + ": plan frame '" + msg.header.frame_id +
        "' does not match expected '" + path_frame_ + "'");
      return;
    }

    std::vector<rt::Point2> points;
    points.reserve(msg.poses.size());
    for (const auto & pose : msg.poses) {
      points.push_back({pose.pose.position.x, pose.pose.position.y});
    }

    rt::PathGeometry candidate;
    std::string error;
    if (!candidate.build(points, path_options_, &error)) {
      rejectPath("path_id=" + std::to_string(path_id) + ": cannot build path: " + error);
      return;
    }

    cached_path_ = std::move(candidate);
    have_cached_path_ = true;
    last_valid_plan_t_ = now();
    goal_reached_ = false;
    emergency_stop_ = false;
    path_activation_rejected_ = false;

    RCLCPP_INFO(
      get_logger(), "accepted path_id=%lu: %zu samples, %.3f m, max smooth deviation %.4f m",
      static_cast<unsigned long>(path_id), cached_path_.size(), cached_path_.length(),
      cached_path_.maxSmoothDeviation());

    if (tracking_enabled_) {
      if (!activateCachedPath(&error)) {
        rejectActivatedPath(error);
      }
    } else {
      publishStatus(
        robot_interfaces::msg::TrackingStatus::DISABLED,
        "path cached; tracking disabled");
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!validOdomFrames(*msg)) {
      have_odom_ = false;
      last_odom_error_ =
        "Odometry frames must be '" + odom_frame_ + "' and '" + base_frame_ +
        "', received '" + msg->header.frame_id + "' and '" + msg->child_frame_id + "'";
      if (generator_) {
        generator_->clearPath();
      }
      publishZero();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s", last_odom_error_.c_str());
      return;
    }

    last_odom_t_ = now();
    last_odom_error_.clear();
    mpc_state_.x = msg->pose.pose.position.x;
    mpc_state_.y = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    const double raw_odom_yaw =
      robot_control::tracing::yawFromQuaternion(q.x, q.y, q.z, q.w);
    mpc_state_.yaw = robot_control::tracing::normalizeYaw(raw_odom_yaw + odom_yaw_offset_);
    // nav_msgs/Odometry expresses twist in child_frame_id.  The current lidar
    // publisher's child frame is the physical chassis frame, so with the
    // default zero offset this is an identity transform.  A nonzero parameter
    // remains available only for an explicitly known legacy frame alignment.
    const rt::Vector2 chassis_velocity = robot_control::tracing::rotatePlanarVelocity(
      -odom_yaw_offset_, msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    // The odometry twist is reported in child_frame_id.  Convert it here, at
    // the ROS boundary, so every planar quantity passed to MpcController is in
    // the same world frame as the path and trajectory reference.
    const rt::Vector2 world_velocity = robot_control::tracing::bodyVelocityToWorld(
      mpc_state_.yaw, chassis_velocity.x, chassis_velocity.y);
    mpc_state_.vx = world_velocity.x;
    mpc_state_.vy = world_velocity.y;
    mpc_state_.vw = msg->twist.twist.angular.z;

    RCLCPP_INFO_ONCE(
      get_logger(),
      "odometry alignment: raw yaw=%.3f rad (%.1f deg), corrected yaw=%.3f rad (%.1f deg), "
      "twist rotation=%.3f rad (%.1f deg)",
      raw_odom_yaw, raw_odom_yaw * 180.0 / kPi,
      mpc_state_.yaw, mpc_state_.yaw * 180.0 / kPi,
      -odom_yaw_offset_, -odom_yaw_offset_ * 180.0 / kPi);

    motion_state_.position = {mpc_state_.x, mpc_state_.y};
    motion_state_.velocity = world_velocity;
    have_odom_ = true;

    if (tracking_enabled_ && have_cached_path_ && !generator_->hasActivePath() && !goal_reached_ &&
      !emergency_stop_ && !path_activation_rejected_)
    {
      std::string error;
      if (!activateCachedPath(&error)) {
        rejectActivatedPath(error);
      }
    }
  }

  void controllerCallback(const robot_interfaces::msg::ControllerCmd::SharedPtr msg)
  {
    const bool requested_enabled = msg->trajectory != 0U;
    if (requested_enabled == tracking_enabled_) {
      return;
    }

    tracking_enabled_ = requested_enabled;
    goal_reached_ = false;
    emergency_stop_ = false;
    path_activation_rejected_ = false;
    if (!tracking_enabled_) {
      generator_->clearPath();
      have_locked_yaw_ = false;
      diagnostics_ = {};
      publishZero();
      publishStatus(robot_interfaces::msg::TrackingStatus::DISABLED, "tracking switch disabled");
      return;
    }

    if (have_odom_) {
      locked_yaw_ = mpc_state_.yaw;
      have_locked_yaw_ = true;
    } else {
      have_locked_yaw_ = false;
    }

    std::string error;
    if (have_cached_path_ && have_odom_ && !activateCachedPath(&error)) {
      rejectActivatedPath(error);
    }
  }

  bool activateCachedPath(std::string * error)
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

    // A rejected replacement must not leave the old path active.  In a
    // replan, carrying on with the old path could drive into the new obstacle.
    generator_->clearPath();
    if (!generator_->activatePath(cached_path_, motion_state_, error)) {
      return false;
    }
    diagnostics_ = generator_->diagnostics();
    path_activation_rejected_ = false;
    return true;
  }

  void rejectPath(const std::string & detail)
  {
    have_cached_path_ = false;
    goal_reached_ = false;
    emergency_stop_ = false;
    generator_->clearPath();
    diagnostics_ = {};
    path_activation_rejected_ = true;
    publishZero();
    publishStatus(robot_interfaces::msg::TrackingStatus::PATH_REJECTED, detail);
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s", detail.c_str());
  }

  void rejectActivatedPath(const std::string & detail)
  {
    generator_->clearPath();
    diagnostics_ = {};
    publishZero();
    publishStatus(robot_interfaces::msg::TrackingStatus::PATH_REJECTED, detail);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "path rejected: %s", detail.c_str());
  }

  void controlTick()
  {
    const rclcpp::Time current_time = now();

    if (!tracking_enabled_) {
      publishZero();
      publishStatus(robot_interfaces::msg::TrackingStatus::DISABLED, "tracking switch disabled");
      return;
    }

    if (!have_odom_) {
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::WAITING_FOR_ODOMETRY,
        last_odom_error_.empty() ? "waiting for valid odometry" : last_odom_error_);
      return;
    }

    if ((current_time - last_odom_t_).seconds() > odom_timeout_) {
      generator_->clearPath();
      publishZero();
      publishStatus(robot_interfaces::msg::TrackingStatus::ODOMETRY_TIMEOUT, "odometry timeout");
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "odometry timeout -> zero command");
      return;
    }

    if (!have_cached_path_) {
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::WAITING_FOR_PATH,
        "waiting for valid path");
      return;
    }

    if (plan_timeout_ > 0.0 && (current_time - last_valid_plan_t_).seconds() > plan_timeout_) {
      generator_->clearPath();
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::WAITING_FOR_PATH,
        "plan heartbeat timeout");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "plan heartbeat timeout -> zero command");
      return;
    }

    if (emergency_stop_) {
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::EMERGENCY_STOP,
        "emergency braking cannot stop within the active path");
      return;
    }

    if (goal_reached_) {
      publishZero();
      publishStatus(robot_interfaces::msg::TrackingStatus::GOAL_REACHED, "goal reached");
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
      motion_state_, dt_, static_cast<std::size_t>(N_), &reference, heading_provider);
    diagnostics_ = generator_->diagnostics();

    if (trajectory_status == rt::TrajectoryStatus::EmergencyInfeasible) {
      emergency_stop_ = true;
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::EMERGENCY_STOP,
        "emergency deceleration cannot stop before path end");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 500,
        "path emergency infeasible: remaining=%.3f m residual=%.3f m/s -> zero command",
        diagnostics_.remaining_length,
        diagnostics_.terminal_speed_if_unstoppable);
      return;
    }

    if (trajectory_status == rt::TrajectoryStatus::NoActivePath ||
      reference.size() < static_cast<std::size_t>(N_))
    {
      publishZero();
      publishStatus(
        robot_interfaces::msg::TrackingStatus::PATH_REJECTED,
        "trajectory horizon unavailable");
      return;
    }

    const double measured_speed = std::hypot(motion_state_.velocity.x, motion_state_.velocity.y);
    if (diagnostics_.remaining_length <= goal_position_tolerance_ &&
      measured_speed <= goal_speed_tolerance_)
    {
      goal_reached_ = true;
      generator_->clearPath();
      publishZero();
      publishStatus(robot_interfaces::msg::TrackingStatus::GOAL_REACHED, "goal reached");
      return;
    }

    std::vector<robot_control::TrajectoryPoint> mpc_reference;
    mpc_reference.reserve(reference.size());
    for (const rt::ReferencePoint & point : reference) {
      const double reference_yaw = point.heading.valid ? point.heading.yaw : locked_yaw_;
      robot_control::TrajectoryPoint mpc_point;
      mpc_point.x = point.position.x;
      mpc_point.y = point.position.y;
      mpc_point.yaw = reference_yaw;
      mpc_point.vx = point.velocity.x;
      mpc_point.vy = point.velocity.y;
      mpc_point.vw = point.heading.valid ? point.heading.angular_velocity : 0.0;
      mpc_reference.push_back(mpc_point);
    }

    mpc_->setAccelerationLimits(
      trajectory_status == rt::TrajectoryStatus::EmergencyBraking ?
      emergency_mpc_accel_ : normal_mpc_accel_);
    mpc_->setReferenceRelativeSpeedLimit(
      reference_speed_margin_,
      trajectory_status == rt::TrajectoryStatus::EmergencyBraking ?
      motion_limits_.emergency_decel : reference_speed_braking_decel_);

    robot_control::ControlCmd command;
    if (!mpc_->solveMPC(mpc_state_, mpc_reference, command)) {
      publishZero();
      publishStatus(robot_interfaces::msg::TrackingStatus::MPC_FAILURE, "MPC solve failed");
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "MPC solve failed -> zero command");
      return;
    }

    // MpcController returns a world-frame planar command.  /cmd_track is a
    // chassis-frame interface, so perform the only output conversion here
    // using the current measured yaw (not the locked/reference yaw).
    const rt::Vector2 command_body = robot_control::tracing::worldVelocityToBody(
      mpc_state_.yaw, rt::Vector2{command.vx, command.vy});
    geometry_msgs::msg::Twist output;
    output.linear.x = command_body.x;
    output.linear.y = command_body.y;
    output.angular.z = command.vw;
    pub_cmd_track_->publish(output);
    publishDebug(reference.front(), trajectory_status, command);

    if (trajectory_status == rt::TrajectoryStatus::EmergencyBraking) {
      publishStatus(
        robot_interfaces::msg::TrackingStatus::EMERGENCY_BRAKING,
        "normal braking insufficient; emergency profile active");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "path emergency braking: remaining=%.3f m normal deficit=%.3f m",
        diagnostics_.remaining_length, diagnostics_.stop_deficit);
    } else {
      publishStatus(robot_interfaces::msg::TrackingStatus::TRACKING, "tracking");
    }
  }

  void publishZero()
  {
    pub_cmd_track_->publish(geometry_msgs::msg::Twist{});
  }

  void publishStatus(uint8_t state, const std::string & detail)
  {
    robot_interfaces::msg::TrackingStatus status;
    status.header.stamp = now();
    status.header.frame_id = path_frame_;
    status.has_path = have_cached_path_;
    status.path_id = have_seen_path_id_ ? latest_path_id_ : 0U;
    status.state = state;
    status.progress = diagnostics_.progress;
    status.remaining_distance = diagnostics_.remaining_length;
    status.stop_deficit = diagnostics_.stop_deficit;
    status.terminal_speed = diagnostics_.terminal_speed_if_unstoppable;
    status.detail = detail;
    pub_status_->publish(status);
  }

  void publishDebug(
    const rt::ReferencePoint & point,
    const rt::TrajectoryStatus trajectory_status,
    const robot_control::ControlCmd & command)
  {
    robot_interfaces::msg::TrackingDebug debug;
    debug.header.stamp = now();
    // Keep all planar debug vectors in the same world frame used by MPC so a
    // bag can compare reference, feedback and optimizer output directly.
    debug.header.frame_id = path_frame_;
    debug.trajectory_status = static_cast<uint8_t>(trajectory_status);
    debug.progress = diagnostics_.progress;
    debug.remaining_distance = diagnostics_.remaining_length;
    debug.nominal_phase_arc_length = point.nominal_phase_arc_length;
    debug.nominal_phase_speed = point.nominal_phase_speed;
    debug.nominal_speed_at_progress = point.nominal_speed_at_progress;
    debug.reference_arc_length = point.arc_length;
    debug.reference_speed = point.speed;
    debug.reachability_limited = point.reachability_limited;
    debug.spatial_safety_limited = point.spatial_safety_limited;
    debug.nominal_reference.linear.x = point.nominal_phase_velocity.x;
    debug.nominal_reference.linear.y = point.nominal_phase_velocity.y;
    debug.nominal_reference.angular.z = point.heading.valid ? point.heading.angular_velocity : 0.0;
    debug.final_reference.linear.x = point.velocity.x;
    debug.final_reference.linear.y = point.velocity.y;
    debug.final_reference.angular.z = point.heading.valid ? point.heading.angular_velocity : 0.0;
    debug.command.linear.x = command.vx;
    debug.command.linear.y = command.vy;
    debug.command.angular.z = command.vw;
    debug.measured.linear.x = mpc_state_.vx;
    debug.measured.linear.y = mpc_state_.vy;
    debug.measured.angular.z = mpc_state_.vw;
    pub_debug_->publish(debug);
  }

  int N_{20};
  double dt_{0.05};
  rt::MotionLimits motion_limits_;
  rt::PathBuildOptions path_options_;
  rt::GeneratorOptions generator_options_;

  Eigen::Matrix<double, 6, 1> q_diag_{};
  Eigen::Vector3d r_diag_{};
  Eigen::Vector3d normal_mpc_accel_{};
  Eigen::Vector3d emergency_mpc_accel_{};
  double reference_speed_margin_{0.05};
  double reference_speed_braking_decel_{4.0};
  double e_xy_max_{1000.0};
  double e_yaw_max_{kPi};
  double goal_position_tolerance_{0.05};
  double goal_speed_tolerance_{0.05};
  double odom_timeout_{0.30};
  double plan_timeout_{0.0};
  double odom_yaw_offset_{0.0};

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

  bool tracking_enabled_{false};
  bool have_odom_{false};
  bool have_cached_path_{false};
  bool have_seen_path_id_{false};
  bool have_locked_yaw_{false};
  bool goal_reached_{false};
  bool emergency_stop_{false};
  bool path_activation_rejected_{false};
  double locked_yaw_{0.0};
  uint64_t latest_path_id_{0U};
  std::string last_odom_error_;

  rclcpp::Time last_odom_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_valid_plan_t_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_track_;
  rclcpp::Publisher<robot_interfaces::msg::TrackingDebug>::SharedPtr pub_debug_;
  rclcpp::Publisher<robot_interfaces::msg::TrackingStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_plan_;
#ifdef ROBOT_CONTROL_HAS_PLAN_META
  rclcpp::Subscription<navigation::msg::PlanMeta>::SharedPtr sub_plan_meta_;
  nav_msgs::msg::Path::SharedPtr pending_plan_;
  navigation::msg::PlanMeta::SharedPtr pending_plan_meta_;
#endif
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<robot_interfaces::msg::ControllerCmd>::SharedPtr sub_controller_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TracingNode>("tracing_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
