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
 *
 * `map` and `odom` are deliberately treated as numerically identical in this
 * first deployment.  Do not use this adapter with a drifting map->odom
 * transform; introduce tf2 first when that assumption stops being true.
 */

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "robot_interfaces/msg/controller_cmd.hpp"
#include "robot_interfaces/msg/tracking_status.hpp"

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

    rclcpp::QoS plan_qos(rclcpp::KeepLast(1));
    // nav_msgs/Path publishers normally use volatile durability.  Requesting
    // transient_local here would make this subscriber incompatible with them.
    plan_qos.reliable().durability_volatile();
    sub_plan_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic_, plan_qos,
      std::bind(&TracingNode::planCallback, this, std::placeholders::_1));
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

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&TracingNode::controlTick, this));

    RCLCPP_INFO(
      get_logger(),
      "tracing_node: N=%d dt=%.3f, /plan=%s /odom=%s /cmd_track=%s; "
      "map and odom are configured as numerically identical; odom yaw offset=%.3f rad (%.1f deg)",
      N_, dt_, plan_topic_.c_str(), odom_topic_.c_str(), cmd_track_topic_.c_str(),
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

    motion_limits_.cruise_speed = declare_parameter<double>("cruise_speed", 0.6);
    motion_limits_.max_accel = declare_parameter<double>("max_accel", 2.0);
    motion_limits_.normal_decel = declare_parameter<double>("normal_decel", 2.0);
    motion_limits_.emergency_decel = declare_parameter<double>("emergency_decel", 3.0);
    motion_limits_.max_lateral_accel = declare_parameter<double>("max_lateral_accel", 3.0);
    motion_limits_.terminal_speed = declare_parameter<double>("terminal_speed", 0.0);
    motion_limits_.accel_fraction = declare_parameter<double>("accel_fraction", 0.20);
    motion_limits_.decel_fraction = declare_parameter<double>("decel_fraction", 0.25);

    path_options_.sample_spacing = declare_parameter<double>("sample_spacing", 0.02);
    path_options_.rdp_epsilon = declare_parameter<double>("rdp_epsilon", 0.03);
    path_options_.smooth_half_window = declare_parameter<double>("smooth_half_window", 0.15);
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

    q_diag_ << declare_parameter<double>("q_ex", 120.0),
      declare_parameter<double>("q_ey", 120.0),
      declare_parameter<double>("q_eyaw", 90.0),
      declare_parameter<double>("q_vx", 2.0),
      declare_parameter<double>("q_vy", 0.5),
      declare_parameter<double>("q_vw", 2.0);
    r_diag_ << declare_parameter<double>("r_du_x", 1.5),
      declare_parameter<double>("r_du_y", 1.5),
      declare_parameter<double>("r_du_w", 0.8);

    normal_mpc_accel_ << declare_parameter<double>("a_max_x", 2.0),
      declare_parameter<double>("a_max_y", 3.0),
      declare_parameter<double>("a_max_w", 4.0);
    emergency_mpc_accel_ <<
      declare_parameter<double>("emergency_a_max_x", motion_limits_.emergency_decel),
      declare_parameter<double>("emergency_a_max_y", motion_limits_.emergency_decel),
      declare_parameter<double>("emergency_a_max_w", normal_mpc_accel_(2));

    e_xy_max_ = declare_parameter<double>("e_xy_max", 0.30);
    e_yaw_max_ = declare_parameter<double>("e_yaw_max", 0.5236);
    goal_position_tolerance_ = declare_parameter<double>("goal_position_tolerance", 0.05);
    goal_speed_tolerance_ = declare_parameter<double>("goal_speed_tolerance", 0.05);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.30);
    plan_timeout_ = declare_parameter<double>("plan_timeout", 0.0);
    // Temporary alignment for the current lidar odometry: when the chassis
    // front points along world +x, its quaternion reports approximately +90
    // degrees. Add -90 degrees so all downstream transforms use the physical
    // chassis-forward yaw. Set this parameter to 0 after the odometry frame is
    // corrected at its source.
    odom_yaw_offset_ = declare_parameter<double>("odom_yaw_offset", -kPi / 2.0);

    plan_topic_ = declare_parameter<std::string>("plan_topic", "plan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "OdometryHighFreq");
    controller_topic_ = declare_parameter<std::string>("controller_topic", "cmd_controller");
    cmd_track_topic_ = declare_parameter<std::string>("cmd_track_topic", "cmd_track");
    status_topic_ = declare_parameter<std::string>("status_topic", "tracking_status");
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
    // Temporary single-path test mode: after the first usable path is cached,
    // ignore every later message.  The planner currently publishes at 2 Hz
    // without a path_id, so treating them as replans would perturb a fixed-path
    // tracking test.  Restart tracing_node to select another test path.
    if (have_cached_path_) {
      last_valid_plan_t_ = now();
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ignoring nav_msgs/Path: single-path test mode already has a cached path");
      return;
    }

    if (!validPlanFrame(msg->header.frame_id)) {
      rejectPath(
        "Plan frame '" + msg->header.frame_id + "' does not match expected '" + path_frame_ + "'");
      return;
    }

    // Temporary planner interface: nav_msgs/Path has no path_id.  The first
    // valid message is used as the fixed test path.  When the planner switches
    // to PlanPath, restore same-ID heartbeat filtering and new-ID replanning.

    std::vector<rt::Point2> points;
    points.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      points.push_back({pose.pose.position.x, pose.pose.position.y});
    }

    rt::PathGeometry candidate;
    std::string error;
    if (!candidate.build(points, path_options_, &error)) {
      rejectPath("Cannot build path: " + error);
      return;
    }

    cached_path_ = std::move(candidate);
    have_cached_path_ = true;
    last_valid_plan_t_ = now();
    goal_reached_ = false;
    emergency_stop_ = false;
    path_activation_rejected_ = false;

    RCLCPP_INFO(
      get_logger(), "received nav_msgs/Path: %zu samples, %.3f m, max smooth deviation %.4f m",
      cached_path_.size(), cached_path_.length(),
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
    mpc_state_.vx = msg->twist.twist.linear.x;
    mpc_state_.vy = msg->twist.twist.linear.y;
    mpc_state_.vw = msg->twist.twist.angular.z;

    RCLCPP_INFO_ONCE(
      get_logger(),
      "odometry yaw alignment: raw=%.3f rad (%.1f deg), corrected=%.3f rad (%.1f deg)",
      raw_odom_yaw, raw_odom_yaw * 180.0 / kPi,
      mpc_state_.yaw, mpc_state_.yaw * 180.0 / kPi);

    motion_state_.position = {mpc_state_.x, mpc_state_.y};
    motion_state_.velocity = robot_control::tracing::bodyVelocityToWorld(
      mpc_state_.yaw, mpc_state_.vx, mpc_state_.vy);
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
      const rt::Vector2 velocity_body = robot_control::tracing::worldVelocityToBody(
        reference_yaw, point.velocity);
      robot_control::TrajectoryPoint mpc_point;
      mpc_point.x = point.position.x;
      mpc_point.y = point.position.y;
      mpc_point.yaw = reference_yaw;
      mpc_point.vx = velocity_body.x;
      mpc_point.vy = velocity_body.y;
      mpc_point.vw = point.heading.valid ? point.heading.angular_velocity : 0.0;
      mpc_reference.push_back(mpc_point);
    }

    mpc_->setAccelerationLimits(
      trajectory_status == rt::TrajectoryStatus::EmergencyBraking ?
      emergency_mpc_accel_ : normal_mpc_accel_);

    robot_control::ControlCmd command;
    if (!mpc_->solveMPC(mpc_state_, mpc_reference, command)) {
      publishZero();
      publishStatus(robot_interfaces::msg::TrackingStatus::MPC_FAILURE, "MPC solve failed");
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "MPC solve failed -> zero command");
      return;
    }

    geometry_msgs::msg::Twist output;
    output.linear.x = command.vx;
    output.linear.y = command.vy;
    output.angular.z = command.vw;
    pub_cmd_track_->publish(output);

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
    // nav_msgs/Path has no path_id.  Keep the existing status message ABI, but
    // report 0 until the planner migrates to robot_interfaces/PlanPath.
    status.path_id = 0U;
    status.state = state;
    status.progress = diagnostics_.progress;
    status.remaining_distance = diagnostics_.remaining_length;
    status.stop_deficit = diagnostics_.stop_deficit;
    status.terminal_speed = diagnostics_.terminal_speed_if_unstoppable;
    status.detail = detail;
    pub_status_->publish(status);
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
  double e_xy_max_{0.30};
  double e_yaw_max_{0.5236};
  double goal_position_tolerance_{0.05};
  double goal_speed_tolerance_{0.05};
  double odom_timeout_{0.30};
  double plan_timeout_{0.0};
  double odom_yaw_offset_{-kPi / 2.0};

  std::string plan_topic_;
  std::string odom_topic_;
  std::string controller_topic_;
  std::string cmd_track_topic_;
  std::string status_topic_;
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
  bool have_locked_yaw_{false};
  bool goal_reached_{false};
  bool emergency_stop_{false};
  bool path_activation_rejected_{false};
  double locked_yaw_{0.0};
  std::string last_odom_error_;

  rclcpp::Time last_odom_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_valid_plan_t_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_track_;
  rclcpp::Publisher<robot_interfaces::msg::TrackingStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_plan_;
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
