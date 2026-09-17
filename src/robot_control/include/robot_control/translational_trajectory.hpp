#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace robot_control
{
namespace trajectory
{

// All positions, velocities and accelerations use one caller-defined world frame.
struct Point2 // 世界坐标系位置
{
  double x{ 0.0 };
  double y{ 0.0 };
};

struct Vector2 // 世界坐标系二维向量
{
  double x{ 0.0 };
  double y{ 0.0 };
};

struct MotionState2D // 当前底盘状态（位置、速度）
{
  Point2 position;
  Vector2 velocity;
};

// The translational library deliberately does not create a heading plan.  A caller
// can use this optional value to merge a separate yaw plan into the MPC reference.
struct HeadingReference // 外部yaw参考
{
  bool valid{ false };
  double yaw{ 0.0 };
  double angular_velocity{ 0.0 };
  double angular_acceleration{ 0.0 };
};

using HeadingProvider = std::function<HeadingReference(double time_s, double arc_length_m)>;

struct PathBuildOptions // 路径构建参数
{
  double sample_spacing{ 0.02 };        // 平滑的采样间距m
  double rdp_epsilon{ 0.03 };           // RDP抽稀容差m
  double smooth_half_window{ 0.15 };    // 坐标滑动平均的半窗长度m
  double max_smooth_deviation{ 0.05 };  // 路径与原始折线的最大允许偏差m, relative to the input polyline
  int max_smoothing_attempts{ 5 }; // 平滑超偏差时窗口减半的最大尝试次数
};

// Safety-related values are intentionally invalid until the caller supplies them.
// This prevents an unnoticed default from becoming a vehicle safety limit.
struct MotionLimits // 运动学约束
{
  double cruise_speed{ -1.0 };       // 最高巡航速度 m/s
  double max_accel{ -1.0 };          // 正常加速度上限 m/s^2, normal forward acceleration
  double normal_decel{ -1.0 };       // 正常制动上限 m/s^2
  double emergency_decel{ -1.0 };    // 紧急制动上限 m/s^2, must be >= normal_decel
  double max_lateral_accel{ -1.0 };  // 横向加速度上限 m/s^2
  double terminal_speed{ 0.0 };      // 末速度 m/s; zero stops at the path end
  double accel_fraction{ 0.20 };     // 加速的剩余路径比例 desired fraction of remaining path
  double decel_fraction{ 0.25 };     // 减速的剩余路径比例 desired fraction of remaining path

  bool isValid(std::string* error = nullptr) const; // 检查运动学约束是否合理
};

struct GeneratorOptions // 在线生成的配置
{
  double max_activation_offset{ 0.50 };      // 切换新路径时车辆到新路径最近点最大距离 m: reject paths too far from the robot
  double local_projection_backtrack{ 1.0 };  // 运行过程投影搜索回看距离 m
  double local_projection_lookahead{ 3.0 };  // 运行过程投影搜索前看距离 m
  double profile_spacing{ 0.02 };            // 速度包络离散使用的弧长间隔 m
  double minimum_speed_for_time{ 1e-4 };     // 时间表除零保护的最小速度 m/s
  double max_reference_lead{ 0.10 };         // 名义速度相位最多领先实测投影的弧长 m
};

struct PathSample // 某弧长位置的平滑路径位置、单位切线、弧长、曲率
{
  Point2 position;
  Vector2 tangent{ 1.0, 0.0 };  // unit tangent in the world frame
  double arc_length{ 0.0 };     // m
  double curvature{ 0.0 };      // 1/m, signed
};

struct Projection // 	车辆位置向路径投影后的弧长和投影距离。
{
  double arc_length{ 0.0 };
  double distance{ 0.0 };
};

class PathGeometry
{
public:
  bool build(const std::vector<Point2>& raw_points, const PathBuildOptions& options, std::string* error = nullptr);

  bool valid() const
  {
    return valid_;
  }
  std::size_t size() const
  {
    return arc_lengths_.size();
  }
  double length() const
  {
    return total_length_;
  }
  double maxSmoothDeviation() const
  {
    return max_smooth_deviation_;
  }
  const std::vector<double>& sampleArcLengths() const
  {
    return arc_lengths_;
  }

  PathSample sample(double arc_length) const;

  // With a hint, projection searches the local route branch first and falls back
  // to the entire path if that local match is implausibly far away.
  Projection project(const Point2& position, double arc_length_hint = -1.0, double backtrack = 1.0,
                     double lookahead = 3.0) const;

private:
  bool valid_{ false };
  double total_length_{ 0.0 };
  double max_smooth_deviation_{ 0.0 };
  std::vector<double> arc_lengths_;
  std::vector<Point2> points_;
  std::vector<Vector2> tangents_;
  std::vector<double> curvatures_;

  Projection projectRange(const Point2& position, double s_lo, double s_hi) const;
};

enum class TrajectoryStatus
{
  NoActivePath,
  Ready,
  PathRejected,
  EmergencyBraking,
  EmergencyInfeasible
};

struct TrajectoryDiagnostics
{
  TrajectoryStatus status{ TrajectoryStatus::NoActivePath };
  double progress{ 0.0 };
  double remaining_length{ 0.0 };
  double normal_stop_distance{ 0.0 };
  double emergency_stop_distance{ 0.0 };
  double stop_deficit{ 0.0 };
  double terminal_speed_if_unstoppable{ 0.0 };
};

struct ReferencePoint // 参考轨迹点
{
  double time_from_now{ 0.0 };
  double arc_length{ 0.0 };
  Point2 position;
  Vector2 velocity;
  Vector2 acceleration;
  double speed{ 0.0 };
  double tangential_acceleration{ 0.0 };
  double curvature{ 0.0 };
  HeadingReference heading;
}; 

// Owns the currently accepted geometry and a persistent nominal speed phase.
// Each horizon is position-anchored to the latest measured projection while the
// nominal phase advances across calls. Safety envelopes are still rebuilt from
// measured state on every request.
class TrajectoryGenerator
{
public:
  explicit TrajectoryGenerator(const MotionLimits& limits, const GeneratorOptions& options = GeneratorOptions{});

  bool setLimits(const MotionLimits& limits, std::string* error = nullptr);
  const MotionLimits& limits() const
  {
    return limits_;
  }

  bool activatePath(const PathGeometry& geometry, const MotionState2D& current_state, std::string* error = nullptr);

  void clearPath();
  bool hasActivePath() const
  {
    return active_;
  }
  const PathGeometry& path() const
  {
    return path_;
  }
  const TrajectoryDiagnostics& diagnostics() const
  {
    return diagnostics_;
  }

  // Returns the current status. `out` is cleared on NoActivePath.  Sample zero is
  // at dt seconds in the future, which is the usual first state in an MPC horizon.
  TrajectoryStatus makeHorizon(const MotionState2D& current_state, double dt, std::size_t steps,
                               std::vector<ReferencePoint>* out,
                               const HeadingProvider& heading_provider = HeadingProvider{});

private:
  struct ProfileNode
  {
    double time{ 0.0 };
    double arc_length{ 0.0 };
    double speed{ 0.0 };
    double tangential_acceleration{ 0.0 };
  };

  struct ExactSinusoid
  {
    bool active{ false };
    bool direct_ramp{ false };
    double start_arc_length{ 0.0 };
    double length{ 0.0 };
    double start_speed{ 0.0 };
    double peak_speed{ 0.0 };
    double end_speed{ 0.0 };
    double accel_distance{ 0.0 };
    double cruise_distance{ 0.0 };
    double decel_distance{ 0.0 };
  };

  MotionLimits limits_;
  GeneratorOptions options_;
  PathGeometry path_;
  bool active_{ false };
  double progress_{ 0.0 };
  TrajectoryDiagnostics diagnostics_;
  std::vector<ProfileNode> profile_;
  ExactSinusoid exact_nominal_;
  std::vector<ProfileNode> reference_profile_;
  ExactSinusoid reference_exact_nominal_;
  double reference_time_{ 0.0 };

  bool rebuildProfile(const MotionState2D& current_state, bool apply_nominal_shape);
  ReferencePoint sampleProfile(const std::vector<ProfileNode>& profile, const ExactSinusoid& exact_nominal,
                               double profile_time) const;
  double profileSpeedAtArcLength(const std::vector<ProfileNode>& profile, double arc_length) const;
  double profileDuration(const std::vector<ProfileNode>& profile, const ExactSinusoid& exact_nominal) const;
  double profileTimeAtArcLength(const std::vector<ProfileNode>& profile, const ExactSinusoid& exact_nominal,
                                double arc_length) const;
};

}  // namespace trajectory
}  // namespace robot_control
