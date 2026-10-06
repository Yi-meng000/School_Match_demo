#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace robot_control
{
namespace trajectory
{

// The complete library uses one caller-defined world frame and SI units.
struct Point2 // 二维标量点
{
  double x{0.0};
  double y{0.0};
};

struct Vector2 // 二维矢量
{
  double x{0.0};
  double y{0.0};
};

struct MotionState2D // 二维运动状态，包括位置和速度
{
  Point2 position;
  Vector2 velocity;
};

// Yaw is supplied by the caller. The translational library never derives it
// from the path tangent and never constrains it.
struct HeadingReference // 航向参考定义
{
  bool valid{false};  // 该份航向参考是否有效，默认无效，区分有无提供的航向参考以及该航向参考是否有效
  double yaw{0.0};    // 参考偏航角
  double angular_velocity{0.0};       // 参考角速度
  double angular_acceleration{0.0};   // 参考角加速度
};
//using 类型赋别名
//std::function<返回类型(参数类型...)>函数变量，存同种类型的不同函数，在同一段代码实现中调用不同函数，内部只有一种实现，外部却是不同的函数传入
//内部提供operator bool(),可在if中被判断，判断条件为，内部无调用对象为false, 有调用对象为true
using HeadingProvider = std::function<HeadingReference(double time_s, double arc_length_m)>;

// Defines the fixed spatial speed schedule v_des(s) created at path activation.
// The terminal speed is deliberately fixed at zero.
struct SpeedProfileOptions // 速度规划器配置
{
  double cruise_speed{-1.0};   // 巡航速度
  double nominal_accel{-1.0};  // 最大加速度
  double nominal_decel{-1.0};  // 最大减速度
  double accel_fraction{0.20}; // 加速段占比
  double decel_fraction{0.25}; // 减速段占比

  bool isValid(std::string* error = nullptr) const; // 检查速度配置是否有效，末尾的const限制函数本身不能对结构体内部成员值进行修改
  //函数本身返回值用来判断是否合法，函数传入的指针字符串说明非法原因
};

struct GeneratorOptions // 轨迹生成器配置
{
  double max_activation_offset{0.50};         // 新路径激活时车辆到路径最近点允许的最大距离
  // 路径搜索窗口限制，防止只凭最近点跳出路径
  double local_projection_backtrack{1.0};     // 每周期定位路径进度时，允许从当前弧长向后搜索距离
  double local_projection_lookahead{3.0};     // 每周期定位路径进度时，允许从当前弧长向前搜索距离

  bool isValid(std::string* error = nullptr) const; //检查参数是否合法
};

struct PathSample  // 指定弧长s的几何查询结果，路径参考————>参考侧
{
  Point2 position;            // 世界坐标
  Vector2 tangent{1.0, 0.0};  // 前进方向（切线），转换世界系参考速度
  double arc_length{0.0};     // 从路径起点沿路径走过路径s
  double curvature{0.0};      // 曲率，用于控制法向加速度
};

struct Projection  // 指定世界位置的路径查询结果，实际位置推测值————>反馈侧
{
  double arc_length{0.0};  // 对应沿路径距离s
  double distance{0.0};    // 车离路径最近点的直线距离
};

// PathGeometry preserves the planner's geometry. It only validates finite
// values and removes consecutive duplicate points before building lookup data.
class PathGeometry  // 路径几何查询对象
{
public:
  bool build(const std::vector<Point2>& points, std::string* error = nullptr);  // 构建路径入口；参数：路径点数组，错误信息存放字符串
  bool valid() const { return valid_; }                                         // 查询路径是否有效
  std::size_t size() const { return points_.size(); }                           // 查询路径包含路径点个数
  double length() const { return total_length_; }                               // 查询路径总长度
  const std::vector<Point2>& points() const { return points_; }                 // 返回路径点数组的只读引用
  const std::vector<double>& sampleArcLengths() const { return arc_lengths_; }  // 返回弧长数组的只读引用

  PathSample sample(double arc_length) const;                                   // 沿路径走到弧长s时，对应的几何信息查询；参数：弧长，返回PathSample

  // A non-negative hint restricts the search to the current route branch.
  // Pass a negative hint to explicitly search the complete path.
  Projection project(    // 查询具体位置在路径桑对应进度；参数：位置（必须），弧长位置作为搜索提示（-1表示全局），向起点方向搜索距离，向终点方向搜索距离，返回Projection
    const Point2& position, double arc_length_hint = -1.0,
    double backtrack = 1.0, double lookahead = 3.0) const;                       

private:
  Projection projectRange(const Point2& position, double s_lo, double s_hi) const; // 指定弧长区间的最近位置搜索，在project()内部调用

  bool valid_{false};                   // 路径是否构建成功
  double total_length_{0.0};            // 路径总长度
  std::vector<double> arc_lengths_;     // 每个点对应的弧长距离
  std::vector<Point2> points_;          // 按照路径顺序保存的二维点
  std::vector<Vector2> tangents_;       // 每个点处的切线方向
  std::vector<double> curvatures_;      // 每个点处的曲率
};
// 强类型枚举，不会自动转换成整数
enum class TrajectoryStatus     // 轨迹状态枚举体
{
  NoActivePath,
  Ready,
  PathRejected
};

struct TrajectoryDiagnostics   //  轨迹当前运行状态
{
  TrajectoryStatus status{TrajectoryStatus::NoActivePath};    // 当前状态
  double progress{0.0};                                       // 当前路径进度
  double remaining_length{0.0};                               // 距离终点剩余路径距离
  double speed_at_progress{0.0};                              // 当前进度处期望速度
  // 目前仅做警告，不做其余制动
  double required_peak_deceleration{0.0};                     // 当前路径正弦减速所需最大减速度大小
  bool nominal_decel_exceeded{false};                         // 上述减速度大小是否超过nominal_decel
};

struct ReferencePoint // MPC参考状态
{
  double time_from_now{0.0};            // 距离本次预测还有多久(0.05 0.10 .... 1.00)
  double arc_length{0.0};               // 参考点在路径上的弧长距离
  Point2 position;                      // 参考世界坐标
  Vector2 velocity;                     // 参考世界系速度
  Vector2 acceleration;                 // 参考世界系加速度 (ax,ay)
  double speed{0.0};                    // 速度大小，标量不区分方向
  double tangential_acceleration{0.0};  // 沿路径方向加速度（加速为正，减速为负）
  double curvature{0.0};                // 参考位置曲率
  HeadingReference heading;             // 航向参考
};

// The speed schedule is built exactly once for every accepted path activation.
// Later control ticks only update monotonic measured progress and sample that
// same spatial function, so a given arc length always has one desired speed.
class TrajectoryGenerator // 轨迹生成对象，包含路径几何与速度规划
{
public:
  /**
   * @brief Construct a new Trajectory Generator object
   * 
   * @param speed_options      速度规划器配置
   * @param generator_options  轨迹生成器配置（默认包含一个使用默认值的配置对象）
   */
  explicit TrajectoryGenerator(
    const SpeedProfileOptions& speed_options,
    const GeneratorOptions& generator_options = GeneratorOptions{});

  bool setSpeedProfileOptions(
    const SpeedProfileOptions& options, std::string* error = nullptr); // 检查并设置速度规划器配置
  const SpeedProfileOptions& speedProfileOptions() const { return speed_options_; } // 返回当前速度配置的只读引用

  bool activatePath(
    const PathGeometry& geometry, const MotionState2D& current_state,
    std::string* error = nullptr); // 把构建好的路径传入生成器，建立运动规划
  void clearPath(); // 清除当前路径，重置轨迹生成器状态，保留path_id

  bool hasActivePath() const { return active_; }  // 查询当前是否有激活的路径
  const PathGeometry& path() const { return path_; }    // 返回当前路径几何对象的只读引用
  const TrajectoryDiagnostics& diagnostics() const { return diagnostics_; }

  double desiredSpeed(double arc_length) const;
  double profileStartArcLength() const { return profile_.start_arc_length; }
  double profileDuration() const;

  // Reference sample zero is dt seconds in the future.
  TrajectoryStatus makeHorizon(
    const MotionState2D& current_state, double dt, std::size_t steps,
    std::vector<ReferencePoint>* out,
    const HeadingProvider& heading_provider = HeadingProvider{});

private:
  struct SinusoidalProfile
  {
    bool valid{false};
    bool direct_deceleration{false};
    double start_arc_length{0.0};
    double length{0.0};
    double start_speed{0.0};
    double peak_speed{0.0};
    double accel_distance{0.0};
    double cruise_distance{0.0};
    double decel_distance{0.0};
    double accel_duration{0.0};
    double cruise_duration{0.0};
    double decel_duration{0.0};
  };

  bool buildProfile(double start_arc_length, double start_speed);
  double profileTimeAtArcLength(double arc_length) const;
  ReferencePoint sampleProfileTime(double profile_time) const;

  SpeedProfileOptions speed_options_;
  GeneratorOptions generator_options_;
  PathGeometry path_;
  bool active_{false};
  double progress_{0.0};
  SinusoidalProfile profile_;
  TrajectoryDiagnostics diagnostics_;
};

}  // namespace trajectory
}  // namespace robot_control
