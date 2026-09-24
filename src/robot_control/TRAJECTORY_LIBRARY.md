# 全向底盘平动轨迹处理库说明

## 1. 当前实现

`translational_trajectory` 已经接入 `tracing_node`，旧的 `PathTable` 和
`SpeedProfile` 已删除。库本身保持纯 C++；ROS 适配、固定 yaw 和 MPC 坐标转换都在
`tracing_node` 完成。

如果需要按源码逐段阅读（包含节点从等待到运行的状态流、全部成员变量、函数和关键
条件分支），请看 [TRACING_AND_TRAJECTORY_CODE_GUIDE.md](TRACING_AND_TRAJECTORY_CODE_GUIDE.md)。

| 组成 | 文件 | 职责 |
| --- | --- | --- |
| 平动库 | include/robot_control/translational_trajectory.hpp、src/translational_trajectory.cpp | 世界系平动路径、速度包络、曲率限速和紧急制动诊断。 |
| ROS 适配 | src/tracing_node.cpp | 配对接收标准 nav_msgs/Path 与 navigation/PlanMeta 的 path_id；订阅 Odometry、遥控器寻迹开关，完成固定 yaw、MPC 参考和状态话题。 |
| 坐标辅助 | include/robot_control/tracing_adapter.hpp | 世界系/机体系速度旋转及四元数 yaw 提取。 |

新库输出世界坐标系的 vx、vy、ax、ay；MpcController 的速度误差状态则使用机体系
速度。两者不能仅因字段同名就直接赋值，必须先做坐标适配，见第 8 节。

---

## 2. 新库的职责、输入和输出

新库解决的是全向底盘的平动轨迹问题：

- 规划器给出密集、锯齿或栅格化的二维路径点；
- 路径发生变化时，需要从车辆当前位置安全地重新对齐；
- 直线希望平滑地加速、匀速、减速；
- 急弯必须限制速度，不能仅维持全局巡航速度；
- 终点默认停车；
- 路径太短时应报告正常制动或紧急制动是否可行；
- 车头朝向由未来独立的 yaw 规划器决定，不被强制绑定到路径切线。

完整数据流如下：

~~~
规划器二维点列 vector<Point2>
          |
          v
输入校验 -> 去重 -> RDP 抽稀 -> 等弧长重采样 -> 平滑 -> 偏差检查
          |
          v
PathGeometry: p(s), T(s), kappa(s)
          |
          +---- 当前世界系状态 p_vehicle, v_vehicle
          |                 |
          v                 v
新路径投影 / 当前进度 s0 / 当前切向速度 v0
          |
          v
正弦名义速度形状 + 曲率限速 + 加减速可行速度包络
          |
          v
唯一的弧长时间关系 s(t)
          |
          v
未来 N 个 ReferencePoint:
x, y, vx, vy, ax, ay, 可选 yaw、omega、alpha
~~~

该库不读取 CSV，不订阅 ROS 话题，不查询障碍物地图，不进行轮组运动学分配，也不产生 yaw 轨迹。这些工作放在调用方或后续适配层。

### 临时速度对比话题

为验证实车是否真的在跟随路径速度剖面，`tracing_node` 额外发布了临时诊断话题
`/tracking_debug`（`robot_interfaces/TrackingDebug`，reliable、volatile、深度 10）。它只用于
录包和调参，不参与任何控制决策。

每个成功求解 MPC 的控制周期发布一条消息。该话题的
`header.frame_id=map`，所有平动 `Twist.linear` 均在世界系，便于直接对比：

- `nominal_speed_at_progress`：固定名义剖面在实测进度处的 `v_des(s)`；
- `nominal_phase_*` / `nominal_reference`：本周期名义相位对应的速度与弧长；
- `reference_*` / `final_reference`：经过当前可达性与空间安全包络后，真正送给 MPC 的第一项参考；
- `reachability_limited`：因实测速度到名义速度在一个 dt 内不可达而发生夹紧；
- `spatial_safety_limited`：因曲率或终点制动安全包络而发生夹紧；
- `command`：MPC 输出的世界系平动指令；节点随后按当前实测 yaw
  转成车体系后才发布 `/cmd_track`；
- `measured`：已从 Odometry child frame 转到世界系、真正送进 MPC 的最新速度。

建议下一次测试同时录制：

~~~bash
ros2 bag record /tracking_debug /cmd_track /OdometryHighFreq /tracking_status /plan /cmd_controller
~~~

当两个 `*_limited` 都为 false 时，`reference_speed` 应接近名义剖面的速度；若二者频繁为
true，再分别判断是加速度/减速度参数过保守，还是实测速度已经偏离名义状态。

当前 `tracing_node` 默认使用临时测试参数：`max_accel=4.0 m/s²`、
`normal_decel=4.0 m/s²`、`emergency_decel=6.0 m/s²`。为检查减速跟踪，
正常和紧急 MPC 的平动单步增量界默认提高到 `100.0 m/s²`，在 `dt=0.05 s`
时相当于每轴每拍 `5.0 m/s`，对当前 `1.8 m/s` 测试近似不限制。
`a_max_w` 和 `emergency_a_max_w` 也默认 `100.0 rad/s²`，每拍最多改变 `5.0 rad/s`；参考相关的平动速度上界仍生效。
恢复原界时，将 `a_max_x/y` 设为 `4.0`、`emergency_a_max_x/y` 设为 `6.0`，`a_max_w` 和 `emergency_a_max_w` 设为 `4.0`。节点测试模式默认
`enforce_curve_speed_limit=false`、`enforce_dynamic_safety_envelope=false`：MPC 接收固定
名义 `v_des(s)`，不会因实时里程计反馈触发弯前或终点 `Emergency*` 零命令。终点的名义
正弦减速段、里程计超时、人工关闭寻迹、MPC 参考相关速度上界及 yaw 误差归一化仍然保留。

纯 C++ 库的两个开关仍默认开启；这是 `tracing_node` 为底盘能力与雷达速度反馈测试所选的
临时默认值。完成测试后，用下列参数恢复完整安全包络：

~~~bash
ros2 run robot_control tracing_node --ros-args \
  -p enforce_curve_speed_limit:=true \
  -p enforce_dynamic_safety_envelope:=true
~~~

---

## 3. 公开 API

公开头文件为 include/robot_control/translational_trajectory.hpp，命名空间是 robot_control::trajectory。

### 3.1 基础类型

| 类型 | 含义 |
| --- | --- |
| Point2 | 世界坐标系二维位置，单位 m。 |
| Vector2 | 世界坐标系二维向量，用于速度、加速度、单位切线。 |
| MotionState2D | 当前车辆状态，包含世界系 position 和 velocity。 |
| PathSample | 某弧长位置的平滑路径位置、单位切线、弧长、带符号曲率。 |
| Projection | 车辆位置向路径投影后的弧长和投影距离。 |
| ReferencePoint | 未来某个预测时刻的平动参考。 |

ReferencePoint 各字段：

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| time_from_now | s | 从当前控制时刻起算的未来时间。 |
| arc_length | m | 当前参考对应的路径弧长 s。 |
| position | m | 世界系参考位置。 |
| velocity | m/s | 世界系参考速度。 |
| acceleration | m/s² | 世界系参考加速度。 |
| speed | m/s | 沿路径切线的标量速度。 |
| tangential_acceleration | m/s² | 沿路径切线的加速度。 |
| curvature | 1/m | 带正负号的曲率。 |
| heading | - | 可选外部 yaw 参考；由调用方提供。 |

### 3.2 路径构建配置 PathBuildOptions

| 字段 | 默认值 | 作用 |
| --- | ---: | --- |
| sample_spacing | 0.02 m | 弧长表、曲率和几何平滑的采样间距。 |
| rdp_epsilon | 0.03 m | RDP 抽稀容差。适合约 0.025 m 的栅格路径起始值。 |
| smooth_half_window | 0.15 m | 坐标滑动平均的半窗长度。 |
| max_smooth_deviation | 0.05 m | 平滑路径与原始折线的最大允许偏差。 |
| max_smoothing_attempts | 5 | 平滑超偏差时窗口减半的最大尝试次数。 |

### 3.3 物理限制 MotionLimits

安全相关字段故意没有安全默认值；默认是 -1，必须在调用方填写。

| 字段 | 单位 | 作用 |
| --- | --- | --- |
| cruise_speed | m/s | 无局部约束时的最高巡航速度。 |
| max_accel | m/s² | 正常加速上限。 |
| normal_decel | m/s² | 正常、可重复使用的制动上限。 |
| emergency_decel | m/s² | 紧急制动上限，必须不低于 normal_decel。 |
| max_lateral_accel | m/s² | 曲线运动时允许的横向加速度上限。 |
| terminal_speed | m/s | 路径末端速度；默认 0，即停车。 |
| accel_fraction | 无量纲 | 希望用于加速的剩余路径比例。 |
| decel_fraction | 无量纲 | 希望用于减速的剩余路径比例。 |

MotionLimits::isValid 会检查：

- 所有数值有限；
- 速度、加速度、减速度和横向加速度为正；
- terminal_speed 非负且不超过 cruise_speed；
- emergency_decel 不小于 normal_decel；
- 两个比例非负。

### 3.4 在线生成配置 GeneratorOptions

| 字段 | 默认值 | 作用 |
| --- | ---: | --- |
| max_activation_offset | 0.50 m | 接收新路径时，车辆到新路径最近点的最大允许距离。 |
| local_projection_backtrack | 1.0 m | 路径运行中，投影搜索的回看范围。 |
| local_projection_lookahead | 3.0 m | 路径运行中，投影搜索的前看范围。 |
| profile_spacing | 0.02 m | 速度包络离散使用的弧长间隔。 |
| minimum_speed_for_time | 1e-4 m/s | 时间表除零保护的最小速度。 |
| max_reference_lead | 0.10 m | 名义速度相位相对实测投影允许的最大弧长超前量；超过后暂停相位推进。 |
| enforce_curve_speed_limit | true | 是否按 `v_curve(s)` 限制激活时的名义剖面。纯库默认开启。 |
| enforce_dynamic_safety_envelope | true | 是否根据每周期实测速度重建可达性、弯前制动和终点制动包络；关闭后输出固定名义 `v_des(s)`，不产生 `Emergency*` 停机。纯库默认开启。 |

### 3.5 yaw 扩展接口 HeadingProvider

新库不规划 yaw，但能透传外部回调：

~~~
using HeadingProvider = std::function<HeadingReference(
  double time_s,
  double arc_length_m)>;
~~~

外部回调可以填充：

- valid：当前是否有有效 yaw 参考；
- yaw：角度；
- angular_velocity：角速度；
- angular_acceleration：角加速度。

库不会要求 yaw 等于路径切线角，也不会替 yaw 施加约束。

---

## 4. PathGeometry 的实现方式

### 4.1 输入校验和去重

PathGeometry::build 的前半段按以下顺序处理：

1. 检查每一个 Point2 是否为有限值，发现 NaN 或 Inf 立即失败；
2. 删除相邻重复点；
3. 去重后少于两个不同点则失败；
4. 所有后续过程均保留首点和终点。

目的：避免零长度边、零除、无定义切线和无意义轨迹。

### 4.2 RDP 抽稀

RDP 是 Ramer-Douglas-Peucker 折线抽稀算法。对一段首尾点构成的线段：

1. 找所有中间点到该线段的最大距离；
2. 若最大距离超过 rdp_epsilon，以最大距离点将折线递归拆分；
3. 若不超过，认为中间点都是可省略的细碎台阶。

它比每隔固定点数抽样更适合栅格路径，因为固定间隔会保留锯齿，而 RDP 按几何误差保留真正的转向。

### 4.3 等弧长重采样

RDP 后的边长不均匀。库按总长度重新采样，使点列沿弧长近似等间隔，并将首点和末点强制恢复到输入端点。

用途：

- 曲率数值微分的尺度更稳定；
- 可以按 s 查询路径；
- 速度规划更容易使用统一的 delta_s。

### 4.4 平滑、端点固定和偏差检查

平滑方法是对 x、y 分别做滑动平均。

- 窗口超出首尾时，采用局部线段的线性外推，而不是复制端点；
- 线性外推避免纯直线路径被向内拖短；
- 平滑后，首点和末点重新固定到规划器端点，保证起终点不漂移。

由于平滑可能切掉拐角，库计算两条折线的对称最大偏差：

1. 原始点列到平滑折线的最近距离；
2. 平滑点列到原始折线的最近距离。

若偏差超过 max_smooth_deviation：

1. smooth_half_window 减半；
2. 再次平滑、重采样、检查；
3. 尝试次数耗尽仍超限时，build 返回 false。

规划器应预先膨胀障碍物，使实际净空至少满足：

~~~
车体最大外廓半径 + 额外安全余量 + max_smooth_deviation
~~~

库没有地图，无法验证平滑曲线是否撞障碍物；它只能严格执行几何偏差预算。

### 4.5 切线和曲率

中间点使用中心差分得到切线，随后归一化：

~~~
T[i] = normalize(p[i+1] - p[i-1])
~~~

曲率采用三点外接圆等价公式：

~~~
kappa = 2 * cross(p[i]-p[i-1], p[i+1]-p[i])
        / (|p[i]-p[i-1]| * |p[i+1]-p[i]| * |p[i+1]-p[i-1]|)
~~~

曲率正负表示左右转。首尾点使用相邻内点曲率。

### 4.6 sample 和 project

PathGeometry::sample(s)：

- 将 s 限制在有效区间；
- 用二分查找定位相邻弧长节点；
- 线性插值位置、切线和曲率。

PathGeometry::project(position, hint, backtrack, lookahead)：

- 无 hint：全局逐段点到线段投影；
- 有 hint：先在局部弧长窗口搜索；
- 局部最近距离大于 0.5 m 时，认为 hint 失效，退化为全局搜索。

hint 对自交路径特别重要。几何位置相同可能对应不同弧长；给出上一拍进度能保持车辆在正确的路径分支。

---

## 5. 速度规划和时间参数化

### 5.1 当前切向速度

车辆当前位置投影到路径得到 s0 后，当前正向切向速度为：

~~~
v0 = max(0, dot(v_vehicle_world, T(s0)))
~~~

其中 T(s0) 是路径单位切线。

因此横向速度不被误认为进度；负向速度在当前版本按 0 处理。当前库只支持沿路径正向行驶。

### 5.2 名义正弦加速、匀速、减速

无曲率限速和无紧急状态时，每段使用时间域正弦坡：

~~~
v(t) = va + (vb - va)/2 * (1 - cos(pi*t/T))

a(t) = (vb - va)*pi/(2*T) * sin(pi*t/T)

s(t) = (va + vb)/2 * t
       - (vb - va)*T/(2*pi) * sin(pi*t/T)
~~~

正弦坡的段首、段尾加速度都为 0，因此和匀速段连接时不会出现加速度跳变。

给定端速度和段距离，正弦坡峰值加速度为：

~~~
a_peak = pi * |vb^2 - va^2| / (4*d)
~~~

给定加速度上限时，最短可行距离为：

~~~
d_min = pi * |vb^2 - va^2| / (4*a_max)
~~~

所以 accel_fraction 和 decel_fraction 只是期望比例。比例距离不足 d_min 时，库优先满足加速度上限；路径仍不足时，会降低可达峰值速度并退化为无匀速段速度形状。

### 5.3 曲率限速

质心沿曲率为 kappa 的曲线行驶，横向加速度为：

~~~
a_lat = v^2 * |kappa|
~~~

由 max_lateral_accel 得到局部速度上限：

~~~
v_curve(s) = sqrt(max_lateral_accel / |kappa(s)|)
~~~

这不依赖 yaw 是否跟随路径切线。全向底盘即使保持车头不转、以蟹行方式通过弯道，质心速度方向仍会变化，因此仍必须满足横向加速度约束。

### 5.4 弯前提前制动：后向速度传播

仅在弯道处降速是不够的，还要在弯前预留制动距离。

首先构建节点速度帽：

~~~
v_cap = min(cruise_speed, v_curve, v_nominal)
~~~

`cruise_speed` 是期望巡航目标，不是要求实测速度在当前瞬间必须低于它的安全边界。若控制器反馈的切向速度暂时高于巡航值、前方没有更低的曲率限速，包络会按 `normal_decel` 在可用距离内把它平滑降回巡航值；这不会误报为 `EmergencyInfeasible`。曲率限速和终点 `terminal_speed` 仍是硬约束，绝不因这项放宽而提高。

然后从终点向前递推：

~~~
v_back[i] = min(
  v_cap[i],
  sqrt(v_back[i+1]^2 + 2*decel*delta_s)
)
~~~

含义：第 i 个节点的速度不能高到无法在之后的弯道或终点前制动下来。

速度网格包含：

- 固定 profile_spacing 的节点；
- PathGeometry 的所有曲率采样节点。

并且每个小区间两端会按该区间最大曲率共同收紧速度帽。这个处理稍微保守，但避免了曲率峰值恰好落在两个时间节点之间而漏限速。

### 5.5 正常、紧急和不可行状态

| 状态 | 含义 | 上层动作 |
| --- | --- | --- |
| Ready | 正常速度、曲率、终点和制动限制均可满足。 | 正常送 MPC。 |
| PathRejected | 新路径无效、离车太远或配置无效。 | tracing_node 立即输出零速度，不沿旧路径继续运行。 |
| EmergencyBraking | 正常正弦制动来不及，但 emergency_decel 仍可能满足。 | 发送紧急参考并上报。 |
| EmergencyInfeasible | 紧急减速度也无法在后续限速点或终点前满足约束。 | 外部安全层急停或避障，不能假装能停车。 |
| NoActivePath | 没有已接受路径，或运行条件失效。 | 不得继续普通跟踪。 |

TrajectoryDiagnostics 提供：

- progress：当前单调路径进度；
- remaining_length：剩余距离；
- normal_stop_distance：按正常正弦制动的最短停车距离；
- emergency_stop_distance：按紧急正弦制动的最短停车距离；
- stop_deficit：正常模式停车距离缺口；
- terminal_speed_if_unstoppable：紧急也不可行时，到路径末端的残余速度。

### 5.6 唯一的时间关系和输出加速度

若曲率和其他约束没有改变名义形状，库保留解析正弦的 s(t)、v(t)、a(t)。

若曲率限速、终点或紧急制动改变名义形状，库建立速度包络的弧长时间表。相邻节点采用：

~~~
delta_t = 2*delta_s / (v[i] + v[i+1])
~~~

并在该微段内使用恒定切向加速度。这样位置、速度和时间来自同一条 s(t)，而不是先生成位置再事后裁剪速度。

世界系输出关系：

~~~
velocity_world = T * v

acceleration_world = T * a_t + N * kappa * v^2

N = (-T_y, T_x)
~~~

第一项是切向加速度，第二项是法向或向心加速度。

### 5.7 滚动窗口中的持续速度相位

路径激活时，生成器保存一份完整的名义 `time--s--v` 剖面，并把名义时间置零。之后每次
`makeHorizon` 只把名义时间推进一个 `dt`，不会再从正弦加速段的 `t=0` 重新开始。因此即使
车辆最初几拍尚未克服底盘死区，首参考速度也会持续增长，而不是永远重复第一拍的小速度。

名义时间并非无限前跑：它最多比实测投影超前 `max_reference_lead`。同时，每个输出窗口的
位置都从最新实测投影重新积分，第一参考位置只前进一个控制步，不会累计形成永久位置偏置。
每周期仍会从实测位置和速度重建一份独立安全包络，用于曲率限速、加速度可达性、终点制动
以及正常/紧急状态判断。最终参考同时满足持续名义相位和当前安全包络；MPC 还会将每一步
输出速度约束在该参考速度附近，避免仅为追位置窗口而持续超速。

---

## 6. 如何调用新库

### 6.1 基本调用顺序

每当规划器给出一条几何上变化的新路径：

1. 转换成 vector<Point2>；
2. 调用 PathGeometry::build；
3. 用当前世界系位置和速度调用 TrajectoryGenerator::activatePath；
4. 每个控制周期调用 makeHorizon；
5. 根据返回状态和 diagnostics 决定跟踪、降级或急停。

最小示例：

~~~cpp
#include "robot_control/translational_trajectory.hpp"

namespace rt = robot_control::trajectory;

rt::PathBuildOptions geometry_options;
geometry_options.sample_spacing = 0.02;
geometry_options.rdp_epsilon = 0.03;
geometry_options.smooth_half_window = 0.15;
geometry_options.max_smooth_deviation = 0.05;

rt::MotionLimits limits;
limits.cruise_speed = 1.2;
limits.max_accel = 1.5;
limits.normal_decel = 1.8;
limits.emergency_decel = 3.0;
limits.max_lateral_accel = 2.0;
limits.terminal_speed = 0.0;

rt::PathGeometry geometry;
std::string error;
if (!geometry.build(planner_points, geometry_options, &error)) {
  // 记录错误，要求重规划或安全停车。
}

rt::MotionState2D state;
state.position = {odom_x_world, odom_y_world};
state.velocity = {vx_world, vy_world};

rt::TrajectoryGenerator generator(limits);
if (!generator.activatePath(geometry, state, &error)) {
  // 新路径离车太远、路径无效或限制配置无效。
}

std::vector<rt::ReferencePoint> horizon;
const rt::TrajectoryStatus status = generator.makeHorizon(
  state, 0.05, 20, &horizon);

if (status == rt::TrajectoryStatus::EmergencyInfeasible ||
    status == rt::TrajectoryStatus::NoActivePath) {
  // 交给独立安全层处理。
}
~~~

makeHorizon 的第 0 个输出是 dt 秒后的参考点，而不是 t 等于 0 的当前位置；这是标准 MPC 预测窗口的第一步。

### 6.2 传入外部 yaw 规划

~~~cpp
rt::HeadingProvider heading_provider =
  [](double time_s, double arc_length) {
    rt::HeadingReference result;
    result.valid = true;
    result.yaw = planned_yaw(time_s, arc_length);
    result.angular_velocity = planned_omega(time_s, arc_length);
    result.angular_acceleration = planned_alpha(time_s, arc_length);
    return result;
  };

generator.makeHorizon(state, 0.05, 20, &horizon, heading_provider);
~~~

暂时没有 yaw 规划时不传回调即可，ReferencePoint.heading.valid 为 false。

---

## 7. 调参与实车标定顺序

建议严格按以下顺序：

1. 标定纵向 max_accel；
2. 标定可长期重复使用的 normal_decel；
3. 仅在确认机械、电流、轮胎和底盘承受能力后填写 emergency_decel；
4. 在不同曲率弯道实测不打滑、不超轮组能力的 max_lateral_accel；
5. 确认规划器障碍物膨胀能覆盖车体和 max_smooth_deviation；
6. 再调 rdp_epsilon、smooth_half_window 和巡航速度；
7. 若频繁出现 EmergencyBraking，优先降低 cruise_speed 或让规划器更早更新，而不是无限提高紧急减速度。

增大 smooth_half_window 能增大转弯半径、降低曲率峰值，因此通常可以更快过弯；但它同时增加切角风险，必须受 max_smooth_deviation 和规划器净空约束。

---

## 8. ROS 接入、坐标系和生命周期

`tracing_node` 的接口如下：

| Topic | 类型 | QoS/坐标约定 | 用途 |
| --- | --- | --- | --- |
| `plan` | `nav_msgs/Path` | reliable + volatile，depth 1；必须是 `map` | 仅提供路径几何；必须与同 header 的 `plan_meta` 配对。 |
| `/terrain_minco/plan_meta` | `navigation/PlanMeta` | reliable + volatile，depth 1；header 必须与对应 `plan` 完全相同 | 仅读取 `uint64 path_id`；同 ID 不重置，较旧 ID 丢弃，较新 ID 才重建并切换路径。`publish_seq`、`replanned`、`path_start_s`、`reason` 不参与寻迹。 |
| `OdometryHighFreq` | `nav_msgs/Odometry` | best-effort，depth 1；pose=`odom`，twist=`base_link_hf` | 实测位姿和机体系速度。 |
| `cmd_controller` | `robot_interfaces/ControllerCmd` | reliable | 只使用 `trajectory` 字段作寻迹开关。 |
| `cmd_track` | `geometry_msgs/Twist` | reliable，机体系 | MPC 输出；本轮尚未由 commmux 转发到底盘。 |
| `tracking_status` | `robot_interfaces/TrackingStatus` | reliable + transient_local，depth 1 | 当前进度、完成状态和安全故障。 |

当前系统明确假设 `map` 与 `odom` 数值完全重合，不使用 tf2。`tracing_node` 会拒绝
不是 `map` 的路径，以及不是 `odom` / `base_link_hf` 的里程计。若以后地图定位引入
真实的 `map->odom` 漂移，必须改用 tf2 统一坐标系后才能继续使用。

雷达里程计现已将四元数 yaw 与 child-frame twist 修正为真实底盘约定。节点默认
`odom_yaw_offset=0.0`：原始 yaw 直接使用，`raw twist` 先按当前 yaw 转为
MPC 所用的世界系速度。
该参数保留仅用于明确已知的旧版或第三方里程计帧偏置；不得再为当前雷达设置 `-90 deg`，
否则会再次引入重复旋转。

新库只接受世界系状态。Odometry 的 raw twist 先转换到物理车体系，再转换到世界系：

~~~
v_chassis = v_raw  # 默认 odom_yaw_offset = 0
vx_world = cos(yaw)*vx_chassis - sin(yaw)*vy_chassis
vy_world = sin(yaw)*vx_chassis + cos(yaw)*vy_chassis
~~~

MPC 的位置、平动速度、速度误差和平动速度增量全部在世界系中。
轨迹库的世界系参考速度不再转到锁定 yaw 坐标系。MPC 求解后，
`tracing_node` 只在 `/cmd_track` 边界按“当前实测 yaw”转为底盘车体系：

~~~
vx_cmd_body = cos(yaw_actual)*vx_cmd_world + sin(yaw_actual)*vy_cmd_world
vy_cmd_body = -sin(yaw_actual)*vx_cmd_world + cos(yaw_actual)*vy_cmd_world
~~~

### MPC 的参考相关速度上限

速度参考既是 MPC 代价函数中的跟踪目标，也是输出的硬安全边界。对预测窗口第 `k` 点，节点
将平动指令限制为：

~~~
v_target[k] = norm([vx_ref[k], vy_ref[k]]) + reference_speed_margin
v_reachable[k] = max(0, norm([vx_measured, vy_measured]) - a_brake * (k + 1) * dt)
v_limit[k] = max(v_target[k], v_reachable[k])
norm([vx_cmd[k], vy_cmd[k]]) <= v_limit[k]  # 内接八边形实现可能更紧
~~~

默认 `reference_speed_margin=0.05 m/s`，只留给位置纠偏很小的余量；因此参考进入减速段或
终点零速时，MPC 上限也会同步降低。若当前实测速度已经高于该上限，
`reference_speed_braking_decel`（默认 `normal_decel`）使每个未来点的上限按可实现制动能力
逐步下降，避免把 QP 强行变成“当前拍必须瞬停”的不可行问题。紧急制动状态自动改用
`emergency_decel`。可达项只是放宽速度上界，并不强制指令按该斜率制动。

该圆形速度上限在 QP 中由内接八边形实现，故无论蟹行方向如何都不会越过设定的速度模长；
八边形在边中方向的实际速度上限为 `v_limit * cos(π/8)`，可能比参考速度还低。
例如参考 `1.8 m/s`、余量 `0.05 m/s` 且实测未超速时，该方向仅允许约 `1.71 m/s`。
它与每拍 `a_max_x/y` 增量界同时生效；当前节点测试默认值为 `100.0 m/s²`，
使平动增量界在 `1.8 m/s` 测试范围内基本不生效。`q_vx`、`q_vy` 默认均为 `20`，提高速度跟踪
在代价函数中的优先级，但硬上限不依赖权重，不能只靠调权重替代。

开关关闭时生成器清空并持续发布零 `cmd_track`，但缓存最后一条合法路径；开关上升沿锁住
当前 yaw，并从当前位置向该缓存路径重新投影。每个 `/plan` 必须与 `/terrain_minco/plan_meta`
的 `header.stamp` 和 `header.frame_id` 完全一致才会被处理。相同 `path_id` 只更新可选的
`plan_timeout` 保活，不重建也不改变 `progress_`；较旧 ID 会丢弃；更大的 ID 才构建候选几何，
并在启用状态下从当前实测位置切换。

默认 `plan_timeout=0`，不要求规划器周期重发。订阅使用 volatile durability 以兼容普通
nav_msgs/Path 发布者，因此节点晚于规划器启动时，规划器需要再发布一对匹配的消息。里程计
超时始终生效，默认 `0.30 s`。构建此适配功能时 ROS 环境必须提供 `navigation/msg/PlanMeta`；
本地缺少该包时，节点会明确拒绝没有 path_id 的 `/plan`，而不会退回旧的首帧锁存逻辑。

寻迹暂不使用 ROS Action：遥控器开关承担启停，当前 `nav_msgs/Path` 是任务内的重规划更新，
`TrackingStatus` 提供进度和结果。以后需要行为树、抢占和客户端等待结果时，再在
“目标输入—规划—寻迹完成”的整体外层添加 Action。

---

## 9. robot_control 包内每个文件的职责

### 构建与元数据

| 文件 | 职责 |
| --- | --- |
| CMakeLists.txt | 构建 commmux_node、tracing_node 和静态库 translational_trajectory；安装导出库和头文件；注册 GTest。 |
| package.xml | ROS 2 包元数据和依赖声明；包含 ament_cmake_gtest 测试依赖。 |
| LICENSE | 包许可证。 |
| TRAJECTORY_LIBRARY.md | 本说明文档。 |

### 新平动库

| 文件 | 职责 |
| --- | --- |
| include/robot_control/translational_trajectory.hpp | 新库公开类型、配置、状态枚举、PathGeometry 和 TrajectoryGenerator 接口。 |
| src/translational_trajectory.cpp | 新库实现：路径清洗、RDP、重采样、平滑、偏差检查、投影、曲率、速度包络、时间参数化、紧急状态。 |
| test/translational_trajectory_test.cpp | 12 个 GTest：限制校验、异常与重复输入、自交路径投影、端点、单次与滚动起停、静止死区起步、单次与滚动曲率、紧急制动、重规划和 yaw 透传。 |

### MPC 和 ROS 适配

| 文件 | 职责 |
| --- | --- |
| include/robot_control/mpc_controller.hpp | 基于 Eigen 与 OsqpEigen 的 MPC/QP；世界系平动与 yaw 在同一 MPC 中优化，同时约束速度增量与参考相关的平动速度模长；紧急制动时同步切换增量和速度上限的制动斜率。控制器内部不做坐标转换。 |
| include/robot_control/tracing_adapter.hpp | 无 ROS 依赖的坐标边界辅助：Odometry 机体系速度到世界系、MPC 世界系指令到当前车体系、四元数到平面 yaw，以及固定偏置后的 yaw 归一化。 |
| test/tracing_adapter_test.cpp | 验证上述坐标适配和固定 yaw 偏置归一化。 |
| test/tracing_pipeline_test.cpp | 串联持续相位轨迹窗口与 MPC；验证静止起步连续性，以及即使位置误差很大也不能超出参考相关速度上限。 |

### ROS 节点

| 文件 | 职责 |
| --- | --- |
| src/tracing_node.cpp | 当前寻迹节点。临时缓存 nav_msgs/Path、订阅高频里程计和遥控器寻迹位；调用新平动库、锁定 yaw、适配到 MPC、发布 cmd_track 与 tracking_status；处理重规划、完成、超时和紧急状态。 |
| src/commmux_node.cpp | 通信仲裁早期骨架。订阅 cmd_controller 并发布 cmd_chassis；同时发布空 tracing_input。自动/手动仲裁和 cmd_track 订阅仍是 TODO。 |

---

## 10. 构建和测试

~~~bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_control --cmake-args -DBUILD_TESTING=ON
~~~

新库安装为 libtranslational_trajectory.a，并随 robot_control 包导出。

运行轨迹库和坐标适配单元测试：

~~~bash
cd build/robot_control
source /opt/ros/humble/setup.bash
ctest --output-on-failure -R '^(translational_trajectory_test|tracing_adapter_test|tracing_pipeline_test)$'
~~~

当前结果：轨迹库的 13 个 GTest、坐标适配的 6 个 GTest 和 2 个轨迹到 MPC 的管线测试
全部通过。测试包含静止起步连续调用、完整滚动起步/巡航/停车、滚动弯道横向加速度约束、
坐标偏置适配，以及大位置误差下参考相关速度上限仍能强制生效。

---

## 11. 当前范围外的功能

以下功能不属于当前实现范围：

- CSV 读取；
- 地图碰撞检查；
- 动态障碍物检测；
- 倒车或反向路径；
- yaw、角速度和角加速度规划；
- 轮组转角、转角速度、轮速和电流限制；
- commmux 的手动/寻迹仲裁与 `cmd_track` 转发；
- `map` 与 `odom` 不重合时的 tf2 适配；
- Action 形式的高层任务管理。

特别是 EmergencyInfeasible 不是可以忽略的日志状态：它表示给定当前速度、路径剩余长度、曲率速度帽和 emergency_decel 后，仍不存在满足约束的停车轨迹。tracing_node 会在此状态直接发布零 `cmd_track` 并报告 `EMERGENCY_STOP`；未来的 commmux 或独立安全层仍必须确保该急停请求实际传给底盘。
