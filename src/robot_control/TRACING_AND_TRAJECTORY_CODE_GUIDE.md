# 轨迹追踪与平动轨迹库源码指南

本文是 `robot_control` 轨迹追踪代码的唯一维护指南，内容对应当前精简版源码。

当前版本的设计边界非常明确：

- 规划器提供已经平滑、采样合理、完成碰撞检查的 `map` 系路径；
- 轨迹库不再修改规划器几何，只建立弧长查询和固定的空间速度函数 `v_des(s)`；
- yaw 不取路径切线，当前 ROS 节点把一次寻迹任务开始时的 yaw 锁定为参考角度；
- MPC 在世界系内同时跟踪位置、固定 yaw 和速度；
- 坐标转换只出现在 `tracing_node` 的 ROS 边界；
- `commmux_node` 根据手动/寻迹开关选择最终底盘速度来源。

必须特别注意：当前版本没有曲率限速、动态制动安全包络、MPC 硬约束、ROS 输出限幅或紧急制动状态。`Q/S/R` 只能改变优化偏好，不能保证绝对速度、加速度或角速度上限。实车安全边界必须由底盘固件或后续独立安全层提供。

---

## 1. 建议阅读顺序与文件职责

按下面顺序读代码最容易建立完整数据流。

| 顺序 | 文件 | 职责 |
|---|---|---|
| 1 | `robot_control/CMakeLists.txt` | 定义纯 C++ 库、两个 ROS 节点、三个 GTest 目标和依赖关系。 |
| 2 | `robot_interfaces/msg/TrackingDebug.msg` | 定义每个有效控制周期的世界系参考、指令和反馈遥测。 |
| 3 | `robot_interfaces/msg/TrackingStatus.msg` | 定义寻迹生命周期和故障状态。 |
| 4 | `include/robot_control/translational_trajectory.hpp` | 定义几何路径、速度曲线和参考窗口的纯数据接口。 |
| 5 | `src/translational_trajectory.cpp` | 实现点列校验、弧长/切线/曲率、投影、正弦速度曲线和时间采样。 |
| 6 | `include/robot_control/mpc_controller.hpp` | 实现世界系无约束线性 MPC 和 Eigen LDLT 求解。 |
| 7 | `include/robot_control/tracing_adapter.hpp` | 实现 ROS 边界使用的二维坐标旋转、四元数转 yaw 和到达判断。 |
| 8 | `src/tracing_node.cpp` | 配对路径与 path ID、处理开关/里程计、生成参考、调用 MPC 并发布速度。 |
| 9 | `src/commmux_node.cpp` | 在手动速度和 `/cmd_track` 之间仲裁，发布 `/cmd_chassis`。 |
| 10 | `launch/robot_system.launch.py` | 同时启动 `robot_comm` 的 launch 和 `robot_control` 的两个节点。 |
| 11 | `test/*.cpp` | 验证几何、固定速度曲线、LDLT MPC、坐标合同和真实终点条件。 |

已经删除的旧文件和接口：

- `TRAJECTORY_LIBRARY.md`：内容与本文重复且已经过时；
- `robot_interfaces/msg/PlanPath.msg`：实际规划输入是标准 `nav_msgs/msg/Path` 与 `navigation/msg/PlanMeta`；
- `PathBuildOptions`：不再有 RDP、重采样、窗口平滑或偏差上限；
- `MotionLimits`：被只描述速度曲线形状的 `SpeedProfileOptions` 取代；
- OsqpEigen：MPC 已经没有不等式约束，因此直接求解线性方程。

---

## 2. 构建结构和依赖

### 2.1 `robot_control/CMakeLists.txt`

构建文件先查找以下依赖：

| 依赖 | 用途 |
|---|---|
| `ament_cmake` | ROS 2 CMake 包基础。 |
| `rclcpp` | 节点、订阅、发布、定时器和日志。 |
| `geometry_msgs` | `Twist` 指令和调试消息字段。 |
| `robot_interfaces` | `ControllerCmd`、`TrackingDebug` 和 `TrackingStatus`。 |
| `nav_msgs` | `/plan` 与 `/OdometryHighFreq`。 |
| `std_msgs` | 路径和元数据 header 配对。 |
| `Eigen3` | MPC 矩阵计算和 LDLT 分解。 |
| `navigation` | 必需的 `navigation/msg/PlanMeta`。 |

`navigation` 使用 `REQUIRED`。如果构建终端没有 source 规划工作区的安装空间，CMake 会直接失败；这是有意行为，因为没有 `PlanMeta.path_id` 的 `tracing_node` 不能正确区分新旧路径。

构建目标：

| 目标 | 类型 | 链接/依赖 |
|---|---|---|
| `translational_trajectory` | 纯 C++ 库 | 不依赖 ROS，只使用 C++14 标准库。 |
| `tracing_node` | ROS 2 可执行文件 | 依赖轨迹库、Eigen、消息包和 `navigation`。 |
| `commmux_node` | ROS 2 可执行文件 | 依赖 `rclcpp`、`geometry_msgs` 和 `robot_interfaces`。 |
| `translational_trajectory_test` | GTest | 链接轨迹库。 |
| `tracing_pipeline_test` | GTest | 链接轨迹库和 Eigen，重点测 MPC。 |
| `tracing_adapter_test` | GTest | 只测头文件中的坐标与到达辅助函数。 |

安装规则会：

- 把两个节点放到 `lib/robot_control`；
- 安装并导出 `translational_trajectory` 库；
- 安装 `include/`；
- 安装 `launch/`。

### 2.2 `robot_control/package.xml`

`package.xml` 声明与 CMake 一致的运行/构建依赖。`navigation` 是正式依赖，`robot_comm` 是 launch 的运行依赖，`ament_cmake_gtest` 和 lint 包只在测试阶段使用。

### 2.3 `robot_interfaces/CMakeLists.txt`

当前注册四个消息：

- `ChassisCmd.msg`
- `ControllerCmd.msg`
- `TrackingDebug.msg`
- `TrackingStatus.msg`

`PlanPath.msg` 已删除且不再注册。因为调试和状态消息包含 `Header`/`Twist`，接口包继续依赖 `std_msgs` 和 `geometry_msgs`。

---

## 3. ROS 接口、QoS 和坐标系

### 3.1 `tracing_node` 的订阅

| Topic 参数默认值 | 消息 | QoS | 使用内容 |
|---|---|---|---|
| `plan` | `nav_msgs/msg/Path` | reliable、volatile、depth 1 | `header` 和 `poses[].pose.position.x/y`。 |
| `/terrain_minco/plan_meta` | `navigation/msg/PlanMeta` | reliable、volatile、depth 1 | 与 `/plan` 完全相同的 header 和 `path_id`。其他字段不参与控制。 |
| `OdometryHighFreq` | `nav_msgs/msg/Odometry` | best effort、depth 1 | 世界系位置/yaw、车体系平动速度和角速度。 |
| `cmd_controller` | `robot_interfaces/msg/ControllerCmd` | reliable、depth 10 | 只用 `trajectory` 作为寻迹开关。 |

`/plan` 与 `/terrain_minco/plan_meta` 是两个独立 DDS topic，节点不能假设到达顺序。只有二者的：

```text
stamp.sec
stamp.nanosec
frame_id
```

全部相同才组成一对。若不匹配，丢弃较旧的一侧并等待下一条。`publish_seq`、`replanned`、`path_start_s` 和 `reason` 不参与追踪。

### 3.2 `tracing_node` 的发布

| Topic 参数默认值 | 消息 | QoS | 坐标系 |
|---|---|---|---|
| `cmd_track` | `geometry_msgs/msg/Twist` | reliable、volatile、depth 1 | 当前底盘车体系。 |
| `tracking_status` | `robot_interfaces/msg/TrackingStatus` | reliable、transient local、depth 1 | 距离字段对应路径世界系。 |
| `tracking_debug` | `robot_interfaces/msg/TrackingDebug` | reliable、volatile、depth 10 | `reference/command/measured` 全部是世界系。 |

### 3.3 坐标合同

当前系统假定 `map` 与 `odom` 的数值完全重合，不做 TF 查询：

- `/plan.header.frame_id` 必须等于参数 `path_frame`，默认 `map`；
- odometry 的 `header.frame_id` 必须等于 `odom_frame`，默认 `odom`；
- odometry 的 `child_frame_id` 必须等于 `base_frame`，默认 `base_link_hf`；
- odometry pose 的位置和姿态作为世界系状态；
- odometry twist 按 `child_frame_id` 车体系解释；
- `bodyVelocityToWorld(yaw, vx_body, vy_body)` 把实测速度转为世界系；
- 轨迹参考、MPC 状态、MPC 平动输出始终使用世界系；
- `worldVelocityToBody(current_yaw, command_world)` 只在发布 `/cmd_track` 前执行。

没有 `odom_yaw_offset`。雷达发布器必须直接给出正确的底盘 yaw 和 child-frame twist。如果以后 `map -> odom` 不再数值重合，应在节点边界加入 tf2，而不是在 MPC 内加入补偿角。

---

## 4. ROS 消息字段

### 4.1 `TrackingDebug.msg`

该消息用于定量比较固定速度曲线、MPC 输出和雷达反馈。

| 字段 | 含义 |
|---|---|
| `header` | 时间戳；`frame_id` 为路径世界系。 |
| `path_id` | 当前被接受的规划几何版本。 |
| `progress` | 当前单调路径投影弧长。 |
| `remaining_distance` | `path.length - progress`。 |
| `endpoint_distance` | 当前实测位置到真实路径末点的欧氏距离。 |
| `speed_at_progress` | 固定空间速度函数在当前进度处的 `v_des(progress)`。 |
| `reference_arc_length` | horizon 第一个点，即 `dt` 秒后的参考弧长。 |
| `reference_speed` | horizon 第一个点的参考标量速度。 |
| `reference` | 第一个参考点的世界系 `vx/vy/vw`。 |
| `command` | MPC 求出的世界系 `vx/vy/vw`，尚未转为 `/cmd_track` 车体系。 |
| `measured` | 从 odometry 转出的世界系实测 `vx/vy/vw`。 |

`speed_at_progress` 与 `reference_speed` 不一定相等：后者是 `dt` 秒后的前视点，前者是车当前投影位置的明确速度要求。

### 4.2 `TrackingStatus.msg`

字段：

| 字段 | 含义 |
|---|---|
| `header` | 发布时间和路径世界系。 |
| `has_path` | 是否缓存了合法路径。 |
| `path_id` | 最近见到的新路径 ID；尚未收到时为 0。 |
| `state` | 下表中的生命周期状态。 |
| `progress` | 当前单调路径进度。 |
| `remaining_distance` | 沿路径剩余弧长。 |
| `endpoint_distance` | 到真实几何终点的直线距离。 |
| `detail` | 面向日志和调试的状态说明。 |

状态常量：

| 状态 | 行为 |
|---|---|
| `DISABLED` | 寻迹开关关闭，持续发布零 `/cmd_track`。 |
| `WAITING_FOR_ODOMETRY` | 尚未收到合法里程计。 |
| `WAITING_FOR_PATH` | 尚未缓存合法路径。 |
| `TRACKING` | 正常生成 horizon、求解 MPC 和发布指令。 |
| `GOAL_REACHED` | 三个到达条件同时满足，持续发布零。 |
| `PATH_REJECTED` | 路径非法、坐标不符、激活距离过远或 horizon 无法生成。 |
| `MPC_FAILURE` | LDLT 求解或输入有限性检查失败，当前周期发零。 |
| `ODOMETRY_TIMEOUT` | 里程计超过阈值未更新，清活动轨迹并发零。 |

没有 `EmergencyBraking`、`EmergencyInfeasible` 或 `EmergencyStop`。

---

## 5. 平动轨迹库公开类型

命名空间是 `robot_control::trajectory`。

### 5.1 基础数据

| 类型 | 字段 | 含义 |
|---|---|---|
| `Point2` | `x, y` | 世界系位置，单位 m。 |
| `Vector2` | `x, y` | 世界系二维向量。 |
| `MotionState2D` | `position, velocity` | 当前世界系实测平动状态。 |
| `HeadingReference` | `valid, yaw, angular_velocity, angular_acceleration` | 外部 yaw 参考。 |
| `HeadingProvider` | `(time_s, arc_length_m) -> HeadingReference` | 调用方注入 yaw 的函数对象。 |

`HeadingProvider` 与平动曲线完全解耦。当前节点返回锁定 yaw、零角速度、零角加速度；以后可以在关键弧长点给 yaw，但不需要修改平动轨迹库。

### 5.2 `SpeedProfileOptions`

| 字段 | 节点默认值 | 含义 |
|---|---:|---|
| `cruise_speed` | 1.8 m/s | 长路径希望达到的匀速速度。 |
| `nominal_accel` | 4.0 m/s² | 计算达到巡航速度所需最短正弦加速距离。 |
| `nominal_decel` | 3.0 m/s² | 计算正常正弦减速所需最短距离。 |
| `accel_fraction` | 0.20 | 长路径期望给加速段的剩余距离比例。 |
| `decel_fraction` | 0.35 | 长路径期望给减速段的剩余距离比例。 |

`isValid()` 要求速度和名义加减速度为有限正数，两个比例为有限非负数。终点速度不再是参数，固定为零。

### 5.3 `GeneratorOptions`

| 字段 | 默认值 | 含义 |
|---|---:|---|
| `max_activation_offset` | 0.50 m | 新路径激活时，车辆到路径最近点允许的最大距离。 |
| `local_projection_backtrack` | 1.0 m | 控制周期局部投影相对当前进度向后搜索的距离。 |
| `local_projection_lookahead` | 3.0 m | 控制周期局部投影相对当前进度向前搜索的距离。 |

三个字段都必须为有限非负数。它们用于路径有效性和自交路径分支稳定性，不改变 `v_des(s)`。

### 5.4 几何与参考数据

| 类型 | 主要字段 | 含义 |
|---|---|---|
| `PathSample` | `position, tangent, arc_length, curvature` | 指定弧长的几何查询结果。 |
| `Projection` | `arc_length, distance` | 点到路径投影的弧长和欧氏距离。 |
| `ReferencePoint` | `time_from_now, arc_length, position, velocity, acceleration, speed, tangential_acceleration, curvature, heading` | 一个完整 MPC 参考点。 |
| `TrajectoryDiagnostics` | `status, progress, remaining_length, speed_at_progress, required_peak_deceleration, nominal_decel_exceeded` | 当前进度与固定速度曲线诊断。 |

轨迹库状态只有：

- `NoActivePath`
- `Ready`
- `PathRejected`

`required_peak_deceleration` 和 `nominal_decel_exceeded` 只用于提示短路径的零速终点曲线有多激进，不会截停、不重建曲线，也不会改成非零终速。

---

## 6. `PathGeometry`：直接采用规划器几何

### 6.1 `build(points, error)`

构建过程只有四步：

1. 清空旧几何表；
2. 检查每个 x/y 是否有限；
3. 删除距离小于数值 epsilon 的连续重复点；
4. 对剩余点建立累计弧长、单位切线和离散曲率。

如果去重后少于两个点或总长度为零，构建失败。

不会执行：

- RDP 抽稀；
- 等弧长重采样；
- 滑动窗口平均；
- 样条拟合；
- 最大平滑偏差检查；
- 碰撞检查。

因此 `points()` 返回的就是规划器点列去掉连续重复点后的结果，起点、终点和所有非重复中间点均保持不变。

### 6.2 弧长表

对第 i 个点：

```text
s[0] = 0
s[i] = s[i-1] + norm(p[i] - p[i-1])
```

`length()` 返回最后一个累计弧长。

### 6.3 切线

- 起点使用 `p[1] - p[0]`；
- 终点使用 `p[n-1] - p[n-2]`；
- 中间点使用中心弦 `p[i+1] - p[i-1]`；
- 全部归一化为世界系单位向量。

`sample(s)` 在相邻结点之间线性插值位置、切线和曲率，切线插值后再次归一化。这里不会移动几何结点；插值位置仍位于规划器折线段上。

### 6.4 曲率

三个相邻点使用有符号外接圆曲率：

```text
kappa[i] = 2 * cross(p[i]-p[i-1], p[i+1]-p[i])
           / (|p[i]-p[i-1]| * |p[i+1]-p[i]| * |p[i+1]-p[i-1]|)
```

首尾曲率复制相邻内部结点。曲率当前只用于输出法向加速度和调试，不用于限速。

### 6.5 投影

`project(position)` 在全路径所有线段上找最近点，用于新路径激活。

`project(position, hint, backtrack, lookahead)` 只搜索：

```text
[hint - backtrack, hint + lookahead]
```

用于运行中的进度更新。局部搜索可以防止自交路径在交点处从当前分支跳到另一个分支。`TrajectoryGenerator` 还会对投影结果执行：

```text
progress = max(previous_progress, projected_s)
```

因此进度不会回跳。

---

## 7. 固定空间速度函数 `v_des(s)`

### 7.1 激活时的起点

`activatePath()` 先把当前车辆位置全局投影到新路径：

- 投影距离大于 `max_activation_offset` 时拒绝；
- `profile_start_s` 等于投影弧长；
- 初速度等于世界系实测速度在起点切线上的正向投影：

```text
v0 = max(0, dot(v_measured_world, tangent(profile_start_s)))
```

随后只构建一次正弦速度曲线。直到下一次 `activatePath()`，任何后续里程计速度都不会改变这个 `v_des(s)`。

### 7.2 一个正弦速度段

从 `v0` 平滑变化到 `v1`，持续时间 T：

```text
v(t) = v0 + 0.5 * (v1 - v0) * (1 - cos(pi*t/T))

a(t) = 0.5 * (v1 - v0) * pi/T * sin(pi*t/T)

s(t) = 0.5 * (v0 + v1) * t
       - (v1 - v0) * T/(2*pi) * sin(pi*t/T)
```

因为一个完整正弦段的平均速度是 `(v0+v1)/2`，指定段长 d 后：

```text
T = 2*d/(v0+v1)
```

满足峰值加速度 `a_limit` 所需的最短距离：

```text
d_min = pi * abs(v1^2 - v0^2) / (4*a_limit)
```

`inverseRampTime()` 用带二分保护的 Newton 迭代求 `s -> t`，从而保证 `desiredSpeed(s)` 与时间采样使用同一条解析曲线。

### 7.3 正常长路径

先计算从初速度到巡航速度的最短加速距离和从巡航速度到零的最短减速距离。如果二者总和不超过剩余路径：

```text
accel_distance = max(accel_fraction * L, accel_min)
decel_distance = max(decel_fraction * L, decel_min)
cruise_distance = L - accel_distance - decel_distance
```

如果两个期望比例使总长度超过 L，只按加速段和减速段各自多于最短距离的 slack 比例缩短，不会突破名义加减速度要求。

### 7.4 短路径三角曲线

路径不足以达到巡航速度时，匀速段为零。峰值速度由两段最短正弦距离恰好填满 L 得到。

令：

```text
ca = pi/(4*nominal_accel)
cd = pi/(4*nominal_decel)
```

则：

```text
v_peak^2 = (L + ca*v0^2)/(ca+cd)
```

对应距离：

```text
d_accel = ca*(v_peak^2-v0^2)
d_decel = L-d_accel
```

### 7.5 高初速或制动距离不足

满足任一条件时：

- 激活初速高于 `cruise_speed`；
- 按 `nominal_decel` 从当前速度减到零所需距离超过剩余路径；

整段剩余路径直接使用 `v0 -> 0` 的单个正弦减速段。它仍保证参考终点速度严格为零。

所需峰值减速度为：

```text
a_required = pi*v0^2/(4*L)
```

若它超过 `nominal_decel`：

- `nominal_decel_exceeded = true`；
- 节点在路径激活时输出一次 WARN；
- 参考曲线保持不变；
- 不进入紧急停车，不绕过 MPC，不伪造非零终速。

这只是测试期的明确行为，不代表车辆一定能执行该减速度。

---

## 8. 从当前进度生成 MPC 时间窗口

`makeHorizon(current_state, dt, steps, out, heading_provider)` 每周期执行：

1. 在当前进度附近做局部投影；
2. 用 `max(old_progress, projected_s)` 更新单调进度；
3. 求固定速度曲线中该进度对应的 `t(progress)`；
4. 对第 k 个点取：

   ```text
   t_k = t(progress) + (k+1)*dt
   ```

5. 从同一解析曲线得到 `s(t_k)`、速度和切向加速度；
6. 从 `PathGeometry` 取位置、单位切线和曲率；
7. 生成世界系速度：

   ```text
   velocity = tangent * speed
   ```

8. 生成包含切向和法向项的世界系加速度：

   ```text
   normal = [-tangent.y, tangent.x]
   acceleration = tangent * tangential_acceleration
                  + normal * curvature * speed^2
   ```

9. 如果存在 `HeadingProvider`，把其结果原样放进参考点。

注意：窗口由实测弧长进度拖动，不按一次启动后的墙钟时间继续前跑。但是空间速度要求不随反馈变化；在同一个激活周期中，同一个 s 始终对应同一个 `v_des(s)`。

主要公开调用：

```cpp
namespace rt = robot_control::trajectory;

rt::PathGeometry path;
std::string error;
if (!path.build(planner_points, &error)) {
  // reject path
}

rt::SpeedProfileOptions speed;
speed.cruise_speed = 1.8;
speed.nominal_accel = 4.0;
speed.nominal_decel = 3.0;
speed.accel_fraction = 0.20;
speed.decel_fraction = 0.35;

rt::GeneratorOptions generator_options;
rt::TrajectoryGenerator generator(speed, generator_options);
if (!generator.activatePath(path, measured_state, &error)) {
  // too far or invalid
}

std::vector<rt::ReferencePoint> horizon;
const auto status = generator.makeHorizon(
  measured_state, 0.05, 20, &horizon,
  [](double, double) {
    return rt::HeadingReference{true, fixed_yaw, 0.0, 0.0};
  });
```

---

## 9. 无约束世界系 MPC

### 9.1 输入输出

`mpc_controller.hpp` 中的三个结构：

| 类型 | 字段 | 坐标约定 |
|---|---|---|
| `State` | `x,y,yaw,vx,vy,vw` | 位置和 vx/vy 均为世界系。 |
| `TrajectoryPoint` | `x,y,yaw,vx,vy,vw,ax,ay,aw` | 参考位置、速度和加速度的平动量均为世界系。 |
| `ControlCmd` | `vx,vy,vw` | 平动输出仍为世界系。 |

控制器内部不读取 ROS 消息，不做车体/世界系旋转。一次求解还显式接收 `previous_command`，它是上一周期实际发布的世界系命令：

```cpp
bool solveMPC(
  const State& current_state,
  const std::vector<TrajectoryPoint>& reference,
  const ControlCmd& previous_command,
  ControlCmd& command);
```

### 9.2 时间对齐和状态模型

当前里程计状态是 `t=0` 的 `x0`，轨迹库返回的 `reference[0..N-1]` 分别属于 `dt, 2dt, ..., N*dt`。MPC先从 `x0` 预测 `x1..xN`，再逐点比较：

```text
x1 <-> reference[0]
x2 <-> reference[1]
...
xN <-> reference[N-1]
```

状态和控制量：

```text
x = [world_x, world_y, yaw, measured_world_vx, measured_world_vy, measured_w]
u = [command_world_vx, command_world_vy, command_w]
```

底盘速度响应采用三轴一阶模型：

```text
dv/dt = (u - v) / tau
```

令：

```text
F = diag(exp(-dt/tau_x), exp(-dt/tau_y), exp(-dt/tau_w))
G = I - F
L = diag(tau_x, tau_y, tau_w) * G
M = dt*I - L
```

精确离散模型：

```text
p(k+1) = p(k) + L*v(k) + M*u(k)
v(k+1) = F*v(k) + G*u(k)

A = [ I  L ]
    [ 0  F ]

B = [ M ]
    [ G ]
```

`tau` 不是硬限制，而是 MPC 对底盘响应速度的模型。它越大，控制器越认为实测速度对速度命令的跟随更慢。根据 2026-09-24 实车录包的正常运行段，当前默认平动轴为 0.18 s、角速度轴为 0.10 s，后续仍应使用新录包继续标定。

### 9.3 完整未来参考与预测矩阵

未来状态、控制和参考序列：

```text
X = [x1, x2, ..., xN]
U = [u0, u1, ..., u(N-1)]
R = [reference[0], reference[1], ..., reference[N-1]]
```

预测展开：

```text
X = Phi*x0 + Gamma*U
```

`Phi` 的块行为 `A, A^2, ..., A^N`；`Gamma` 是由 `B, AB, A^2B...` 组成的下三角块矩阵。`A/B/Phi/Gamma` 只依赖 N、dt 和 tau，在构造阶段一次建立。

第1到第N个参考点的 `x/y/yaw/vx/vy/vw` 全部进入状态参考和代价。yaw 逐点展开到与上一角度最近的等价角，避免跨越正负 pi 产生假大误差；这不是误差限幅。

### 9.4 参考前馈和反馈修正

由一阶模型可得轨迹速度和加速度对应的连续近似前馈：

```text
u_ff = v_ref + tau * a_ref
```

轨迹库已有世界系 `ax/ay`，固定 yaw provider 提供零 `aw`。完整控制序列写为：

```text
U = U_ff + C
```

`C` 是待求反馈修正。只应用前馈时的未来状态误差：

```text
E0 = Phi*x0 + Gamma*U_ff - R
X - R = E0 + Gamma*C
```

当模型和参考完全一致时，`E0` 接近零，最优 `C` 也接近零，输出自然等于参考前馈。

### 9.5 Q/S/R、修正幅值和指令差分

节点默认：

```text
Q = diag(120, 120, 90, 20, 20, 2)
S = diag(2.0, 2.0, 1.0)
R = diag(1.5, 1.5, 0.8)
```

Q 依次对应：

```text
[世界系x误差, 世界系y误差, yaw误差, 世界系vx误差, 世界系vy误差, 角速度误差]
```

S 对应三轴反馈修正 `C=U-U_ff` 的幅值代价，防止位置纠偏长期把命令大幅推离轨迹前馈。R 对应相邻三轴完整速度命令的变化代价。终端 Q 使用普通 Q 的 3 倍。差分矩阵 `D` 定义：

```text
DeltaU_cmd =
[
  u0 - previous_command,
  u1 - u0,
  ...
  u(N-1) - u(N-2)
]
             = D*U - b
```

目标函数：

```text
J = (X-R)' Q_big (X-R)
  + C' S_big C
  + (D*U-b)' R_big (D*U-b)
```

代入 `U=U_ff+C`：

```text
H = 2 * (
      Gamma' * Q_big * Gamma
    + S_big
    + D' * R_big * D
)

g = 2 * (
      Gamma' * Q_big * E0
    + D' * R_big * (D*U_ff-b)
)
```

`setWeights()` 要求所有 Q 和 S 有限且非负、所有 R 有限且严格为正。`D` 可逆且 `R>0`，因此即使允许某个 S 为零，Hessian 仍保持正定。

### 9.6 Eigen LDLT

每次权重设置后预计算 Hessian，并做一次 LDLT 分解。每个控制周期根据当前状态、完整参考、前馈和上一命令更新梯度，然后求：

```text
H * C = -g
```

输出取第一步：

```text
command = U_ff.head(3) + C.head(3)
```

`lastOptimalityResidual()` 保存 `max(abs(H*C+g))`，用于测试求解精度。

控制器里不存在：

- OSQP 或稀疏约束矩阵；
- 速度或速度增量上下界；
- 最大速度、加速度或角速度；
- 参考相关速度帽；
- 内切八边形；
- 位置或 yaw 误差截断；
- 输出饱和。

---

## 10. `tracing_node` 生命周期

### 10.1 构造顺序

构造函数依次：

1. 声明并读取全部参数；
2. 校验速度曲线和投影参数；
3. 构造 `TrajectoryGenerator`；
4. 用三轴响应时间常数构造 `MpcController` 并设置 Q/S/R；
5. 创建四个订阅；
6. 创建三个发布器；
7. 以 dt 创建控制定时器。

### 10.2 新路径

`tryAcceptPendingPlan()` 配对相同 header 后：

- `path_id == latest_path_id`：重复消息，忽略；
- `path_id < latest_path_id`：过期消息，忽略并警告；
- `path_id > latest_path_id` 或第一条 ID：检查并接受新几何。

接受新 ID 时：

1. 要求 `/plan.header.frame_id == path_frame`；
2. 只读取 poses 的 x/y；
3. 调用 `PathGeometry::build()`；
4. 缓存新路径，清除旧 goal/rejection 状态；
5. 若开关已打开，立刻从当前里程计重新投影并建立新的固定速度曲线；
6. 重规划不改变任务开始时锁定的 yaw。

非法新路径会停止旧活动轨迹并发布零，避免在规划器已经发现障碍物后继续沿旧路径前进。

### 10.3 里程计

`odomCallback()`：

1. 检查 frame 和所有使用数值是否有限；
2. 从四元数直接计算 yaw；
3. 将 child-frame `linear.x/y` 按当前 yaw 旋转到世界系；
4. 填充 `mpc_state_` 和 `motion_state_`；
5. 更新最后里程计时间；
6. 如果寻迹已开、有缓存路径且尚未激活，则尝试激活。

frame 或数值错误会立即清活动轨迹并发布零。

### 10.4 寻迹开关和固定 yaw

只读取 `ControllerCmd.trajectory`：

- 上升沿：清到达/拒绝状态；若已有里程计，锁定当前 `mpc_state_.yaw`，并用实测世界系速度初始化上一拍命令；若路径也存在，立即激活；
- 下降沿：清活动生成器、清锁定 yaw、发布零，但保留最后一条合法路径；
- 再次打开：从新的当前位置重新投影，同时重新锁定当时 yaw。

新的 path ID 在同一次寻迹任务中不会更新 `locked_yaw_`。

### 10.5 每个控制周期

`controlTick()` 按顺序检查：

1. 开关是否关闭；
2. 是否有合法里程计；
3. 里程计是否超过 `odom_timeout`；
4. 是否有缓存路径；
5. 是否已经到达；
6. 最新路径激活是否被拒绝；
7. 是否需要重新激活生成器；
8. 是否成功生成 N 点参考窗口；
9. 是否满足真实到达条件；
10. MPC 是否求解成功。

任何等待或故障分支都发布零 `/cmd_track`，同时把记录的上一拍世界系命令置零。正常分支把轨迹位置、速度、加速度和固定 yaw 填入完整世界系 MPC 参考，并传入上一拍世界系命令；求得新世界系命令后，保存它供下一周期计算指令变化，再按当前实测 yaw 转成车体系发布。

### 10.6 真实终点判断

必须同时满足：

```text
remaining_distance <= goal_position_tolerance
endpoint_euclidean_distance <= goal_position_tolerance
measured_planar_speed <= goal_speed_tolerance
```

`remaining_distance` 来自单调投影，可能在车辆从终点附近飘走后仍保持为零；因此不能单独作为到达条件。新增的 `endpoint_distance` 防止在远离真实终点处误报完成。

---

## 11. `tracing_node` 参数全集

### 11.1 时间与速度曲线

| 参数 | 默认值 | 用途 |
|---|---:|---|
| `N` | 20 | MPC horizon 点数。 |
| `dt` | 0.05 s | 控制周期和参考采样周期。 |
| `cruise_speed` | 1.8 m/s | 目标巡航速度。 |
| `nominal_accel` | 4.0 m/s² | 正弦加速的名义峰值限制。 |
| `nominal_decel` | 3.0 m/s² | 正弦减速的名义峰值限制。 |
| `accel_fraction` | 0.20 | 长路径加速段期望占比。 |
| `decel_fraction` | 0.35 | 长路径减速段期望占比。 |

### 11.2 路径激活和投影

| 参数 | 默认值 | 用途 |
|---|---:|---|
| `max_activation_offset` | 0.50 m | 新路径距车辆过远时拒绝。 |
| `local_projection_backtrack` | 1.0 m | 局部投影回看范围。 |
| `local_projection_lookahead` | 3.0 m | 局部投影前看范围。 |

### 11.3 MPC 权重

| 参数 | 默认值 | 用途 |
|---|---:|---|
| `q_ex` | 120 | 世界系 x 位置误差。 |
| `q_ey` | 120 | 世界系 y 位置误差。 |
| `q_eyaw` | 90 | 锁定 yaw 误差。 |
| `q_vx` | 20 | 世界系 vx 误差。 |
| `q_vy` | 20 | 世界系 vy 误差。 |
| `q_vw` | 2 | 角速度误差。 |
| `s_correction_x` | 2.0 | 世界系 x 命令偏离参考前馈的幅值代价。 |
| `s_correction_y` | 2.0 | 世界系 y 命令偏离参考前馈的幅值代价。 |
| `s_correction_w` | 1.0 | 角速度命令偏离参考前馈的幅值代价。 |
| `r_du_x` | 1.5 | x 速度增量代价。 |
| `r_du_y` | 1.5 | y 速度增量代价。 |
| `r_du_w` | 0.8 | 角速度增量代价。 |
| `mpc_tau_x` | 0.18 s | 世界系 x 实测速度对命令的一阶响应时间常数。 |
| `mpc_tau_y` | 0.18 s | 世界系 y 实测速度对命令的一阶响应时间常数。 |
| `mpc_tau_w` | 0.10 s | 实测角速度对命令的一阶响应时间常数。 |

### 11.4 到达、超时、topic 与 frame

| 参数 | 默认值 |
|---|---|
| `goal_position_tolerance` | 0.05 m |
| `goal_speed_tolerance` | 0.05 m/s |
| `odom_timeout` | 0.30 s |
| `plan_topic` | `plan` |
| `plan_meta_topic` | `/terrain_minco/plan_meta` |
| `odom_topic` | `OdometryHighFreq` |
| `controller_topic` | `cmd_controller` |
| `cmd_track_topic` | `cmd_track` |
| `status_topic` | `tracking_status` |
| `debug_topic` | `tracking_debug` |
| `path_frame` | `map` |
| `odom_frame` | `odom` |
| `base_frame` | `base_link_hf` |

已经不存在的参数包括所有几何平滑参数、曲率/动态包络参数、紧急减速度、终点速度、MPC 加速度界、参考速度帽、误差截断、路径保活超时和雷达 yaw 偏置。

---

## 12. 长期成员变量

### 12.1 `PathGeometry`

| 成员 | 用途 |
|---|---|
| `valid_` | 构建是否成功。 |
| `total_length_` | 总弧长。 |
| `arc_lengths_` | 每个保留规划点的累计弧长。 |
| `points_` | 去掉连续重复点后的原规划点。 |
| `tangents_` | 每个结点的世界系单位切线。 |
| `curvatures_` | 每个结点的离散有符号曲率。 |

### 12.2 `TrajectoryGenerator`

| 成员 | 用途 |
|---|---|
| `speed_options_` | 固定速度曲线参数。 |
| `generator_options_` | 激活和局部投影范围。 |
| `path_` | 当前活动几何的值拷贝。 |
| `active_` | 是否能生成参考。 |
| `progress_` | 单调实测投影弧长。 |
| `profile_` | 当前激活周期唯一的解析正弦曲线参数。 |
| `diagnostics_` | 进度、当前位置速度要求和激进减速提示。 |

`SinusoidalProfile` 内部字段：

| 字段 | 用途 |
|---|---|
| `valid` | 曲线是否可采样。 |
| `direct_deceleration` | 是否为整段 `v0 -> 0`。 |
| `start_arc_length` | 该曲线在完整路径中的起始 s。 |
| `length` | 从激活投影到终点的长度。 |
| `start_speed` | 激活时实测正向切向速度。 |
| `peak_speed` | 巡航或三角曲线峰值速度。 |
| `accel/cruise/decel_distance` | 三阶段距离。 |
| `accel/cruise/decel_duration` | 三阶段持续时间。 |

### 12.3 `MpcController`

| 成员 | 用途 |
|---|---|
| `horizon_`, `dt_` | 预测步数和步长。 |
| `response_time_constants_` | vx/vy/w 的一阶速度响应时间常数。 |
| `q_diag_`, `correction_diag_`, `r_diag_` | 状态误差、前馈修正幅值和相邻完整命令变化权重。 |
| `state_matrix_`, `input_matrix_` | 单步位置/实测速度模型 A/B。 |
| `phi_`, `gamma_` | 从当前状态和未来命令预测完整未来状态。 |
| `difference_matrix_` | 构造相邻完整速度命令之差。 |
| `q_big_`, `correction_big_`, `r_big_` | horizon 块对角状态、修正幅值和命令变化权重。 |
| `hessian_` | 固定无约束二次型 Hessian。 |
| `ldlt_` | Hessian 分解，循环中直接回代。 |
| `factorization_valid_` | 权重和分解是否可用。 |
| `last_optimality_residual_` | 最近一次一阶最优条件残差。 |

### 12.4 `TracingNode`

配置成员：

- `horizon_`, `dt_`
- `speed_options_`, `generator_options_`
- `q_diag_`, `correction_diag_`, `r_diag_`, `mpc_response_time_constants_`
- `goal_position_tolerance_`, `goal_speed_tolerance_`, `odom_timeout_`
- 十个 topic/frame 字符串参数

核心对象与数据：

| 成员 | 用途 |
|---|---|
| `mpc_` | 无约束 MPC。 |
| `generator_` | 固定 `v_des(s)` 参考生成器。 |
| `cached_path_` | 最后一条合法规划几何。 |
| `motion_state_` | 轨迹库使用的世界系平动状态。 |
| `diagnostics_` | 最近一次 horizon 的诊断快照。 |
| `mpc_state_` | MPC 使用的世界系位姿和速度。 |
| `previous_world_command_` | 上一周期实际发布的世界系速度命令；零输出分支同步清零。 |

状态成员：

| 成员 | 用途 |
|---|---|
| `tracking_enabled_` | 控制器寻迹开关当前值。 |
| `have_odom_` | 是否收到合法里程计。 |
| `have_cached_path_` | 是否缓存合法路径。 |
| `have_seen_path_id_` | 是否已经接受过 path ID。 |
| `have_locked_yaw_` | 当前任务是否锁定参考 yaw。 |
| `goal_reached_` | 到达状态锁存。 |
| `path_activation_rejected_` | 最新路径因距离等原因无法激活。 |
| `locked_yaw_` | 一次寻迹任务开始时的 yaw。 |
| `latest_path_id_` | 最新接受/处理的新几何 ID。 |
| `last_odom_error_` | 等待状态中回报的里程计错误。 |
| `last_odom_t_` | 超时判断基准。 |

DDS 对象成员包括三个 publisher、四个 subscription、一个 timer，以及用于配对两个路径 topic 的 `pending_plan_` 和 `pending_plan_meta_`。

---

## 13. `commmux_node` 与 launch

### 13.1 `commmux_node`

该节点本次没有修改。它订阅：

- `cmd_controller`：保存使能、保护、寻迹开关和手动速度；
- `cmd_track`：保存寻迹节点的车体系速度。

100 Hz 定时器发布 `cmd_chassis`：

```text
trajectory == true  -> 使用 track_vel
trajectory == false -> 使用 controller_vel
```

无论选哪组速度，`enable` 和 `protect` 都来自 `ControllerCmd`。

### 13.2 `robot_system.launch.py`

launch 做三件事：

1. include `robot_comm/launch/robot.launch.py`；
2. 启动 `robot_control/commmux_node`；
3. 启动 `robot_control/tracing_node`。

启动命令：

```bash
ros2 launch robot_control robot_system.launch.py
```

---

## 14. 测试覆盖

### 14.1 `translational_trajectory_test.cpp`

覆盖：

- 非法速度参数；
- NaN/Inf 和全重合路径失败；
- 只删除连续重复点；
- 非均匀规划点和端点不变；
- 弧长采样与离散曲率；
- 自交路径的局部投影分支；
- 长路径加速、巡航、零速终点；
- 短路径三角速度曲线；
- 高初速短路径直接正弦减速，只报告名义减速度超限；
- 不同反馈下同一 s 的 `v_des(s)` 不变；
- 进度单调；
- 新路径从新投影和实测切向速度重新建曲线；
- 过远路径拒绝；
- 外部 yaw 原样透传；
- 加速度包含切向和曲率法向项。

### 14.2 `tracing_pipeline_test.cpp`

覆盖：

- 非法响应时间常数和非法 Q/S/R 拒绝；
- 非零 yaw 时平动状态仍是世界系；
- 静止直线起步的第一拍沿参考正方向，重复相同静止状态也不会反向；
- 真实 `TrajectoryGenerator` 正弦起步窗口在车辆尚未移动时重复求解仍保持正向；
- 只修改后半段未来位置就会改变第一拍命令，证明完整参考位置进入优化；
- 增大 S 后同一位置误差产生的命令更接近前馈，证明修正幅值代价实际生效；
- 固定 yaw 误差产生正确方向角速度；
- 大位置误差不截断；
- 不存在八边形造成的方向相关速度界；
- LDLT 解满足 `H*C+g` 近零；
- 按同一阶模型闭环推进时连续求解保持有限。

### 14.3 `tracing_adapter_test.cpp`

覆盖：

- 车体系实测速度转世界系；
- 世界系 MPC 指令转车体系；
- 四元数提取 yaw；
- 剩余弧长、真实终点距离和实测速度三个到达条件必须同时成立。

---

## 15. 构建、启动、录包与调参

### 15.1 构建

先 source ROS，再 source 含 `navigation/msg/PlanMeta` 的规划工作区：

```bash
source /opt/ros/humble/setup.bash
source /path/to/fastlio_nav2/install/setup.bash
cd /home/yimeng/Desktop/rc/mpc_trajectory_ws
colcon build --packages-up-to robot_control --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

测试：

```bash
colcon test --packages-select robot_control
colcon test-result --verbose
```

如果 CMake 报找不到 `navigationConfig.cmake`，说明当前终端没有 source 正确的规划工作区，不应把 `navigation` 改回可选依赖。

### 15.2 启动前核对

```bash
ros2 interface show navigation/msg/PlanMeta
ros2 topic info /plan -v
ros2 topic info /terrain_minco/plan_meta -v
ros2 topic info /OdometryHighFreq -v
```

确认 `/plan` 和 PlanMeta header 完全相同，且新几何才增加 `path_id`。

### 15.3 录包

推荐一次记录完整追踪链：

```bash
ros2 bag record \
  /plan \
  /terrain_minco/plan_meta \
  /OdometryHighFreq \
  /cmd_controller \
  /cmd_track \
  /cmd_chassis \
  /tracking_debug \
  /tracking_status
```

消息结构已经发生不兼容变化，旧 bag 中的 `/tracking_debug` 和 `/tracking_status` 不能直接按新定义反序列化；旧 bag 的 `/plan`、odometry 和速度 topic 不受影响。

### 15.4 推荐调参顺序

1. 低速直线确认 odometry yaw、body twist、世界系路径和 `/cmd_track` 方向一致；
2. 观察 `speed_at_progress`、`reference_speed`、`command`、`measured`，先确认雷达速度反馈可信；
3. 设置 `cruise_speed`、`nominal_accel`、`nominal_decel` 和两个距离比例；
4. 用命令与实测速度的阶跃或录包关系标定 `mpc_tau_x/y/w`，不要把纯通信延迟全部误当作时间常数；
5. 调 `q_vx/q_vy`，决定预测实测速度贴近参考速度的程度；
6. 调 `s_correction_x/y`，限制位置纠偏长期把命令推离前馈的程度；
7. 调 `r_du_x/r_du_y`，控制相邻完整命令的平滑程度；
8. 调 `q_ex/q_ey`，控制路径纠偏强度；
9. 调 `q_eyaw/s_correction_w/r_du_w`，验证固定 yaw，不要通过坐标混用补偿；
10. 用长直线、弯道、绕障和短终点分别录包；
11. 最后在底盘固件或独立安全层恢复经过实测的速度、加速度和角速度保护。

提高位置 Q 会让 MPC 更愿意追位置；提高速度 Q 会让预测实测速度更接近 `v_des(s)`；提高 S 会让命令更接近 `u_ff`，但也会削弱位置纠偏；提高 R 会减小相邻完整命令变化。tau 过小会高估底盘响应能力，tau 过大则会产生更强的提前量。由于当前没有硬约束，任何权重或模型参数都不能替代最终安全边界。

---

## 16. 当前版本明确不负责的内容

- 规划器路径平滑、重采样和碰撞检查；
- 根据曲率自动降速；
- 实测状态驱动的可达性夹紧；
- 正常/紧急制动包络；
- MPC 速度、加速度、角速度硬约束；
- ROS 输出限幅；
- 非零终点速度；
- 从路径切线推导 yaw；
- `map -> odom` TF 漂移补偿；
- 舵轮分配和底盘执行器控制。

这份划分使调试关系保持清楚：规划器决定几何，轨迹库决定固定 `v_des(s)`，MPC 根据 Q/S/R 做闭环跟踪，ROS 边界负责坐标和生命周期，底盘安全层负责最终物理保护。
