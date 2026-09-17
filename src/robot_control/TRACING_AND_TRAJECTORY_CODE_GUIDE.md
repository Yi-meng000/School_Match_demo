# tracing_node 与平动轨迹库源码阅读指南

这份文档对应当前工作区的下列三个文件：

- src/robot_control/src/tracing_node.cpp
- src/robot_control/include/robot_control/translational_trajectory.hpp
- src/robot_control/src/translational_trajectory.cpp

目的不是重复接口注释，而是回答三个阅读源码时最容易混在一起的问题：

1. tracing_node 从收到消息、等待条件满足，到每 50 ms 输出一次底盘速度，实际依次做了什么；
2. 轨迹库如何把离散点列变成可按弧长查询的几何路径，再变成按时间查询的速度、加速度参考；
3. 每个类型、成员变量、函数和关键条件分支为什么存在，以及它会改变什么状态。

文中“第 Lx--Ly 行”均以当前源码文件的行号为准。以后源码增删行时，行号会变化；函数名和变量名仍是更可靠的定位方式。

## 1. 先建立总图：谁调用谁，数据在哪个坐标系

    /plan : nav_msgs/Path（临时接口）
          |  header.frame_id, poses[].pose.position.{x,y}
          v
    tracing_node::planCallback
          |  PathGeometry::build
          v
    cached_path_ --------------+
                               |
    /OdometryHighFreq ---------+--> MotionState2D(世界系位置、世界系速度)
          |                    |    State(MPC 用的位置、机体系速度)
          v                    |
    tracing_node::odomCallback |
                               v
    timer, 每 dt 秒 --> controlTick
                        |
                        +--> TrajectoryGenerator::makeHorizon
                        |      返回 N 个 ReferencePoint：
                        |      世界系 x/y, vx/vy, ax/ay, s, 曲率
                        |
                        +--> worldVelocityToBody(锁定 yaw, 世界速度)
                        |
                        +--> MpcController::solveMPC
                        |
                        +--> /cmd_track : Twist（底盘机体系 vx/vy/vw）

本节点目前的一个明确前提是：map 与 odom 的数值完全重合。它不是 TF 变换，也没有查询 map 到 odom 的坐标变换。路径位置按 map 数字使用，里程计位置按 odom 数字使用，直接认为两者是同一世界坐标。若以后 map 相对 odom 会漂移，必须先接入 tf2，不能仅修改 frame_id 字符串。

速度有两种表达，不能混用：

| 名称 | 所在坐标系 | 用途 |
| --- | --- | --- |
| motion_state_.velocity | 世界系 | 给平动轨迹库投影、计算路径切线速度。 |
| mpc_state_.vx / vy | 底盘机体系 | MPC 当前实测速度。 |
| ReferencePoint.velocity | 世界系 | 路径切线乘以标量速度。 |
| TrajectoryPoint.vx / vy | 参考车体系 | 送给现有 MPC 的速度参考。 |
| /cmd_track.linear.x / y | 底盘机体系 | MPC 的输出，尚由 commmux_node 以后的工作接管。 |

两个旋转公式都在 tracing_adapter.hpp：

    v_world.x = cos(yaw) * v_body.x - sin(yaw) * v_body.y
    v_world.y = sin(yaw) * v_body.x + cos(yaw) * v_body.y

反向变换为：

    v_body.x =  cos(yaw) * v_world.x + sin(yaw) * v_world.y
    v_body.y = -sin(yaw) * v_world.x + cos(yaw) * v_world.y

这里的 yaw 不是路径切线方向。第一版在遥控器寻迹开关上升沿把车辆当前 yaw 锁进 locked_yaw_，整个任务期间保持不变，因此全向底盘可以沿任意方向平移而不转车身。

### 1.1 tracing_adapter.hpp 的三个内联函数

这个头文件的函数很短，因而放成 inline；编译器可直接把公式展开到
tracing_node，不需要单独的 cpp 文件。

| 函数 | 每一句的作用 |
| --- | --- |
| bodyVelocityToWorld(yaw, vx_body, vy_body) | L17--18 分别算 cos/sin；L19--20 乘标准二维旋转矩阵 R(yaw)，把 Odometry.twist 的机体系线速度变为轨迹库使用的世界系线速度。 |
| worldVelocityToBody(reference_yaw, velocity_world) | L28--29 算 cos/sin；L30--31 乘 R(yaw) 的转置，即逆旋转，把轨迹库世界速度转成 MpcController 参考速度所用的参考车体系。 |
| yawFromQuaternion(x,y,z,w) | L36--38 用平面 yaw 的 atan2 公式从四元数取得绕 z 的偏航角。它不归一化四元数；输入 Odometry 应提供规范化姿态。 |

这两个速度函数互为逆变换的前提是两次使用同一个 yaw。当前实现中，实测
速度使用当前 mpc_state_.yaw 转世界系，而参考速度使用 locked_yaw_ 转参考车体系；
这是有意的：前者描述“车现在实际怎么运动”，后者描述“固定朝向下 MPC 应怎样平移”。

## 2. tracing_node 的完整生命周期

### 2.1 构造时：只建立能力，不会立刻运动

源码 L45--88 是 TracingNode 构造函数，调用顺序如下。

1. L48 调用 declareParameters()。它从 ROS 参数服务器读取全部数值，填入普通成员变量；此时还没有订阅任何话题。
2. L50--53 调用 MotionLimits::isValid。物理限制缺失、为负、紧急减速度小于正常减速度时，节点直接抛异常退出，避免带着不可信限值启动。
3. L55 创建 generator_。它保存轨迹库的当前激活路径、进度和速度剖面。
4. L56 创建 mpc_。N_ 是预测点数，dt_ 是每一预测点的时间间隔。
5. L57 设置初始位置/yaw 误差限幅；L58 设置 Q、R 和正常状态下的速度增量上限。
6. L60--70 建立三个订阅者；回调只更新缓存或状态，不直接每次都求 MPC。
7. L72--77 建立两个发布者。cmd_track 是实时命令，tracking_status 是诊断状态。
8. L79--81 建立 wall timer。以后真正的控制统一由 controlTick() 以 dt_ 秒周期运行；默认 dt_=0.05，即 20 Hz。
9. L83--87 输出启动日志。此时 tracking_enabled_、have_odom_、have_cached_path_ 都仍是 false，所以控制周期只会发布零速度和 DISABLED。

### 2.2 刚启动后的状态机

controlTick()（L345--508）每个周期按下列优先级检查。上面的条件先命中就 return，后面的动作不会执行。

    寻迹开关关闭
        -> DISABLED，持续发布零 cmd_track

    开关开启但没有合法里程计
        -> WAITING_FOR_ODOMETRY，持续发布零

    有过里程计但最后一次超过 odom_timeout
        -> ODOMETRY_TIMEOUT，清空激活生成器，持续发布零

    有里程计但没有合法路径
        -> WAITING_FOR_PATH，持续发布零

    可选路径保活超时（plan_timeout > 0）
        -> WAITING_FOR_PATH，清空生成器，持续发布零

    紧急制动仍无法在路径末端停住
        -> EMERGENCY_STOP，持续发布零

    已达到终点
        -> GOAL_REACHED，持续发布零

    路径激活被拒绝的锁存位为真
        -> PATH_REJECTED，持续发布零

    其余情况
        -> 激活路径（若还未激活）-> 生成参考 -> 求 MPC -> 发布 cmd_track

所以“路径先到、开关后来”与“开关先开、路径后来”的效果相同：两者只是在条件最后满足的回调不同，直到三个条件 tracking_enabled_、have_odom_、have_cached_path_ 同时为真前，车始终收到零速度。

### 2.3 正常一次寻迹的事件时间线

下面用一次常见顺序说明各回调如何配合。

    1. /plan 到达
       planCallback 把 poses 转为 Point2，构建并缓存 cached_path_。
       因开关还没开，只发 DISABLED，不激活 generator_。

    2. /OdometryHighFreq 到达
       odomCallback 缓存当前位置、yaw、机体系速度和世界系速度。
       若开关未开，到这里仍不运动。

    3. /cmd_controller.trajectory 从 0 变为非零
       controllerCallback 记录 tracking_enabled_=true。
       用当前 mpc_state_.yaw 锁定 locked_yaw_。
       调用 activateCachedPath：把当前车辆位置投影到 cached_path_。

    4. 每个 dt 周期
       controlTick 调用 makeHorizon(state, dt, N)。
       轨迹库从实测世界系速度和当前位置重建未来 N 点的速度包络。
       节点把每个世界速度旋转成固定 yaw 下的机体系速度。
       MPC 输出一个机体系 ControlCmd，节点发布为 /cmd_track。

    5. 靠近路径终点
       剩余弧长 <= 0.05 m，且实测世界平动速度 <= 0.05 m/s。
       节点锁存 goal_reached_，清空 generator_，后续只发布零速度。

终点阈值不是“参考点到末端”的单独判断，而是同时看轨迹库诊断中的 remaining_length 和里程计测得的实际速度。因此参考已经走到末端、但底盘仍滑行时，节点不会立即宣布 GOAL_REACHED。

## 3. tracing_node：每个接口、参数和成员变量

### 3.1 ROS 话题与 QoS

| 源码位置 | 话题与类型 | QoS | 回调中的职责 |
| --- | --- | --- | --- |
| L60--66 | plan_topic_，nav_msgs/Path | reliable、volatile、深度 1 | 单路径测试时仅缓存第一条成功构建的几何路径。volatile 兼容普通 Path 发布者；节点晚启动后必须等规划器再发一帧。 |
| L65--67 | odom_topic_，nav_msgs/Odometry | best_effort、深度 1 | 提供高频实测状态。丢旧包比排队旧包更安全。 |
| L68--70 | controller_topic_，ControllerCmd | reliable、深度 10 | 只读取 trajectory 字段作为开关。 |
| L72--73 | cmd_track_topic_，geometry_msgs/Twist | reliable、深度 1 | 发布底盘机体系速度。 |
| L74--77 | status_topic_，TrackingStatus | reliable、transient_local、深度 1 | 发布最近一份状态，便于诊断节点晚加入后立即读到。 |

当前临时输入 nav_msgs/Path 的字段用途：

| 字段 | 是否使用 | 具体用途 |
| --- | --- | --- |
| header.frame_id | 使用 | 必须严格等于 path_frame_，默认 map。 |
| header.stamp | 未使用 | 不参与过期判断；过期判断使用本节点收到消息的 now()。 |
| poses[].pose.position.x/y | 使用 | 唯一的几何输入。 |
| poses[].pose.orientation | 未使用 | 平动库和第一版固定 yaw 都不读路径点姿态。 |

临时约定：nav_msgs/Path 没有 path_id，节点处于单路径测试模式：第一条成功构建的 Path
被锁存，后续 2 Hz 消息直接忽略，不会重建或重置 progress_。要测试不同路径需重启节点。
等升级到 robot_interfaces/PlanPath 后，才恢复“相同 ID 只保活、不重置进度”的逻辑。

### 3.2 declareParameters（L91--158）

参数按用途分为下面几组；declare_parameter 的第二个参数就是默认值。

| 成员/参数 | 默认值 | 影响的代码 |
| --- | ---: | --- |
| N_ / N | 20 | 一个 MPC 窗口与一个轨迹窗口的点数。 |
| dt_ / dt | 0.05 s | timer 周期、参考点间隔、MPC 离散周期。 |
| motion_limits_.cruise_speed | 1.2 m/s | 直线无约束时的速度上限。 |
| max_accel | 2.0 m/s2 | 轨迹库正向可达速度及正常 MPC 限制的相关设置。 |
| normal_decel | 2.0 m/s2 | 正常制动包络和是否进入紧急制动。 |
| emergency_decel | 3.0 m/s2 | 仅正常制动不够时可使用的更高减速度；必须上车标定。 |
| max_lateral_accel | 3.0 m/s2 | 根据曲率产生的弯道速度上限。 |
| terminal_speed | 0 m/s | 默认路径末端停车。 |
| accel_fraction / decel_fraction | 0.20 / 0.25 | 无局部限速时正弦加/减速段的目标距离比例，硬约束优先。 |
| sample_spacing | 0.02 m | 几何路径等弧长采样间隔。 |
| rdp_epsilon | 0.03 m | RDP 抽稀最大容差。 |
| smooth_half_window | 0.15 m | 滑动平均半窗口的物理长度。 |
| max_smooth_deviation | 0.05 m | 平滑结果相对原始折线允许的最大双向偏差。 |
| max_smoothing_attempts | 5 | 平滑超偏差后半窗口减半的最多次数。 |
| max_activation_offset | 0.50 m | 重规划时车辆离新路径太远则拒绝。 |
| local_projection_backtrack / lookahead | 1.0 / 3.0 m | 每周期投影优先搜索的进度窗口。 |
| profile_spacing | 0.02 m | 速度包络弧长离散间隔。 |
| minimum_speed_for_time | 1e-4 m/s | 从速度积分时间时的除零保护。 |
| max_reference_lead | 0.10 m | 名义速度相位相对实测进度的最大弧长超前量。 |
| q_diag_ | 120,120,90,2,0.5,2 | MPC 六维状态误差权重。 |
| r_diag_ | 1.5,1.5,0.8 | MPC 三个速度增量的惩罚。 |
| normal_mpc_accel_ | 2,3,4 | 正常状态下 vx、vy、vw 速度增量限值。 |
| emergency_mpc_accel_ | 3,3,4 | EmergencyBraking 时替换 MPC 的 vx、vy、vw 限值。 |
| e_xy_max_ / e_yaw_max_ | 0.30 m / 0.5236 rad | 新路径切换时，送进 MPC 前的位置/yaw 误差饱和幅度。 |
| goal_position_tolerance_ / goal_speed_tolerance_ | 0.05 m / 0.05 m/s | GOAL_REACHED 的双条件阈值。 |
| odom_timeout_ | 0.30 s | 最后里程计超过此时长便停车。 |
| plan_timeout_ | 0 s | 0 表示不要求规划器周期保活；临时 nav_msgs/Path 接口应保持 0，等 path_id 接口恢复后才适合正数保活。 |
| odom_yaw_offset_ | -1.57079632679 rad | 临时补偿雷达 child frame 相对实际车头的 +90 度固定偏差；同一偏置的反向旋转也用于把 twist 转进真实车体系。雷达端修正后设为 0。 |
| *_topic_ | 见源码 L150--154 | 允许 launch 文件改话题名。 |
| path_frame_ / odom_frame_ / base_frame_ | map / odom / base_link_hf | 严格检查输入坐标系标识。 |

L95--97 的作用是拒绝 N<1 或 dt<=0。没有这层检查，后面静态转换 N_ 为 size_t 时，负数会变成很大的无符号值，风险很高。

### 3.3 所有长期保存的成员变量（L531--583）

| 成员 | 保存什么 | 谁写入、谁读取 |
| --- | --- | --- |
| N_、dt_ | 控制和预测长度 | declareParameters 写；构造 timer、makeHorizon、MPC 读。 |
| motion_limits_ | 平动物理限制 | 参数读取；构造轨迹库读。 |
| path_options_ | 建几何路径的参数 | 参数读取；planCallback 的 candidate.build 读。 |
| generator_options_ | 在线投影、剖面网格参数 | 参数读取；构造轨迹生成器读。 |
| q_diag_、r_diag_ | MPC Q/R 对角权重 | 参数读取；构造时 setWeights 使用。 |
| normal_mpc_accel_ | 正常速度增量上限 | 参数读取；每个正常 tick 传给 MPC。 |
| emergency_mpc_accel_ | 紧急制动的速度增量上限 | 参数读取；EmergencyBraking tick 传给 MPC。 |
| e_xy_max_、e_yaw_max_ | MPC 初始误差饱和值 | 参数读取；构造时 setErrorLimits 使用。 |
| goal_*_tolerance_ | 到达判定阈值 | controlTick 读。 |
| odom_timeout_、plan_timeout_ | 新鲜度阈值 | controlTick 读。 |
| 各 topic/frame 字符串 | 通信名和坐标名 | 构造订阅、回调校验读。 |
| mpc_ | MPC 求解器对象 | 构造创建；controlTick 调用。 |
| generator_ | 当前路径的时间参数化对象 | 构造创建；回调激活/清空，tick 生成窗口。 |
| cached_path_ | 最近一条已成功构建的几何路径 | planCallback 写；activateCachedPath 读。 |
| motion_state_ | 世界系位置、世界系速度 | odomCallback 写；轨迹库读。 |
| diagnostics_ | 轨迹库上次生成出的进度和制动诊断 | 激活或生成后复制；状态发布读。 |
| mpc_state_ | x/y/yaw 加机体系 vx/vy/vw | odomCallback 写；MPC 求解读。 |
| tracking_enabled_ | 轨迹开关当前电平 | controllerCallback 写；所有控制分支读。 |
| have_odom_ | 是否已有坐标系合法的里程计 | odomCallback 写；控制与激活读。 |
| have_cached_path_ | cached_path_ 是否可用 | planCallback/rejectPath 写；控制读。 |
| have_locked_yaw_ | locked_yaw_ 是否已赋有效值 | 开关和激活时写；激活读。 |
| goal_reached_ | 已达终点后的零速度锁存 | 控制判定写；控制读；新路径或开关变化清除。 |
| emergency_stop_ | 紧急制动仍不可行后的零速度锁存 | controlTick 写；新路径或开关变化清除。 |
| path_activation_rejected_ | 拒绝无效路径后等待新路径的锁存位 | rejectPath 写；controlTick 读。 |
| locked_yaw_ | 任务期间固定的参考车身朝向 | 开关上升沿或激活时写；heading provider 读。 |
| last_odom_error_ | 最近一次不合法里程计的详细原因 | odomCallback 写；等待状态读。 |
| last_odom_t_ | 最近合法里程计的接收时间 | odomCallback 写；超时检查读。 |
| last_valid_plan_t_ | 最近合法 nav_msgs/Path 的接收时间 | planCallback 写；可选保活检查读。 |
| pub_*、sub_*、timer_ | ROS 实体的智能指针 | 构造创建；必须保存，离开构造函数后才不会被析构。 |

## 4. tracing_node 的每个函数逐段解释

### 4.1 validPlanFrame 与 validOdomFrames（L160--168）

validPlanFrame 只有一条比较：nav_msgs/Path.header.frame_id 必须等于 path_frame_。不允许空 frame、也不接受 odom。

validOdomFrames 同时检查两项：

- msg.header.frame_id 必须是 odom_frame_：Odometry.pose 所在世界系；
- msg.child_frame_id 必须是 base_frame_：Odometry.twist 所在车体系。

这保证 L249 的 bodyVelocityToWorld 使用的是正确语义。仅位置 frame 对、速度 frame 错时，旋转结果仍会是错误速度，所以必须二者都检查。

### 4.2 planCallback（L172--230）

| 行 | 做的事 | 为什么 |
| --- | --- | --- |
| L174--184 | 若已有 cached_path_，只更新接收时间、输出节流 debug 日志并 return。 | 2 Hz 的后续路径完全不影响单路径测试。 |
| L186--190 | 只有尚未缓存路径时才检查 frame；错误则 rejectPath。 | 第一条路径仍必须有正确世界系。 |
| L192--194 | 说明临时接口没有 path_id，第一条有效消息固定为测试路径。 | 明确这是过渡逻辑。 |
| L196--200 | 预留 vector 容量，逐点只复制 position.x/y。 | 不读每个 PoseStamped 的 header 和 orientation。 |
| L202--207 | 在局部 candidate 上 build。失败就拒绝。 | 不会缓存半成品。 |
| L209--214 | 成功后 move 到 cached_path_，更新时间，解除“到终点/急停/拒绝”锁存。 | 只有这一次会建立本轮测试路径。 |
| L216--219 | 打印采样数、总长度、实际最大平滑偏差。 | 方便发现规划点异常或平滑过强。 |
| L221--224 | 若已开寻迹，立刻从当前位置投影并激活这条路径。 | 支持开关先开、路径后到。 |
| L225--229 | 若未开寻迹，只缓存并发布 DISABLED。 | 支持路径先到、开关后开。 |

对于“新路径离当前车太远”的情况，activateCachedPath 会先 clearPath，再被 TrajectoryGenerator::activatePath 因 max_activation_offset 拒绝；因此旧路径不会继续执行。这是有意的安全取舍：新规划往往意味着旧路径可能通往新障碍物。

### 4.3 odomCallback（L223--261）

| 行 | 做的事 | 为什么 |
| --- | --- | --- |
| L225--235 | frame 不合法：置 have_odom_=false，记录文本，清空激活轨迹并立即发零。 | 不在坐标语义不明时控制车辆。 |
| L238--246 | 记录当前时间；把 pose 写入 x/y/yaw；将 raw twist 旋转到物理车体系后写入 MPC 的 vx/vy/vw。 | MPC 期待当前速度在底盘系，不能混入雷达 child frame 的分量。 |
| L242--250 | quaternion 经 yawFromQuaternion 取原始平面偏航角，再加 odom_yaw_offset 并归一化；raw twist 同时乘 R(-odom_yaw_offset)。 | 当前默认 yaw 减 90 度，twist 则加 90 度：`vx_chassis=-vy_raw, vy_chassis=vx_raw`，两者共同使车头沿世界 +x 时的姿态与速度语义一致。 |
| L252--254 | 位置直接复用；已校正的机体系线速度用当前 yaw 旋转到世界系。 | 轨迹库依靠速度在路径切线上的投影来确定初速度。 |
| L251 | 置 have_odom_=true。 | 允许 controlTick 进入下一阶段。 |
| L253--260 | 若已开、已有路径、还未激活、且没有终点/急停/拒绝锁存，尝试激活。 | 支持开关和路径均早到、里程计最后到的顺序。 |

L244--246 不旋转 angular.z，因为二维平面绕 z 的角速度在世界系和车体系数值相同；只有 x/y 平动分量需要旋转。

### 4.4 controllerCallback（L263--294）

ControllerCmd 的其他字段在本节点不读。L265 用 trajectory != 0U 转为 bool。

| 行 | 做的事 |
| --- | --- |
| L266--268 | 电平没有变化则什么也不做；这保证遥控器持续发布“开启”不会每帧重置路径。 |
| L270--273 | 记录新开关值，并清除本次任务的终点、急停、路径拒绝锁存。 |
| L274--280 | 下降沿：清空生成器、解除 yaw 锁、清诊断、立即发布零并报 DISABLED；cached_path_ 故意保留。 |
| L283--288 | 上升沿：若已有里程计，锁当前车身 yaw；没有里程计就标记为尚未锁，等后续激活时补锁。 |
| L290--293 | 路径和里程计都齐时立即尝试激活；否则 controlTick 保持等待零速度。 |

重新关闭再打开时，generator_ 已被清空，因此激活时会从新当前位置向同一缓存路径重新投影，不会沿用关闭前的 progress_。

### 4.5 activateCachedPath、rejectPath、rejectActivatedPath（L296--343）

activateCachedPath 是唯一把 cached_path_ 交给 generator_ 的普通入口。

1. L298--303 先确认开关、路径和里程计都存在；缺一项就返回 false 和可读错误。
2. L304--307 确保有固定 yaw。通常开关上升沿已做过；这里是里程计最后到达时的兜底。
3. L311 先清空旧生成器。不能仅在新路径成功后才清空，因为一条被拒绝的新路径也代表规划器发现了环境变化，继续走旧路径有撞障碍风险。
4. L312 调用轨迹库 activatePath；它会做路径/状态有限值检查、最近点投影和最大起始偏差检查。
5. L315 复制初始诊断，L316 清除 rejectPath 留下的路径拒绝锁存。

rejectPath 用于“消息本身”不合法：frame 错、build 失败等。它清空缓存和活动路径，并把 path_activation_rejected_ 置真，因此控制周期只会报 PATH_REJECTED，直到下一条成功构建的 Path 或开关边沿清除它。

rejectActivatedPath 用于“几何构建成功、但当前位置不能安全接入”或前置条件缺失。它清空 generator 并发零。当前代码没有在这个函数里设置 path_activation_rejected_=true，所以如果 cached_path_ 仍存在，下一次 controlTick 会再尝试 activateCachedPath。失败时仍保持零输出，但会有重复的节流错误日志。这是当前实际行为；如果期望“激活一次失败后必须等待下一条 Path”，应在此函数中也置 path_activation_rejected_=true。

### 4.6 controlTick（L345--508）：逐个安全分支

L347 把 now() 复制到 current_time，保证本周期的两个超时判断使用同一个时刻。

| 行 | 条件 | 动作与含义 |
| --- | --- | --- |
| L349--353 | 开关关闭 | 零命令、DISABLED。即使有路径和里程计也不能运动。 |
| L355--361 | 没有合法里程计 | 零命令、WAITING_FOR_ODOMETRY；detail 优先给具体 frame 错误。 |
| L363--369 | 里程计过期 | clearPath 使旧速度剖面失效，零命令、ODOMETRY_TIMEOUT。 |
| L371--377 | 没有缓存路径 | 零命令、WAITING_FOR_PATH。 |
| L379--388 | 启用了 plan_timeout 且路径保活过期 | 清轨迹、零命令、WAITING_FOR_PATH。默认 0 不进入这里。 |
| L390--396 | emergency_stop_ 已锁存 | 零命令、EMERGENCY_STOP，不再尝试 MPC。 |
| L398--402 | goal_reached_ 已锁存 | 零命令、GOAL_REACHED。 |
| L404--410 | rejectPath 锁存 | 零命令、PATH_REJECTED。 |
| L412--418 | 没有活动生成器 | 再尝试激活缓存路径；失败就拒绝并 return。 |
| L420--425 | 可运行 | 构造固定 yaw 的 HeadingProvider，然后请求 N 个未来参考点。 |
| L426 | 复制生成器诊断 | 状态话题和终点判定使用最新的 progress/停车距离。 |
| L428--440 | EmergencyInfeasible | 锁存 emergency_stop_，绕过 MPC，直接发零；外部安全层需要据此采取急停。 |
| L442--450 | 轨迹库无激活路径或 N 点不足 | 零命令、PATH_REJECTED；不能拿不完整窗口求 MPC。 |
| L452--461 | 剩余弧长与实测速度都进阈值 | 锁存目标完成，清路径，零命令。 |
| L463--477 | 逐点适配给 MPC | 世界位置原样传；世界速度旋转到固定参考 yaw 下的车体系；角速度目前为 0。 |
| L479--481 | 根据轨迹状态设置 MPC 加速度限制 | 紧急制动参考配紧急增量限值，防止 MPC 自己仍只允许正常制动。 |
| L483--489 | 求解失败 | 零命令、MPC_FAILURE。 |
| L491--495 | 求解成功 | 直接把 command.vx/vy/vw 写进 Twist 并发布。 |
| L497--507 | 正常或紧急状态发布 | EmergencyBraking 有特殊状态和 warn；否则 TRACKING。 |

HeadingProvider 的 lambda（L420--422）忽略时间和弧长，始终返回有效的 locked_yaw_、零角速度、零角加速度。库因此不会自行派生 yaw；它只是把这个外部 yaw 值透传到每个 ReferencePoint。

### 4.7 publishZero、publishStatus 与 main

publishZero（L510--513）发布默认构造的 Twist。ROS 消息默认的 linear.x、linear.y、angular.z 都是 0，所以它是一条显式零速度命令。

publishStatus（L515--529）把内部诊断转换为 TrackingStatus：

- header.stamp 是本节点当前时间；
- header.frame_id 固定为 path_frame_；
- has_path 是“有通过 build 的 cached_path_”，不是“正在输出运动命令”；
- progress、remaining_distance、stop_deficit、terminal_speed 来自轨迹库最近一次 diagnostics_；
- detail 是分支提供的短文本，适合日志或上层显示，不能用它代替 state 常量做程序判断。

main（L586--593）依次初始化 ROS、创建节点、spin 让回调和 timer 工作，最终 shutdown。它不包含任何控制逻辑。

## 5. 平动库 hpp：数据模型和每个字段

### 5.1 命名空间与基本量（translational_trajectory.hpp L1--42）

L1 的 pragma once 防止同一个头文件被重复包含。L3--6 只包含大小类型、函数对象、字符串和 vector；这是纯 C++ 库，不包含 ROS、Eigen 或 MPC 类型。

| 类型/字段 | 含义 |
| --- | --- |
| Point2.x、y | 一个世界系位置，单位 m。 |
| Vector2.x、y | 一个世界系二维向量；可表示速度、切线、加速度。单位由上下文决定。 |
| MotionState2D.position | 当前底盘世界系位置。 |
| MotionState2D.velocity | 当前底盘世界系速度；不是机体系速度。 |
| HeadingReference.valid | false 时调用方可忽略其他 yaw 字段。 |
| HeadingReference.yaw | 外部规划提供的参考车身朝向，rad。 |
| angular_velocity | 外部 yaw 参考的一阶导，rad/s。 |
| angular_acceleration | 外部 yaw 参考的二阶导，rad/s2；目前 tracing_node 不交给 MPC。 |
| HeadingProvider | 函数类型：输入“从现在起的时间 t”和“路径弧长 s”，输出 HeadingReference。 |

HeadingProvider 的存在是为了让平动与转向解耦。当前节点传入固定 yaw；以后可接入独立 yaw 规划器，而无需改变几何、速度包络和 MPC 平动参考生成。

### 5.2 配置结构（L44--76）

PathBuildOptions 控制“点列如何变为几何路径”：

| 字段 | 作用 |
| --- | --- |
| sample_spacing | 第一次与第二次等弧长重采样的目标间距。间距越小，曲率和速度包络越精细，计算量越大。 |
| rdp_epsilon | Ramer--Douglas--Peucker 抽稀的偏差容许值。0 表示仅删除严格共线的中间点。 |
| smooth_half_window | 坐标移动平均的半窗口物理长度；转换为整数采样点数后使用。 |
| max_smooth_deviation | 平滑结果与输入原始折线的最大双向距离；超过即逐步减小窗口，仍失败则整条路径拒绝。 |
| max_smoothing_attempts | 上述自适应缩窗的最多次数，至少为 1。 |

MotionLimits 是安全相关物理值，默认 -1 代表“未配置”，不能直接用：

| 字段 | 控制哪种约束 |
| --- | --- |
| cruise_speed | 无曲率和制动约束时的期望最高速度。 |
| max_accel | 向前构造速度时的硬加速上限。 |
| normal_decel | 默认制动包络和正常停车能力。 |
| emergency_decel | 正常停车不够时允许使用的更强制动；必须不小于 normal_decel。 |
| max_lateral_accel | 通过 v <= sqrt(a_lat / abs(kappa)) 限制弯道速度。 |
| terminal_speed | 路径末端目标速度，默认 0。 |
| accel_fraction | 正常正弦加速段希望占剩余距离的比例；不是可以突破加速度约束的强制比例。 |
| decel_fraction | 正常正弦减速段希望占剩余距离的比例。 |
| isValid | 检查上面值的有限性、正负和相互关系，失败可写入 error。 |

GeneratorOptions 控制“每个控制周期怎么从当前状态重算”：

| 字段 | 作用 |
| --- | --- |
| max_activation_offset | 首次接入路径允许的横向距离。超过代表不能安全无缝切换。 |
| local_projection_backtrack | 已经有 progress_ 后，最近点投影允许向后查找的距离。 |
| local_projection_lookahead | 已经有 progress_ 后，优先向前查找的距离。 |
| profile_spacing | 速度包络的基础弧长网格；还会额外插入所有几何曲率结点。 |
| minimum_speed_for_time | profile 的时间积分下限，防止速度和为零时除零。 |
| max_reference_lead | 名义速度相位最多允许领先实测投影的弧长；默认 0.10 m，达到后暂停相位推进。 |

### 5.3 路径查询结果（L78--90）

PathSample 是 PathGeometry::sample(s) 的结果：

- position 是 s 米处的平滑几何位置；
- tangent 是沿“起点到终点”的单位切线；
- arc_length 是经过夹紧后的实际 s；
- curvature 是有符号曲率，左弯为正、右弯为负；速度上限只使用其绝对值。

Projection 是 PathGeometry::project(position) 的结果：

- arc_length 是最近投影点的路径进度；
- distance 是车辆到这个投影点的欧氏距离，而不是沿路径距离。

### 5.4 PathGeometry 类与其私有缓存（L92--135）

| 成员/函数 | 作用 |
| --- | --- |
| build | 完整清空旧表，校验点列，去重，RDP，重采样，平滑，偏差校验，最后填几何查询表。 |
| valid | 构建是否成功。只有 true 时才能 sample/project。 |
| size | 最终平滑等弧长点数，不是原始 planner 点数。 |
| length | 最终平滑路径总弧长。 |
| maxSmoothDeviation | 最终接受版本相对输入折线的实际双向最大偏差。 |
| sampleArcLengths | 返回内部曲率采样结点弧长。只读引用，给速度包络确保不会漏掉曲率峰。 |
| sample | 对任意 s 线性插值位置、切线、曲率。 |
| project | 求位置最近的路径点；有 hint 时优先局部搜索。 |
| valid_ | 只有 build 完整成功最后才置 true，避免半成品可用。 |
| total_length_ | arc_lengths_.back()，总长度。 |
| max_smooth_deviation_ | 最后一次接受的对称偏差。 |
| arc_lengths_ | 每个 points_ 采样点从起点累计的 s。 |
| points_ | 平滑且再等弧长采样后的几何位置表。 |
| tangents_ | 与 points_ 一一对应的单位切线表。 |
| curvatures_ | 与 points_ 一一对应的有符号曲率表。 |
| projectRange | 只在 [s_lo,s_hi] 内投影的实现细节；public project 选择局部或全局范围后调用它。 |

### 5.5 轨迹状态、诊断和输出点（L137--168）

TrajectoryStatus 不等同于 ROS TrackingStatus。前者只描述平动库的速度可行性，后者还描述开关、里程计、MPC 等 ROS 生命周期。

| 库状态 | 含义 | tracing_node 映射 |
| --- | --- | --- |
| NoActivePath | 没有激活路径、参数错误或生成失败。 | PATH_REJECTED/等待类状态。 |
| Ready | 正常加减速、曲率和终点约束同时可行。 | TRACKING。 |
| PathRejected | activatePath 输入非法或接入距离过远。 | PATH_REJECTED。 |
| EmergencyBraking | 正常减速度不够，紧急减速度可行。 | EMERGENCY_BRAKING，MPC 同步放宽增量限值。 |
| EmergencyInfeasible | 紧急减速度仍不能满足末端/弯道速度。 | EMERGENCY_STOP，节点直接发零。 |

TrajectoryDiagnostics 各字段：

| 字段 | 含义 |
| --- | --- |
| status | 上一次 rebuildProfile 得出的库状态。 |
| progress | 单调保存的当前路径弧长位置。 |
| remaining_length | 从 progress 到路径末端的弧长。 |
| normal_stop_distance | 从当前切向速度通过正弦减速到 terminal_speed 的最小距离。 |
| emergency_stop_distance | 同上，但用 emergency_decel。 |
| stop_deficit | 正常停车距离超出剩余距离的缺口，或当前速度高于正常后向包络带来的等价缺口。 |
| terminal_speed_if_unstoppable | EmergencyInfeasible 时仍会带到路径末端的估计速度。 |

ReferencePoint 是每个 MPC 未来时刻的一点：

| 字段 | 含义 |
| --- | --- |
| time_from_now | 该点距当前控制周期的时间。第一个点是 dt，不是 0。 |
| arc_length | 该时刻到达的路径进度 s。 |
| position | 世界系 x/y。 |
| velocity | 世界系速度，等于 tangent * speed。 |
| acceleration | 世界系加速度，包含切向与法向两项。 |
| speed | 非负路径标量速度。 |
| tangential_acceleration | dv/dt。 |
| curvature | 查询位置的有符号曲率。 |
| heading | HeadingProvider 的透传结果。 |

### 5.6 TrajectoryGenerator 类、嵌套结构和成员（L170--240）

TrajectoryGenerator 是“已接受的一条 PathGeometry 加当前测量状态”到“未来时间窗口”的状态机。它并不保存 ROS 消息，也不保存上一次 MPC 命令。路径激活时保存名义速度剖面和相位；每次 makeHorizon 只从最新实测状态重建安全包络。

公开函数的职责：

| 函数 | 输入 | 改变的内部状态 | 输出/失败方式 |
| --- | --- | --- | --- |
| 构造函数 | MotionLimits、GeneratorOptions | 复制配置。 | 不验证；调用方应先 setLimits 或在节点构造时检查。 |
| setLimits | 新 MotionLimits | 验证成功后替换 limits_。 | false 加 error 表示不接受。 |
| limits | 无 | 不改变。 | const 引用，仅供查看。 |
| activatePath | 已建好的 PathGeometry、当前世界状态 | 复制路径，投影并初始化 progress_，建立并保存新的名义 profile，相位置零。 | false 表示路径/状态/接入距离无效。 |
| clearPath | 无 | 清 active_、进度、安全/名义剖面、精确正弦形状、相位和诊断。 | 无返回。 |
| hasActivePath | 无 | 不改变。 | active_ 的值。 |
| path | 无 | 不改变。 | 当前路径的 const 引用。 |
| diagnostics | 无 | 不改变。 | 最近一次诊断的 const 引用。 |
| makeHorizon | 最新状态、dt、步数、输出 vector、可选 yaw 函数 | 重建安全 profile_、推进受超前量限制的名义相位，并推进 progress_。 | 返回库状态，out 得到最多 steps 个未来 ReferencePoint。 |

私有 ProfileNode 是受约束速度包络的离散节点：

| 字段 | 意义 |
| --- | --- |
| time | 从当前时刻到达该弧长节点的累计时间。 |
| arc_length | 此节点的路径弧长。 |
| speed | 此节点允许的路径标量速度。 |
| tangential_acceleration | 到下一个节点的近似常加速度；末点被改为 0。 |

私有 ExactSinusoid 保存“完全没有被曲率或硬约束削掉的名义三段式”：

| 字段 | 意义 |
| --- | --- |
| active | true 表示 sampleProfile 不使用离散 profile_ 的分段常加速度，而按解析正弦计算。 |
| direct_ramp | 车辆初速超过 cruise_speed 时，只有一段从初速直接到终速的正弦减速。 |
| start_arc_length | 这一轮重建时的 progress_，后续相对距离要加回它。 |
| length | 当前剩余路径长度。 |
| start_speed / peak_speed / end_speed | 三段式的起、峰、终标量速度。 |
| accel_distance / cruise_distance / decel_distance | 三段分别占用的弧长。 |

生成器自身成员：

| 成员 | 生命周期内的含义 |
| --- | --- |
| limits_ | 当前使用的物理约束副本。 |
| options_ | 当前使用的在线生成参数副本。 |
| path_ | activatePath 成功后复制的几何路径；与节点的 cached_path_ 是不同对象。 |
| active_ | path_ 是否已被安全激活。 |
| progress_ | 单调不减的路径进度；避免自交路径或定位噪声导致回跳。 |
| diagnostics_ | 最近 activate 或 rebuild 的状态与制动指标。 |
| profile_ / exact_nominal_ | 每周期从实测状态重建的安全 time--s--v 表及其临时解析形式。 |
| reference_profile_ / reference_exact_nominal_ | 激活路径时保存的完整名义速度剖面；不会在普通控制周期中重置。 |
| reference_time_ | 跨 makeHorizon 调用持续推进的名义速度相位。 |

## 6. translational_trajectory.cpp：从基础数学到几何路径

### 6.1 文件开头和匿名命名空间（L1--16）

L1 包含自己的声明；L3--5 引入算法、数学和数值无穷大。L7--10 进入 robot_control::trajectory。L11 开始匿名命名空间，其中的辅助函数只在这个 cpp 文件可见，不会成为公共 API。

- kEps（L14）是 1e-9 的数值“接近零”门槛。它避免长度几乎为零时归一化或除法爆炸。
- kPi（L15）是显式 pi 常量，用于正弦速度曲线。没有依赖 M_PI，因而跨编译器一致。

### 6.2 最基础的二维工具函数（L17--107）

这些函数没有保存任何状态，都是纯函数；它们把后续的几何代码写得更清楚。

| 行 | 函数 | 逐步做什么 |
| --- | --- | --- |
| L17--20 | finite | 调用 std::isfinite，拒绝 NaN 和正负无穷。 |
| L22--25 | finitePoint | 分别检查 Point2 的 x/y，只有两者有限才返回 true。 |
| L27--30 | clamp | 先取 value 与 hi 的较小值，再与 lo 取较大值，得到闭区间裁剪。 |
| L32--35 | subtract | 返回 b 到 a 的向量 a-b。 |
| L37--40 | scale | 对二维向量的两个分量同乘一个标量。 |
| L42--45 | add | 两个二维向量逐分量相加。 |
| L47--50 | dot | a.x*b.x+a.y*b.y，供投影和长度平方使用。 |
| L52--55 | cross | 二维叉积标量 a.x*b.y-a.y*b.x，正负表示左/右转。 |
| L57--60 | norm | hypot(x,y)，比 sqrt(x*x+y*y) 更稳健地求模长。 |
| L62--69 | normalized | 模长小于 kEps 时返回默认 +x 切线，其他情况逐分量除模长。默认值避免除零，但也意味着零长度段不能携带真实方向。 |
| L71--74 | lerp | a+(b-a)*t 的位置线性插值。调用者负责先把 t 限在 0 到 1。 |

distanceSquaredToSegment（L76--96）是几何部分最常用的函数，具体过程：

1. L82 得到线段方向 ab，L83 得到端点 a 指向待测点的 ap。
2. L84 用点积得到线段长度平方，避免先开方。
3. L85--88：若线段有效，计算 ap 在 ab 上的比例 t，然后夹紧到 [0,1]；退化段则 t 保持 0，即最近点就是 a。
4. L89--91：若调用者给了 projection 指针，把 t 回传。
5. L92 求最近位置 closest = a+t(b-a)。
6. L93--95 返回点到 closest 的距离平方。这里不 sqrt，便于 RDP 和最小值比较。

pointToPolylineDistanceSquared（L98--107）逐段调用上述函数并取最小距离平方。它假设 polyline 至少有两点；build 只有在满足此前提后才会调用它。

### 6.3 RDP 抽稀（L109--135）

rdp 是 Ramer--Douglas--Peucker 递归实现。输入 first、last 是当前要判断的点列端点索引，output 由调用方传入。

1. L116--124 遍历中间点，计算每个点到首尾线段的距离平方，记录最远距离和最远点索引。
2. L126 判断最远点是否超过 epsilon。比较 epsilon*epsilon 是为了避免多余开方。
3. 若超过容差，L127 对前半段递归，L128 pop_back 删除两段连接处重复的最远点，再 L129 对后半段递归。
4. 若全部中间点都足够接近首尾直线，L133--134 只保留首尾点。

它会保留整条路径的起点与终点，因为任何递归的基本情形都会 push 当前 first 和 last。RDP 处理的是去除连续重复点后的 input，而不是原始 vector。

### 6.4 等弧长重采样（L137--174）

resamplePolyline 把不均匀 planner 点距变为近乎均匀的弧长点距；它不会创建高阶曲线，所有插值都仍在折线段上。

| 行 | 操作 |
| --- | --- |
| L142--145 | 清空旧输出；少于两点或 spacing 太小立即失败。 |
| L147--150 | cumulative[i] 保存从起点到第 i 点的累积折线距离。 |
| L151--154 | 总长非正表示所有点重合，失败。 |
| L156--158 | steps=ceil(total/spacing)，至少 1；reserve 避免循环扩容。 |
| L159 | segment 指向当前包含目标弧长的输入折线段。 |
| L160--170 | 对每个均匀目标弧长：确定最后一个目标用 total 以精确落终点；while 推进 segment；算本段 ratio；线性插值得到输出点。 |
| L171--172 | 强制首末点等于原始首末点，消除浮点插值误差，保证端点不会漂移。 |

这里输出点之间的间距是 total/steps，而不是严格等于 spacing；这样能保证最后一小段不会特别短，同时仍接近指定间距。

### 6.5 平滑与偏差检查（L176--222）

smoothPolyline 使用逐坐标的对称移动平均。

L182 复制 original，保证所有新点都从同一轮旧数据计算，而不是前一个已平滑点影响后一个。L184--196 的 sample lambda 是关键：

- index 在有效范围内，直接返回 original[index]；
- index<0，用前两个点做线性外推；
- index>=size，用后两个点做线性外推。

边缘外推而非简单复制端点的原因是：简单复制端点会让直线路径端部被平均向内拉，缩短路径。即使如此，L529--530 还会把最终端点再次强制锚定。

L198--207 的双循环表示：对每个输出点 i，累加从 i-half_window 到 i+half_window 的样本，除以 2*half_window+1。half_window<=0 时 L178--180 直接不改点。

symmetricPolylineDeviation（L210--222）不只检查“原始点离平滑线多远”，还反向检查“平滑点离原始线多远”：

1. L215--217 取每个 original 点到 smoothed 折线的最远距离平方；
2. L218--220 取每个 smoothed 点到 original 折线的最远距离平方；
3. L221 开方得到米单位最大值。

双向检查避免平滑曲线在两个原始采样点之间大幅偏出却没被原始点采样到。这个值也是规划器障碍物膨胀时必须预留的最大几何偏差。

### 6.6 建立弧长、切线和曲率表（L224--259）

buildGeometryTable 的五个输出 vector 全部与 points 一一对应。

1. L231--233 先按正确长度分配，默认切线为 +x、曲率为 0。
2. L234--237 对相邻平滑点累加距离，最终最后一个值写入 total_length。
3. L239--243 对每个点取“前一点到后一点”的中心差分并归一化。端点的 before/after 被夹到自身，所以首点实际取 p1-p0，末点取 pn-p(n-1)。
4. 少于三点没有内点能估计曲率，L245--247 直接返回且曲率为 0。
5. L248--255 用相邻三点的三角形公式计算有符号曲率：

       kappa = 2 * cross(a,b) / (|a|*|b|*|c|)

   其中 a=p_i-p_(i-1)，b=p_(i+1)-p_i，c=p_(i+1)-p_(i-1)。分母太小的退化三角形保持 0。
6. L257--258 端点没有中心三点公式，因此复制相邻内点曲率。

切线与曲率是离散估计，不是全局样条的解析导数；因此 sample_spacing 和平滑窗口共同决定它们的噪声程度。

## 7. 速度规划采用的数学工具

### 7.1 minimumSinusoidalDistance（L261--267）

从速度 v0 平滑正弦变化到 v1，限制最大切向加速度为 a 时，最短需要的路径距离为：

    d_min = pi * abs(v1^2 - v0^2) / (4*a)

L263--265 对非正 a 返回无穷大，表示不可行；L266 按该公式计算。它同时用于加速、正常制动、紧急制动的提前可行性判断。

### 7.2 inverseSinusoidalPhase（L269--305）

正弦速度定义很容易按时间 t 求速度，但 NominalSpeedShape 在“已知走了多少距离 s”时要反过来求 t。这个函数解决的方程是：

    s(t) = mean*t - delta*T/(2*pi) * sin(pi*t/T)

其中 mean=(v0+v1)/2，delta=v1-v0，T 是这一段总时长。

| 行 | 意义 |
| --- | --- |
| L273--280 | 时间段为零或平均速度为零没有可逆行程，返回 0。 |
| L282--284 | 建立 [0,T] 二分区间，用 distance/mean 作为初值。 |
| L285--304 | 最多 60 次 Newton--bisection 混合法：算残差，按残差更新二分边界；导数可靠时尝试 Newton 步，否则取中点；Newton 步跑出区间也回退到中点；足够收敛就返回。 |

这样既享受 Newton 的快速收敛，也避免纯 Newton 在某些数值条件下越界。

### 7.3 NominalSpeedShape（L307--411）

这个局部结构只表示“用户想要的、尚未叠加弯道和终点硬约束”的速度形状。它不是最终可执行速度，最终还要经过后向制动包络和前向加速可达性。

字段含义：

| 字段 | 作用 |
| --- | --- |
| valid | build 成功后才允许 speedAt 使用。 |
| direct_ramp | 初始速度已高于 cruise_speed 时，跳过加速/巡航，只从初速平滑减到终速。 |
| length | 当前从 progress 到终点的剩余长度。 |
| start_speed、peak_speed、end_speed | 正弦三段式端点与峰值速度。 |
| accel_distance、cruise_distance、decel_distance | 三段的弧长分配。 |
| accel_limit、decel_limit | build 时拷贝的正常硬加/减速度。 |

build（L321--376）逐步如下：

1. L323--327 记下长度、把负速度夹到 0、从 limits 复制约束；无长度直接失败。
2. L335--344 处理“已超巡航”：不要求先瞬间降到 cruise，而是若正常正弦制动能在剩余距离完成，就建立全长 direct_ramp；否则返回 false，之后由硬约束和紧急状态处理。
3. L346--353 推导可达到的最大峰值。c_accel 和 c_decel 把正弦距离公式写成关于峰值速度平方的系数。maximum_peak 是刚好没有巡航段的三角形速度曲线峰值；实际峰值再受 cruise_speed 限制，并不得低于起/终速度。
4. L355--359 检查从起速到峰值、峰值到终速的硬最小距离之和。仍大于总长说明连三角形曲线都不能用，返回 false。
5. L361--373 先按用户比例分配加/减速距离，但每段至少为各自硬最小距离；两者相加过长时，只按各自“可压缩余量”比例回收 excess，绝不压到硬最小距离以下。
6. L373--375 剩余部分为 cruise_distance，置 valid 成功。

rampSpeed（L378--390）把给定“该段已走 distance 中的 progress”转成速度：

1. 距离或速度和接近零时直接给终速度，避免不合理时长；
2. 由平均速度得到总时间 duration；
3. 用 inverseSinusoidalPhase 找当前时间；
4. 返回 v(t)=from+0.5*(to-from)*(1-cos(pi*t/T))。

speedAt（L392--410）把路径进度分到四种情况：无效返回无穷大（作为 min cap 时不产生限制）、direct_ramp、加速段、匀速段、减速段。

### 7.4 backwardEnvelope（L413--432）

后向速度包络回答的问题是：“若在每个位置的速度都不超过 caps[i]，且终点必须是 terminal_speed，从第 i 点最早需要把速度压到多少？”

1. L419 创建与 caps 同长度的 result；空输入返回空/零表。
2. L423 末点速度是末点局部 cap 与 terminal_speed 的较小者。
3. L424--430 从后往前，每段使用匀减速可达关系：

       v_i <= sqrt(v_(i+1)^2 + 2 * decel * ds)

   再与当前 local cap 取 min。

它实现“提前制动”：弯道局部低速点或终点停车要求会经由这个递推传回到前面的直线段。

## 8. PathGeometry 的全部公开函数

### 8.1 MotionLimits::isValid（cpp L436--463）

L438--440 先把全部数值做 finite 检查；L441--443 检查速度、加速度、比例的符号，terminal_speed 可以为 0 但不能负。失败时若 error 非空就填错误文本并返回 false。

L450--455 单独要求 emergency_decel 不小于 normal_decel。否则“紧急”反而更弱，状态名没有物理意义。

L456--461 要求 terminal_speed 不超过 cruise_speed。否则普通速度上限和终点要求自相矛盾。

### 8.2 PathGeometry::build（L465--562）

build 是一条路径从原始点到可查询几何表的完整事务：任何失败都会保持 valid_=false，不会留下部分有效路径。

| 行 | 每一步的作用 |
| --- | --- |
| L470--476 | 复位 valid、长度、偏差，并清空四张旧表。这样对象可被重复 build。 |
| L478--486 | 校验 PathBuildOptions。sample_spacing 必须正；RDP/平滑/偏差不能负；尝试次数至少 1。 |
| L488--500 | 遍历 raw_points：先拒绝 NaN/Inf；连续重复或距离 <= kEps 的点不压入 input。保留第一次和最后一个不同点。 |
| L501--506 | 去重后不足两点代表没有可走路径，失败。 |
| L508--515 | 对 input 进行 RDP。理论上 RDP 会保首尾；此处仍检查防御性失败。 |
| L517 | half_window 从配置值开始，后续失败会不断减半。 |
| L518--556 | 最多多次尝试建立一个“既平滑又不偏离”的路径。 |
| L519--522 | 先将 RDP 后折线等弧长重采样为 dense，失败跳出尝试循环。 |
| L523--524 | 保存重采样后的真实首尾，供平滑后重新锚定。 |
| L525 | 把米单位 half_window 转为整数采样点个数，lround 使最近整数窗口。 |
| L526 | 对 dense 做一次移动平均。 |
| L527--530 | 无论外推平均结果怎样，强制端点回到规划器端点。 |
| L532--535 | 再等弧长采样一次，消除平滑后点间距不均。 |
| L536--537 | 与最初 input 做双向偏差；在允许范围内才接受。 |
| L538--553 | 对接受的 uniform 构造弧长、切线、曲率表；长度有效后 move 到成员，写总长度、偏差、valid=true 并成功返回。 |
| L555 | 本轮太偏，half_window 减半，下轮平滑更弱。 |
| L558--561 | 所有尝试均失败，给出明确“平滑偏差超限”错误。 |

原始点序列可能是自交的。build 会接受自交路径，因为几何上它仍可定义；但投影在交叉附近可能有多个相同距离的候选，后续依赖 progress_ 局部搜索和单调约束来降低跳支风险。

### 8.3 PathGeometry::sample（L564--590）

sample(s) 的返回值始终把 s 夹到 [0,total_length]。

1. L566--569：路径无效或点表空，返回默认 PathSample（原点、+x、零曲率）。
2. L570：夹紧弧长。
3. L571--579：upper_bound 找第一个大于 s 的结点，转换为左侧 segment 索引；恰好在终点时明确使用倒数第二个段。
4. L580--582：算该段的局部比例并线性插值位置。
5. L583--586：相邻结点切线线性混合后重新归一化，避免混合后模长小于 1。
6. L587--588：写回实际 s，线性插值曲率。

### 8.4 projectRange 与 project（L592--654）

projectRange(position,s_lo,s_hi) 在指定弧长窗口找最近点：

1. L594--598 先把距离设为无穷，路径无效则立即返回“无结果”。
2. L599--603 把范围夹进路径，并容忍调用方把上下界传反。
3. L605--614 用 upper_bound 将 s 范围转换为 points_ 段索引，并夹到合法段数；保证至少检查一个段。
4. L616--631 遍历这些段：先算整段最近投影比例，再把其 s 夹回窗口，调用 sample 取得真正受限位置，比较距离，保留最小者。
5. L626 的 (void)d2 说明首次算的距离仅为了得到 ratio；受 s 窗口裁剪后必须重新用 candidate 位置计算 restricted_d2。

project(position,hint,backtrack,lookahead) 是 public 策略：

- hint<0（L644--646）：初次激活没有进度，搜索整条路径；
- 有 hint（L647--649）：优先只搜 [hint-backtrack,hint+lookahead]；
- 局部最近点超过 0.5 m（L650--652）：认为局部结果不可信，退回全路径搜索；
- 否则返回局部结果。

0.5 m 是当前库内固定的“局部投影不可信”阈值，和 GeneratorOptions::max_activation_offset 是不同概念：前者决定搜索范围是否回退，后者决定一条新路径是否可接入。

## 9. TrajectoryGenerator：持续名义相位、实时安全包络与状态

### 9.1 构造、setLimits、activatePath、clearPath（L656--717）

构造函数（L656--661）只把传入 limits 与 options 拷贝到成员。它故意不在这里抛异常，因此纯库的使用者可以先构造、后 setLimits；tracing_node 则已在构造前检查过一次。

setLimits（L663--670）的次序很重要：

1. L665 校验传入对象，而非先覆盖旧 limits_；
2. L666 失败时旧限制仍保留；
3. L668 成功后才替换。

activatePath（L672--708）定义“缓存一条路径”与“安全接入一条路径”的边界。

| 行 | 每句话的含义 |
| --- | --- |
| L677--680 | 当前物理限制无效，不能激活；诊断标成 PathRejected。 |
| L681--689 | 需要有效 geometry，当前位置有限，速度两分量也有限。任一失败不接受。 |
| L690 | 对当前车辆位置在新 geometry 上做全局最近点投影。 |
| L691--697 | 横向距离超过 max_activation_offset 时拒绝。它不试图“沿旧路径追过去”，因为调用方需要决定安全处置。 |
| L699 | 复制 geometry 到 path_。之后规划器重用/释放原候选对象不会影响生成器。 |
| L700 | active_=true，makeHorizon 才会工作。 |
| L701 | progress_ 从当前投影弧长开始，而非新路径第一个点。 |
| L702 附近 | 清除旧安全/名义 profile 和旧相位，避免换路径后继承上一个任务。 |
| 后续构建 | 立即按激活时的实测状态建立完整名义剖面，保存到 reference_profile_，并把 reference_time_ 置零。 |

clearPath（L710--717）是“回到没有任务”的强复位：

- active_=false 阻止 makeHorizon；
- progress_=0 防止下条路径继承旧进度；
- profile_ 清空；
- exact_nominal_ 置默认值，关闭解析正弦模式；
- reference_profile_、reference_exact_nominal_ 和 reference_time_ 一并复位；
- diagnostics_ 置默认，status 也回到 NoActivePath。

### 9.2 rebuildProfile 的入口和实时安全状态锚定

rebuildProfile 不断接收最新实测状态。激活时传入 `apply_nominal_shape=true`，构建并保存完整正弦名义剖面；普通控制周期传入 false，只建立巡航、曲率、加速可达性和制动包络，不再生成一个新的正弦起步阶段。这使安全判断继续闭环使用实测状态，同时避免速度相位每拍回零。

| 行 | 每句话的含义 |
| --- | --- |
| 函数入口 | 清旧的临时安全表和临时解析缓存；保存的 reference_profile_ 不受影响。 |
| L723--728 | active_、路径、位置、速度任一不合法，诊断标 NoActivePath 并失败。 |
| 配置检查 | profile_spacing、最小时间速度或 max_reference_lead 非法时不能生成。 |
| L734--737 | 用旧 progress_ 作为 hint，只在有限回看/前看范围投影当前车辆位置。 |
| L738 | progress_=max(旧进度, 新投影)。即使定位噪声把投影拉回去，也不会倒退路径。 |
| L739 | 剩余长度夹到非负。 |
| L740 | 查询 progress 处的 PathSample，取得当前位置切线。 |
| L741 | 初速度是世界系实测速度沿单位切线的点积；反向速度会被夹为 0，因为第一版只支持正向走路径。 |
| L743--750 | 重新填诊断：进度、剩余距离、正常/紧急正弦停车距离，以及正常停车距离比剩余距离多出的缺口。 |

如果 remaining<=kEps（L752--759），路径已经走到终点：

- 若 initial_speed 已不大于 terminal_speed，L753--756 报 Ready，末端残速诊断为 0；
- 否则已经没有距离可减速，报 EmergencyInfeasible，terminal_speed_if_unstoppable 直接写当前切向速度；
- L757 仍放入一个单节点 profile，以便调用者能看到末端的确定性参考，而不会因空表崩溃。

注意：tracing_node 在调用 makeHorizon 后再用“remaining 与实测总平动速度”判定 GOAL_REACHED。因此若到末端时仍有速度，这里会先进入 EmergencyInfeasible，节点会进入 EMERGENCY_STOP，而不是把它伪装成到达。

### 9.3 速度包络网格与局部速度上限（L761--815）

这一段先建立“在哪些弧长位置必须检查速度”的网格，再给每个结点建立上限 caps。

| 行 | 每句话的含义 |
| --- | --- |
| L761--764 | segments=ceil(remaining/profile_spacing)，至少 1；创建 arc_lengths。 |
| L765 | reserve 包含基础网格、所有几何采样结点和少量余量，减少 realloc。 |
| L766--770 | 均匀铺 [progress_, path.length]。i=0 是当前进度，i=segments 是终点。 |
| L771--777 | 把 PathGeometry 自己的每个内部弧长结点加进网格。否则曲率在线性插值中若恰好在两基础结点之间有峰值，速度 cap 可能被漏掉。 |
| L778--782 | 排序并把相差小于 1e-10 的重复 s 合并。 |
| L783--784 | 记录结点数，并用 cruise_speed 初始化每点 caps。 |
| L785--789 | 以当前 remaining、initial_speed、limits 构建名义正弦三段式。lambda 只为让 const 变量初始化更整洁。build 失败不代表整个轨迹立即失败；硬包络仍可能用紧急状态处理。 |
| L791--801 | 对每点叠加两类 cap：弯道横向加速度 cap 与名义正弦速度；每次均取 min，最后夹非负。 |

弯道 cap 的公式在 L794--796：

    a_normal = abs(kappa) * v^2 <= max_lateral_accel
    v_curve = sqrt(max_lateral_accel / abs(kappa))

曲率接近零时不做除法，仍保持 cruise/名义 cap。

L802--814 是额外的保守保护。曲率在两个节点间线性插值，若某一段一端曲率高，另一端低，仅限制高的一端可能使时间插值时出现不安全速度。因此对每个相邻 profile 段取两端最大曲率，算 segment_cap，再同时压低两端 caps。代价是小范围早一点慢下来，收益是内部插值也不超过横向加速度限制。

L815 把最后一点 cap 再压到 terminal_speed。这一行确保路径终点约束也是速度包络的局部上限，而不是仅靠诊断文字表示。

### 9.4 正常制动、紧急制动与不可行判定（L817--859）

这一段决定何时从 Ready 进入 EmergencyBraking 或 EmergencyInfeasible。

1. L817--818 用 normal_decel 建 normal_back。它是“从每个节点仍可在终点刹住”的正常后向包络。
2. L819--820 的 normal_infeasible 有两种触发：

       normal_stop_distance > remaining

   表示从初始切向速度平滑刹到终速连总距离都不够；

       initial_speed > normal_back.front()

   表示即使总停车距离看似够，首段上还有曲率/局部 cap，当前速度也无法按正常减速度降到下一段要求。

3. L821 选择后面前向传播时的实际减速度：正常可行用 normal_decel，否则用 emergency_decel。
4. L822--824 正常不够时重算 emergency 后向包络。
5. L826--830 若紧急停车距离仍超剩余长度，或当前初速仍高于紧急后向包络起点，则标 EmergencyInfeasible；否则正常不够、紧急够时标 EmergencyBraking；两者都没发生是 Ready。
6. L831--838 为正常不够的情况增加一个与“速度超过正常包络”对应的等价距离缺口：

       (initial_speed^2 - normal_back.front()^2) / (2*normal_decel)

   stop_deficit 取它与原停车距离缺口的较大值。

这里不会因为 EmergencyInfeasible 就伪造一个能停住的速度表。后续前向传播会保留物理可达速度，并把末端残速写入 diagnostics。

### 9.5 前向可达性：生成唯一的最终速度表（L840--900）

在 caps 与后向制动包络都确定后，speed 表只生成一次。这样不会出现“先生成 v(t)，再把弯道点速度取 min”造成位置积分和速度不一致的问题。

| 行 | 每句话的含义 |
| --- | --- |
| L840--841 | 建 speeds，首点严格使用实测初始切向速度。不会突然把当前速度钳成 cap。 |
| L842--857 | 从前往后逐段计算下一点速度，确保既不违反加速可达性，也不低于物理制动下界。 |
| L843 | ds 是本段弧长。 |
| L844--847 | acceleration_reachable=sqrt(v_prev^2+2*max_accel*ds)，从前点在加速上限下最多能有多快。 |
| L848--850 | braking_floor=sqrt(v_prev^2-2*decel_used*ds)，从前点在选定最大制动下最慢能降到多慢。 |
| L850 | 先取 min(back[i], acceleration_reachable)，同时满足“以后能停/进弯”和“现在加速够不够”。 |
| L851--856 | 若该速度比 braking_floor 还低，说明甚至最大紧急制动也来不及达到后面请求的低速度。改成真实能达到的 braking_floor，并明确标 EmergencyInfeasible。 |
| L858--859 | 不可停时，记录最终仍会有的 speeds.back；否则写 0。 |

随后 L861--884 判断能否保留解析正弦形式：

- 仅 Ready 且 nominal.valid 才可能保留；
- L865--871 把每个最终 speeds[i] 与 nominal.speedAt 比较；
- 任何曲率、制动、加速度硬约束改变过名义形状，就用离散表；
- 全部相等时，L873--882 把名义形状逐字段复制到 exact_nominal_，以后 sampleProfile 可精确求正弦位置、速度、加速度。

最后 L886--900 将离散 speeds 积分成按时间查询的 profile_：

1. L887 写第一个节点：t=0、当前 s、当前 v、暂时 a=0；
2. 每段 L889--897 采用端点速度线性变化的平均速度积分：

       dt = 2*ds / (v_prev + v_next)

   速度和太小改用 minimum_speed_for_time 防除零；
3. 段切向加速度为 (v_next-v_prev)/dt，写进前一个节点，也写进新节点（在下一段覆盖前暂时相同）；
4. L899 强制最后节点的“后续加速度”为 0；
5. L900 返回 true。即使状态为 EmergencyInfeasible，函数仍成功生成诊断性、物理可达的表。

### 9.6 sampleProfile：按时间取出一个完整参考点（L903--1012）

sampleProfile 的输入 time_from_now 是从当前控制周期起算的未来秒数。它不会改变 progress_ 或 profile_。

开始部分 L907--915：

- L908--910：没有 profile 返回默认空点；
- L912--914：默认取 profile 第一节点的 s、v、a；
- 后面再按解析模式、末端或插值模式覆盖这三个量。

#### 解析正弦分支（L915--975）

L916--931 定义局部 phase lambda。给定一段总时长、起止速度与局部时间，它同时计算：

    distance(t) = mean*t - delta*T/(2*pi)*sin(pi*t/T)
    speed(t)    = from + 0.5*delta*(1-cos(pi*t/T))
    accel(t)    = 0.5*delta*pi/T*sin(pi*t/T)

因此平动参考的速度、加速度和位置积分来自同一个解析函数，彼此一致。

L933--946 从 ExactSinusoid 的距离和端点速度算三个段的时间：

- 加速/减速段用 2*d/(v0+v1)；
- 匀速段用 d/v_peak；
- direct_ramp 时总时间只按整段的起终速度算；
- max(kEps, ...) 防止零速度和除零。

L948--973 按 time_from_now 落在哪一段选择：

| 条件 | 输出 |
| --- | --- |
| 已超过总时长 | s=终点，v=end_speed，a=0。 |
| direct_ramp | 对整条剩余长度调用 phase。 |
| 加速段 | phase(start_speed 到 peak_speed)。 |
| 匀速段 | 距离=加速距离+peak_speed*剩余时间，a=0。 |
| 减速段 | phase(peak_speed 到 end_speed)，再加前两段距离。 |

L974 将相对距离加 start_arc_length，转换回整条路径绝对 s。

#### 离散包络分支（L975--994）

当 exact_nominal_.active 为 false 时：

- 时间超过 profile_.back().time：停在最后节点，切向加速度置零；
- time<=0：保留第一节点；
- 中间时间：L980--993 用 upper_bound 找前后两个 ProfileNode。

中间插值把本段视作常加速度：

    a = (v_b-v_a)/(t_b-t_a)
    v = v_a+a*local_time
    s = s_a+v_a*local_time+0.5*a*local_time^2

L993 再把 s 夹到此段两端，处理浮点误差。

#### 把标量结果变为二维运动学（L996--1011）

| 行 | 含义 |
| --- | --- |
| L996 | 对算出的 s 查询几何位置、切线、曲率。 |
| L997 | normal=(-tangent.y,tangent.x)，即切线逆时针旋转 90 度的单位法向。 |
| L998--1003 | 填时间、s、位置、标量速度、切向加速度、曲率。 |
| L1004 | 世界速度 = tangent * speed。 |
| L1005--1007 | 世界加速度 = tangent * a_t + normal * (kappa*v^2)。前一项是加减速，后一项是转弯法向加速度。 |
| L1008--1010 | 若调用方给了 heading_provider，就以当前 t、s 调用并原样写入 heading。 |

库虽算出了 acceleration，但当前 tracing_node 只使用位置、速度和 heading。加速度留给状态诊断和未来的 MPC 前馈扩展。

### 9.7 makeHorizon：持续相位和重新锚定的周期入口

makeHorizon 依次完成：

1. 清空输出并检查 active、dt、steps 和指针；
2. 按最新实测投影和切向速度重建安全包络，但不把名义正弦相位置零；
3. 将 reference_time_ 推进一个 dt；若名义弧长已比实测 progress_ 超前 max_reference_lead，则暂停推进；若车辆跑在名义相位前方，则将相位同步到实测进度；
4. 每个未来点从当前实测 progress_ 重新积分位置，而不是沿上周期累计的位置参考继续前跑；
5. 速度先读取持续名义相位，再同时受单步加减速可达区间和当前空间安全包络限制；
6. 输出时间仍是 dt、2dt、...，外部 heading_provider 看到的是相对当前周期的时间；
7. 返回 Ready、EmergencyBraking 或 EmergencyInfeasible，由调用方执行正常跟踪或安全处置。

这种拆分专门避免两种相反故障：每拍重启正弦会让静止车辆永远只收到极小首速度；直接让整条位置参考按绝对时间前跑又会形成永久位置误差。当前实现只让速度相位持续，位置每拍重新锚定到实测投影。

## 10. 把一次 20 Hz 周期连成可执行心智模型

假设 dt=0.05、N=20，路径已经激活。

    里程计给出：
      世界位置 (x,y)，车身 yaw，车体系速度 (vbx,vby)

    tracing_node：
      v_world = R(yaw) * [vbx,vby]
      调用 makeHorizon(v_world, 0.05, 20)

    平动库：
      1. 将当前位置投影到 path_，且 progress 不允许倒退；
      2. 取 v0 = dot(v_world, path_tangent)；
      3. 从激活时保存的名义曲线继续推进 reference_time；
      4. reference_time 的名义弧长最多领先实测 progress 0.10 m；
      5. 在当前 s 到终点重建不含“新正弦起步”的安全速度网格；
      6. 从终点向前传播曲率与制动能力，再从实测速度向前传播可达性；
      7. 从实测 progress 重新积分未来 20 点的位置；
      8. 每点速度取持续名义相位、单步加减速能力和空间安全包络共同允许的值；
      9. 输出 t=0.05,0.10,...,1.00 的 x/y/vx/vy/ax/ay。

    tracing_node：
      对每一点，以 locked_yaw 把 v_world 转到 MPC 所用参考车体系；
      x/y 保持世界系；
      yaw 始终是 locked_yaw，vw=0；
      将 20 点交给 MPC。

    MPC：
      基于实测车体系 vx/vy/vw 与参考窗口求下一个小速度增量；
      tracing_node 发布求得的底盘系 vx/vy/vw。

这里保存的是名义速度相位，不是累计位置参考。MPC 仍用实际状态和第一未来参考点的误差求控制；第一参考位置每周期只从实测投影向前积分一个 dt，因此车辆暂时不动时不会让位置误差无限增长。

## 11. 关键安全语义与当前实现边界

### 11.1 当前临时接口：只锁存一条路径

PathGeometry 和 TrajectoryGenerator 不认识 ROS 消息；路径代次只应由 ROS 适配层管理。
当前规划器仍发布 nav_msgs/Path 且会以 2 Hz 重发，因此 tracing_node 处于单路径测试模式：

- 第一条通过 frame 与 build 检查的 Path 被缓存；
- 已启用时，它从当前实测位置投影并开始跟踪；
- 之后所有 Path 都不再检查 frame、不再 build、不再改变 progress_ 或终点状态；
- 想换测试路径必须重启 tracing_node；
- TrackingStatus.path_id 为固定的 0，表示“该输入接口没有路径代次”。

后续规划器切换到 robot_interfaces/PlanPath 时，应恢复以下规则：相同 ID 是保活，
新 ID 才重建；几何变化必须伴随 ID 递增。

### 11.2 “正常、紧急、急停”不是同一件事

| 阶段 | 库状态/节点状态 | 车辆命令 |
| --- | --- | --- |
| 正常可停 | Ready / TRACKING | MPC 使用正常速度增量限值。 |
| 正常不够、紧急够 | EmergencyBraking / EMERGENCY_BRAKING | MPC 仍求解，但增量限值换成 emergency_mpc_accel_。 |
| 紧急仍不够 | EmergencyInfeasible / EMERGENCY_STOP | 节点立即发布零，不伪造平滑可停车参考。外部安全层决定实际急停。 |

发布零 Twist 并不等于底盘物理上能立刻停住；它只是要求下游不再执行跟踪速度。紧急减速度参数必须在实车上按稳定可实现的能力标定，不能为了让软件“看起来可行”任意填大。

### 11.3 当前第一版明确不做的事

- 不解析 CSV；外层节点把路径消息转换为 Point2 即可。
- 不做障碍物碰撞检测；规划器必须先按“车体外廓 + 安全余量 + max_smooth_deviation”膨胀障碍。
- 不规划 yaw；仅透传外部 HeadingProvider，当前节点固定 yaw。
- 不处理反向沿路径运动；速度投影为负时被夹成 0。
- 不使用 tf2；map 与 odom 必须数值一致。
- 不发布给轮组，也不改 commmux_node；只发布 cmd_track。
- 不使用 ROS Action；开关负责任务启停，当前 nav_msgs/Path 负责任务内重规划，TrackingStatus 负责反馈。

## 12. 建议的源码阅读顺序

第一次阅读建议不要从 translational_trajectory.cpp 的 L1 开始硬看。按以下顺序最容易建立因果关系：

1. 看 tracing_node.cpp 的 controlTick（L345--508）：先理解什么时候会动、什么时候一定停。
2. 看 planCallback、odomCallback、controllerCallback（L170--294）：理解谁让控制条件满足。
3. 看 tracing_adapter.hpp：确认世界系与机体系速度为何两次旋转。
4. 看 hpp 的 PathGeometry、TrajectoryGenerator、ReferencePoint：确认 cpp 里的缓存含义。
5. 看 PathGeometry::build 和 sample/project：理解几何路径来自哪里。
6. 看 rebuildProfile：理解曲率、终点制动和紧急状态为何能提前影响速度。
7. 最后看 sampleProfile 与 makeHorizon：把弧长速度包络转换为 MPC 所需时间窗口。

当调试某个现象时，可以按症状反查：

| 现象 | 优先看哪里 |
| --- | --- |
| 节点一直不走 | controlTick 的前四个 return、TrackingStatus.detail、frame_id。 |
| 需要换一条测试路径 | 单路径模式故意忽略后续 Path；重启 tracing_node 后再让规划器发布。 |
| 新路径被拒绝 | activatePath 的 projection.distance 和 max_activation_offset。 |
| 弯道仍过快 | PathGeometry 曲率、max_lateral_accel、profile_spacing、平滑偏差。 |
| 终点停不住 | diagnostics 的 normal_stop_distance、emergency_stop_distance、terminal_speed_if_unstoppable。 |
| 参考速度方向错 | bodyVelocityToWorld 与 worldVelocityToBody 的 yaw，以及 Odometry.child_frame_id。 |
| MPC 经常失败 | 先确认 reference.size()==N，再检查 MPC 权重、误差限幅、速度增量约束和 OSQP 日志。 |

## 13. 最小纯 C++ 调用范例

纯库不依赖 ROS，调用关系可以抽象成下面这样：

    using namespace robot_control::trajectory;

    MotionLimits limits;
    limits.cruise_speed = 1.2;
    limits.max_accel = 2.0;
    limits.normal_decel = 2.0;
    limits.emergency_decel = 3.0;
    limits.max_lateral_accel = 3.0;
    limits.terminal_speed = 0.0;

    PathBuildOptions build_options;
    PathGeometry path;
    std::string error;
    if (!path.build(planner_points, build_options, &error)) {
      // 不要生成控制命令；error 说明输入为何被拒绝。
    }

    TrajectoryGenerator generator(limits);
    MotionState2D state;
    state.position = current_world_position;
    state.velocity = current_world_velocity;
    if (!generator.activatePath(path, state, &error)) {
      // 例如车离新路径过远；由上层选择停车或等待下一条路径。
    }

    std::vector<ReferencePoint> reference;
    auto status = generator.makeHorizon(state, 0.05, 20, &reference);

真实 ROS 节点还需要负责：

1. 将 Odometry 机体系 vx/vy 转到 state.velocity 的世界系；
2. 把 ReferencePoint.velocity 从世界系转到现有 MPC 期待的参考车体系；
3. 根据 TrajectoryStatus 选择正常/紧急 MPC 限制或安全停机；
4. 将 MPC 输出按底盘约定发布。

这也正是 tracing_node 的职责边界。
