# 规划器输出的路径。相比裸 nav_msgs/Path，多了两个下游必需的信息。
#
# 规划器侧的约定（很重要，这是整个重规划处理能简化成一行判断的前提）：
#   * path_id 只在**几何路径真正变化时**自增。路径不变时复用同一个 id，
#     哪怕每 2 Hz 都在重发。若每拍都自增，下游就无法区分"同一条路径重发"
#     和"因为新障碍物重新规划"，只能退化去做 Fréchet/Hausdorff 几何比对。
#   * start_pose 单独给出这条路径是从哪个位姿规划出来的。poses 里**不要**
#     再把机器人当前位姿当第一个点混进去 —— 那个点不在规划器的格点上，
#     会在路径起点引入一个远大于格距的拐折，污染抽稀和平滑。
#
# 建议同时把 /plan 的 QoS 改成 transient_local + depth 1，并且只在路径变化时发布。
# 这样下游收一次就够，带宽和歧义一起消失。
std_msgs/Header header
uint32 path_id                       # 路径代次，仅在几何变化时自增
geometry_msgs/PoseStamped start_pose # 这条路径的规划起点
geometry_msgs/PoseStamped goal_pose  # 终点
geometry_msgs/PoseStamped[] poses    # 路径点串（map 系，不含机器人当前位姿）
