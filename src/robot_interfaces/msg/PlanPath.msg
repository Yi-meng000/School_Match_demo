std_msgs/Header header
uint32 path_id                       # 路径代次，仅在几何变化时自增
geometry_msgs/PoseStamped[] poses    # 路径点串（map 系，不含机器人当前位姿）
