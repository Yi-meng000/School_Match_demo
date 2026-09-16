uint8 enable        # 使能标志: 0x00 失能, 0x01 使能
uint8 protect       # 保护标志: 0x00 正常, 0x01 保护锁死
uint8 lockpoint     # 锁点标志: 0x00 未锁点, 0x01 持续锁点
uint8 trajectory    # 轨迹标志: 0x00 无寻迹, 0x01 寻迹到‘
uint16 goal_x       # 目标点 X 坐标 (mm)
uint16 goal_y       # 目标点 Y 坐标 (mm)
uint16 goal_yaw     # 目标点 Yaw 角度 (0~3600, 0.01°/bit)
float32 vx          # X 轴线速度 (前进/后退, m/s)
float32 vy          # Y 轴线速度 (横向平移, m/s)
float32 vw          # Z 轴角速度 (旋转偏航, rad/s)