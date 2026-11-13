# 云台/下位机 -> 视觉，接口 /gimbal_to_vision
#
# mode         : 0=空闲, 1=自瞄
# q            : IMU 四元数，顺序为 w, x, y, z
# yaw/yaw_vel  : 云台实测偏航角（rad）/ 角速度（rad/s）
# pitch/pitch_vel : 云台实测俯仰角（rad）/ 角速度（rad/s）
# bullet_speed : 实测弹速（m/s）
# bullet_count : 累计发射弹数
#
# 说明：仿真中 q 直接取 pitch_link 上的 gz IMU 姿态（200 Hz），与实车下位机回传
# IMU 四元数一致；仅在 IMU 尚未就绪时用 q = Rz(yaw)·Rx(pitch) 兜底。
# 子弹速度取仿真发射速度，bullet_count 取累计发球数。
uint8 mode
float64[4] q
float64 yaw
float64 yaw_vel
float64 pitch
float64 pitch_vel
float64 bullet_speed
uint16 bullet_count
