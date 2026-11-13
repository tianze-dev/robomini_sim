# 视觉 -> 云台/下位机，接口 /vision_to_gimbal
#
# mode : 0 = 不控制
#        1 = 控制云台，不开火
#        2 = 控制云台，且开火（仿真侧按 3 发/s 上限节流）
# 角度单位 rad，角速度 rad/s，角加速度 rad/s²；

uint8 mode
float64 yaw
float64 yaw_vel
float64 yaw_acc
float64 pitch
float64 pitch_vel
float64 pitch_acc
