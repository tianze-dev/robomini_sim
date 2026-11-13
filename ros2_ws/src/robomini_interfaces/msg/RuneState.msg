# 能量机关/裁判状态，接口 /rune/state（20 Hz）
#
# mode          : fixed / random_fixed / small / big
# stage         : 阶段枚举，0=FIXED, 1=RANDOM_FIXED, 2=SMALL, 3=BIG（与 mode 一一对应）
# round         : 小/大能量机关轮次；固定靶、随机固定靶阶段为 0，进入第 1 轮小/大后为 1 并逐轮 +1
# hub_angle     : 机关旋转盘当前关节角（rad，绕世界 +Y，右手定则；来自 gz 插件 /rune/<team>/state）
# speed         : 机关当前角速度（rad/s，来自 gz 插件）
# target_angles : 5 个靶当前在 X-Z 平面内的极角（rad，atan2(dz, dx)，从 +X 轴逆时针），
#                 已包含 hub_angle 旋转：target_angles[i] = theta0[i] - hub_angle
# target_states : 5 个靶当前灯态，0=off（未激活/未选中）, 1=pending（待激活，唯一有效目标）,
#                 2=active（已激活）
# targets       : 5 个靶当前灯态的字符串形式（"off"/"pending"/"active"），与 target_states 一一对应，
#                 方便可视化/调试（与 gz /rune/<team>/lights 的 JSON 字段同义）
# score         : 当前总分（含扣分/罚分，等价于 /score.total - /score.penalty）
# control_mode  : manual / auto，决定每次命中的分值档（人工/自主）
#
# 说明：target_angles 与 target_states 的下标 i 即 gz 侧 tgt_<team>_<i>，按 X-Z 极角升序固定编号。

string mode
uint8 stage
uint32 round
float64 hub_angle
float64 speed
float64[5] target_angles
uint8[5] target_states
string[5] targets
int32 score
string control_mode

# stamp：本消息发布的仿真时刻（use_sim_time，来自 /clock）
builtin_interfaces/Time stamp
