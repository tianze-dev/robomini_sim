# 一次命中事件，接口 /rune/hit（由裁判节点判定后发布）
#
# target     : 被击中的靶索引 0..4（gz 侧 tgt_<team>_<i>）
# valid      : 裁判判定是否有效；只有“命中当前唯一 pending 靶 + 本次发射合法”才为 true
# score      : 本次命中得分（无效时为 0）；分值按 mode + control_mode 查表：
#              fixed 15/30, random_fixed 20/40, small 25/50, big 30/60（人工/自主）
# was_pending: 命中时该靶是否为当前待激活靶（用于区分“打错靶”和“命中有效靶但发射不合法”）
# ball       : 球名（来自 gz 插件，如 ball_07）；插件未提供时为空串
# speed      : 球速（m/s）；插件未提供时为 0.0
# stamp      : 命中事件的仿真时刻（来自 gz 插件 /rune/<team>/hit 的 stamp）

uint8 target
bool valid
int32 score
bool was_pending
string ball
float64 speed
builtin_interfaces/Time stamp
