# 记分板，接口 /score（20 Hz）
#
# total       : 当前总分 = 有效命中得分 - 扣分（与 /rune/state.score 相同；毛得分 = total + penalty）
# mode_hits   : 四种模式的有效命中次数，下标 0=fixed, 1=random_fixed, 2=small, 3=big
# penalty     : 累计扣分（正数；来自超额发射：每超发 1 球 +20 分扣分）
# balls_fired : 本场累计已发射球数（由裁判节点按发射事件统计）

int32 total
int32[4] mode_hits
int32 penalty
uint32 balls_fired
