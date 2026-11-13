"""能量机关裁判状态机（纯 Python 逻辑，**不依赖 ROS**，可直接单测）。

规则依据：《ROBOMINI2027 机甲杯规则手册》5.5/7.3：

* 阶段推进：``FIXED -> RANDOM_FIXED -> (SMALL|BIG)*``；FIXED/RANDOM_FIXED 每场各一次；
  之后每轮在 SMALL/BIG 之间选择（本实现 auto_advance 时按 SMALL→BIG→SMALL... 交替，
  也可通过 :meth:`RuneStateMachine.advance` / :meth:`RuneStateMachine.set_mode` 指定）。
* 每阶段/轮：依次有效命中 5 个**不同**靶（五靶全部 active）即完成。
* 目标选择：FIXED 命中后从未激活靶中随机选下一个；RANDOM_FIXED/SMALL/BIG 待激活靶显示
  3 s 未命中则从未激活靶中换一个；已激活靶永不重选；FIXED 未命中不换。
* 有效命中（裁判侧）：该靶是当前唯一 pending；本次发射合法（额度>0）；直接命中。
* 计分（每次有效命中）：FIXED 15/30、RANDOM_FIXED 20/40、SMALL 25/50、BIG 30/60（人工/自主），
  control_mode=manual 取前者，auto 取后者（默认 auto）。
* 超额发射（ammo_mode=counting 且已发射数 > 额度）：每超发 1 球扣 20 分，对应命中无效、不推进。

本模块与 ROS 解耦：时间全部由调用方以**仿真时间**显式传入；节点层（``rune_referee.py``）
负责把 gz 插件 JSON、`/clock`、ROS 消息与本状态机对接。
"""

from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass
from typing import Optional

from .json_codec import BIG_A_RANGE, BIG_OMEGA_RANGE, TARGET_STATE_NAMES

__all__ = [
    "FIXED",
    "RANDOM_FIXED",
    "SMALL",
    "BIG",
    "MODES",
    "MODE_INDEX",
    "STAGE_FIXED",
    "STAGE_RANDOM_FIXED",
    "STAGE_SMALL",
    "STAGE_BIG",
    "TARGET_OFF",
    "TARGET_PENDING",
    "TARGET_ACTIVE",
    "MODE_SCORES",
    "AUTO_ADVANCE_NEXT",
    "SWITCH_INTERVAL",
    "TARGET_COUNT",
    "OVER_QUOTA_PENALTY",
    "CONTROL_MODES",
    "AMMO_MODES",
    "HitResult",
    "ShotResult",
    "TickResult",
    "AdvanceResult",
    "RuneStateMachine",
]

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

FIXED = "fixed"
RANDOM_FIXED = "random_fixed"
SMALL = "small"
BIG = "big"
MODES = (FIXED, RANDOM_FIXED, SMALL, BIG)
MODE_INDEX = {mode: i for i, mode in enumerate(MODES)}

#: ``RuneState.stage`` 的取值（与 mode 一一对应）
STAGE_FIXED = 0
STAGE_RANDOM_FIXED = 1
STAGE_SMALL = 2
STAGE_BIG = 3

TARGET_OFF = TARGET_STATE_NAMES.index("off")        # 0
TARGET_PENDING = TARGET_STATE_NAMES.index("pending")  # 1
TARGET_ACTIVE = TARGET_STATE_NAMES.index("active")    # 2

#: 每次有效命中分值：(人工, 自主)（规则表 5-2）
MODE_SCORES = {
    FIXED: (15, 30),
    RANDOM_FIXED: (20, 40),
    SMALL: (25, 50),
    BIG: (30, 60),
}

#: auto_advance 时的默认下一模式（确定性便于复现；服务可强制指定）
AUTO_ADVANCE_NEXT = {
    FIXED: RANDOM_FIXED,
    RANDOM_FIXED: SMALL,
    SMALL: BIG,
    BIG: SMALL,
}

#: 待激活靶显示时长（s），仅 RANDOM_FIXED/SMALL/BIG 生效
SWITCH_INTERVAL = 3.0
TARGET_COUNT = 5
#: 每超额发射 1 球的扣分（规则 7.3）
OVER_QUOTA_PENALTY = 20

CONTROL_MODES = ("manual", "auto")
AMMO_MODES = ("unlimited", "counting")
SWITCH_MODES = (RANDOM_FIXED, SMALL, BIG)


# ---------------------------------------------------------------------------
# 结果对象
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class HitResult:
    """一次命中判定的结果（节点据此发布 RuneHit / 更新灯效）。"""

    target: int
    valid: bool
    score: int
    was_pending: bool
    reason: str
    ball: str = ""
    speed: float = 0.0
    stamp: float = 0.0
    stage_completed: bool = False
    advanced: bool = False


@dataclass(frozen=True)
class ShotResult:
    """一次发射对额度/扣分的影响。"""

    legal: bool
    over_quota: bool
    shots_fired: int
    penalty_added: int


@dataclass(frozen=True)
class TickResult:
    """一次时间推进的结果（3 s 换靶定时器）。"""

    switched: bool
    old_pending: Optional[int]
    new_pending: Optional[int]


@dataclass(frozen=True)
class AdvanceResult:
    """阶段/轮推进（含 /rune/set_mode、/rune/next_stage 服务）结果。"""

    prev_mode: str
    new_mode: str
    round: int
    a: Optional[float]
    omega: Optional[float]
    t0: float
    reason: str


# ---------------------------------------------------------------------------
# 状态机
# ---------------------------------------------------------------------------


class RuneStateMachine:
    """能量机关打靶规则状态机（纯逻辑）。

    典型用法（节点层）::

        sm = RuneStateMachine(control_mode="auto", ammo_mode="unlimited",
                              target_base_angles=geometry.base_angles_rad("blue"))
        sm.tick(sim_time)                 # 20 Hz 定时器里推进 3 s 换靶
        sm.on_shot(ball="ball_03")        # /gimbal_to_vision.bullet_count 增加时调用
        res = sm.on_hit(3, ball="ball_03")  # 收到 /rune/blue/hit
        snap = sm.snapshot()              # 发布 RuneState
    """

    def __init__(
        self,
        control_mode: str = "auto",
        ammo_mode: str = "unlimited",
        ammo_quota: int = 10,
        auto_advance: bool = True,
        switch_interval: float = SWITCH_INTERVAL,
        big_a: Optional[float] = None,
        big_omega: Optional[float] = None,
        over_quota_penalty: int = OVER_QUOTA_PENALTY,
        target_base_angles=None,
        target_count: int = TARGET_COUNT,
        require_direct: bool = False,
        rng: Optional[random.Random] = None,
    ) -> None:
        if control_mode not in CONTROL_MODES:
            raise ValueError(f"control_mode must be one of {CONTROL_MODES}, got {control_mode!r}")
        if ammo_mode not in AMMO_MODES:
            raise ValueError(f"ammo_mode must be one of {AMMO_MODES}, got {ammo_mode!r}")
        if int(ammo_quota) < 0:
            raise ValueError(f"ammo_quota must be >= 0, got {ammo_quota!r}")
        if float(switch_interval) <= 0.0:
            raise ValueError(f"switch_interval must be > 0, got {switch_interval!r}")
        if int(target_count) < 2:
            raise ValueError(f"target_count must be >= 2, got {target_count!r}")
        if big_a is not None and not BIG_A_RANGE[0] <= float(big_a) <= BIG_A_RANGE[1]:
            raise ValueError(f"big_a={big_a} outside {BIG_A_RANGE}")
        if big_omega is not None and not BIG_OMEGA_RANGE[0] <= float(big_omega) <= BIG_OMEGA_RANGE[1]:
            raise ValueError(f"big_omega={big_omega} outside {BIG_OMEGA_RANGE}")

        self.control_mode = control_mode
        self.ammo_mode = ammo_mode
        self.ammo_quota = int(ammo_quota)
        self.auto_advance = bool(auto_advance)
        self.switch_interval = float(switch_interval)
        self.over_quota_penalty = int(over_quota_penalty)
        self.target_count = int(target_count)
        self.require_direct = bool(require_direct)
        self._fixed_big_a = None if big_a is None else float(big_a)
        self._fixed_big_omega = None if big_omega is None else float(big_omega)
        self._rng = rng if rng is not None else random.Random()

        if target_base_angles is None:
            # 仅用于无几何信息的纯逻辑测试；节点层必须传入真实 θ0
            self._base_angles = tuple(2.0 * math.pi * i / self.target_count for i in range(self.target_count))
        else:
            if len(target_base_angles) != self.target_count:
                raise ValueError(
                    f"target_base_angles needs {self.target_count} entries, got {len(target_base_angles)}"
                )
            self._base_angles = tuple(float(a) for a in target_base_angles)

        self.reset(now=0.0)

    # -- 只读属性 ---------------------------------------------------------

    @property
    def stage(self) -> int:
        """RuneState.stage：0=FIXED, 1=RANDOM_FIXED, 2=SMALL, 3=BIG。"""
        return MODE_INDEX[self.mode]

    @property
    def score(self) -> int:
        """净得分 = 有效命中得分 - 扣分。"""
        return self.points - self.penalty

    @property
    def remaining_ammo(self) -> Optional[int]:
        """counting 模式剩余额度；unlimited 返回 None。"""
        if self.ammo_mode == "unlimited":
            return None
        return max(0, self.ammo_quota - self.balls_fired)

    @property
    def accepting_hits(self) -> bool:
        """当前是否处于“可被有效命中”的状态（有唯一 pending 靶且未处于阶段完成待推进）。"""
        return self.pending is not None and not self.stage_completed

    # -- 状态初始化/快照 ---------------------------------------------------

    def reset(self, now: float = 0.0) -> dict:
        """复位到初始固定靶阶段（清空记分/灯态/发射计数），返回新快照。"""
        now = float(now)
        self._now = now
        self.mode = FIXED
        self.round = 0
        self.target_states = [TARGET_OFF] * self.target_count
        self.stage_hits = 0
        self.points = 0
        self.penalty = 0
        self.mode_hits = [0, 0, 0, 0]
        self.balls_fired = 0
        self.hub_angle = 0.0
        self.speed = 0.0
        self.a = None
        self.omega = None
        self.stage_completed = False
        self._completed_info = None
        self._pending_since = now
        self._mode_started_at = now
        self._mode_revision = 0
        self._illegal_balls = deque(maxlen=64)
        self._over_quota_unconsumed = 0
        self.pending = None
        self._pick_pending(now)
        return self.snapshot()

    def snapshot(self) -> dict:
        """完整状态快照（节点层用于发布 /rune/state、/score）。"""
        return {
            # RuneState 字段
            "mode": self.mode,
            "stage": self.stage,
            "round": self.round,
            "hub_angle": float(self.hub_angle),
            "speed": float(self.speed),
            "target_angles": self.target_angles(),
            "target_states": list(self.target_states),
            "score": self.score,
            "control_mode": self.control_mode,
            # 内部/调试字段
            "pending": self.pending,
            "stage_hits": self.stage_hits,
            "points": self.points,
            "penalty": self.penalty,
            "mode_hits": list(self.mode_hits),
            "balls_fired": self.balls_fired,
            "a": self.a,
            "omega": self.omega,
            "mode_t0": self._mode_started_at,
            "stage_completed": self.stage_completed,
            "accepting_hits": self.accepting_hits,
            "mode_revision": self._mode_revision,
            "switch_interval": self.switch_interval,
        }

    def target_angles(self) -> list:
        """5 个靶当前 X-Z 极角（rad）：``theta0_i - hub_angle``，归一到 (-pi, pi]。"""
        hub = float(self.hub_angle)
        return [math.atan2(math.sin(theta0 - hub), math.cos(theta0 - hub)) for theta0 in self._base_angles]

    def lights(self) -> list:
        """5 靶灯态整数列表（0/1/2），即 /rune/<team>/lights 的编码来源。"""
        return list(self.target_states)

    def mode_command_args(self) -> dict:
        """编码 /rune/<team>/mode 所需的参数：``{"mode","a","omega","t0"}``。"""
        return {"mode": self.mode, "a": self.a, "omega": self.omega, "t0": self._mode_started_at}

    # -- 外部输入：gz 插件状态回显 -----------------------------------------

    def on_plugin_state(
        self,
        hub_angle: Optional[float] = None,
        speed: Optional[float] = None,
        now: Optional[float] = None,
    ) -> None:
        """吸收 gz 插件 /rune/<team>/state 的 hub_angle/speed（用于 RuneState 与 target_angles）。"""
        if now is not None:
            self._now = float(now)
        if hub_angle is not None:
            self.hub_angle = float(hub_angle)
        if speed is not None:
            self.speed = float(speed)

    # -- 外部输入：发射/命中 -------------------------------------------------

    def on_shot(self, ball: Optional[str] = None, now: Optional[float] = None) -> ShotResult:
        """登记一次实际发射（每发球调用一次，来源 /gimbal_to_vision.bullet_count）。

        counting 模式下 ``balls_fired > ammo_quota`` 的发射即“超额发射”：
        立即扣分，并记下该球（命中到达时判无效）。unlimited 模式只计数不扣分。
        """
        if now is not None:
            self._now = float(now)
        self.balls_fired += 1
        if self.ammo_mode == "counting" and self.balls_fired > self.ammo_quota:
            self.penalty += self.over_quota_penalty
            if ball:
                self._illegal_balls.append(str(ball))
            else:
                self._over_quota_unconsumed += 1
            return ShotResult(
                legal=False,
                over_quota=True,
                shots_fired=self.balls_fired,
                penalty_added=self.over_quota_penalty,
            )
        return ShotResult(legal=True, over_quota=False, shots_fired=self.balls_fired, penalty_added=0)

    def on_hit(
        self,
        target,
        ball: str = "",
        speed: float = 0.0,
        direct: bool = True,
        stamp: float = 0.0,
        now: Optional[float] = None,
    ) -> HitResult:
        """判定一次几何命中（来源 /rune/<team>/hit），必要时推进阶段。

        判定顺序（与规则 5.5.3 一致）：
        1. 靶索引合法且当前只有一个 pending 靶；
        2. 被击中靶就是该 pending 靶（击中已激活/未选中靶 -> 无效，不推进）；
        3. 本次发射合法（counting 模式超额球的命中无效）；
        4. ``require_direct`` 时反弹球（direct=False）无效（默认关闭，不强制）。
        """
        if now is not None:
            self._now = float(now)
        try:
            target_int = int(target)
        except (TypeError, ValueError):
            return HitResult(target=-1, valid=False, score=0, was_pending=False, reason="out_of_range",
                             ball=ball, speed=speed, stamp=stamp)

        in_range = 0 <= target_int < self.target_count
        was_pending = bool(
            in_range
            and self.pending == target_int
            and self.target_states[target_int] == TARGET_PENDING
        )
        reason = "valid"
        valid = False
        score = 0

        if not in_range:
            reason = "out_of_range"
        elif self.pending is None:
            reason = "not_accepting"
        elif not was_pending:
            reason = "already_active" if self.target_states[target_int] == TARGET_ACTIVE else "not_pending"
        elif self.require_direct and not direct:
            reason = "indirect"
        elif self._consume_over_quota_shot(ball):
            reason = "over_quota"
        else:
            valid = True
            score = self._points_per_hit()
            self.target_states[target_int] = TARGET_ACTIVE
            self.points += score
            self.mode_hits[MODE_INDEX[self.mode]] += 1
            self.stage_hits += 1
            self.pending = None

        stage_completed = False
        advanced = False
        if valid:
            stage_completed = self.stage_hits >= self.target_count
            if stage_completed:
                self.stage_completed = True
                self._completed_info = {"mode": self.mode, "round": self.round}
                if self.auto_advance:
                    self.advance(now=self._now, reason="auto")
                    advanced = True
            else:
                self._pick_pending(self._now)

        return HitResult(
            target=target_int,
            valid=valid,
            score=score,
            was_pending=was_pending,
            reason=reason,
            ball=ball,
            speed=speed,
            stamp=stamp,
            stage_completed=stage_completed,
            advanced=advanced,
        )

    def tick(self, now: float) -> TickResult:
        """推进 3 s 换靶定时器（20 Hz 定时器里调用）。

        仅 RANDOM_FIXED/SMALL/BIG 生效；FIXED 未命中时持续保持当前待激活目标。
        """
        now = float(now)
        if now < self._pending_since:
            # 仿真时间回退（world reset）：把计时重新对齐，避免立即触发换靶
            self._pending_since = now
        self._now = now
        if (
            self.mode in SWITCH_MODES
            and self.pending is not None
            and not self.stage_completed
            and now - self._pending_since >= self.switch_interval
        ):
            old_pending = self.pending
            self.target_states[old_pending] = TARGET_OFF
            self.pending = None
            self._pick_pending(now, exclude=old_pending)
            return TickResult(switched=True, old_pending=old_pending, new_pending=self.pending)
        return TickResult(switched=False, old_pending=self.pending, new_pending=self.pending)

    # -- 阶段/轮推进 -------------------------------------------------------

    def advance(
        self,
        mode: Optional[str] = None,
        a: Optional[float] = None,
        omega: Optional[float] = None,
        now: Optional[float] = None,
        reason: str = "service",
    ) -> AdvanceResult:
        """推进到下一阶段/轮（或强制指定模式），清空 5 靶灯态、重新选首个待激活靶。

        默认序列：FIXED→RANDOM_FIXED→SMALL→BIG→SMALL→BIG...；
        ``mode`` 可强制指定（/rune/set_mode、/rune/next_stage 使用）。
        big 模式每轮随机或使用给定的 a/omega；round 在进入每个 SMALL/BIG 轮时 +1。
        """
        now = float(self._now if now is None else now)
        prev_mode = self.mode
        if mode is None:
            new_mode = AUTO_ADVANCE_NEXT[self.mode]
        else:
            if mode not in MODES:
                raise ValueError(f"invalid mode {mode!r}, expected one of {MODES}")
            new_mode = mode

        new_a: Optional[float] = None
        new_omega: Optional[float] = None
        if new_mode == BIG:
            new_a = self._resolve_big_a(a)
            new_omega = self._resolve_big_omega(omega)

        if new_mode in (SMALL, BIG):
            self.round += 1
        else:
            self.round = 0
        self.mode = new_mode
        self.a = new_a
        self.omega = new_omega
        self.target_states = [TARGET_OFF] * self.target_count
        self.stage_hits = 0
        self.stage_completed = False
        self._completed_info = None
        self._mode_started_at = now
        self._mode_revision += 1
        self._now = now
        self.pending = None
        self._pick_pending(now)
        return AdvanceResult(
            prev_mode=prev_mode,
            new_mode=new_mode,
            round=self.round,
            a=self.a,
            omega=self.omega,
            t0=self._mode_started_at,
            reason=reason,
        )

    def set_mode(
        self,
        mode: str,
        a: Optional[float] = None,
        omega: Optional[float] = None,
        now: Optional[float] = None,
    ) -> AdvanceResult:
        """强制切换到指定模式（测试用；允许重复选择同一模式，进入新一轮）。"""
        return self.advance(mode=mode, a=a, omega=omega, now=now, reason="set_mode")

    # -- 内部 --------------------------------------------------------------

    def _points_per_hit(self) -> int:
        manual_points, auto_points = MODE_SCORES[self.mode]
        return manual_points if self.control_mode == "manual" else auto_points

    def _pick_pending(self, now: float, exclude: Optional[int] = None) -> Optional[int]:
        """从未激活靶中随机选一个作为当前唯一待激活靶；已激活靶永不重选。"""
        candidates = [
            i
            for i in range(self.target_count)
            if self.target_states[i] == TARGET_OFF and i != exclude
        ]
        if not candidates:
            self.pending = None
            return None
        chosen = self._rng.choice(candidates)
        self.target_states[chosen] = TARGET_PENDING
        self.pending = chosen
        self._pending_since = float(now)
        return chosen

    def _consume_over_quota_shot(self, ball: str) -> bool:
        """判断该命中是否来自超额发射；是则消耗一次记录并返回 True。"""
        if self.ammo_mode != "counting":
            return False
        if ball:
            ball = str(ball)
            if ball in self._illegal_balls:
                self._illegal_balls.remove(ball)
                return True
            return False
        if self._over_quota_unconsumed > 0:
            self._over_quota_unconsumed -= 1
            return True
        return False

    def _resolve_big_a(self, a: Optional[float]) -> float:
        if a is not None and float(a) > 0.0:
            value = float(a)
            if not BIG_A_RANGE[0] <= value <= BIG_A_RANGE[1]:
                raise ValueError(f"big a={value} outside {BIG_A_RANGE}")
            return value
        if self._fixed_big_a is not None:
            return self._fixed_big_a
        return self._rng.uniform(*BIG_A_RANGE)

    def _resolve_big_omega(self, omega: Optional[float]) -> float:
        if omega is not None and float(omega) > 0.0:
            value = float(omega)
            if not BIG_OMEGA_RANGE[0] <= value <= BIG_OMEGA_RANGE[1]:
                raise ValueError(f"big omega={value} outside {BIG_OMEGA_RANGE}")
            return value
        if self._fixed_big_omega is not None:
            return self._fixed_big_omega
        return self._rng.uniform(*BIG_OMEGA_RANGE)
