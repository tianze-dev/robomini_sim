#!/usr/bin/env python3
"""能量机关裁判节点（裁判接口）。

职责
----
1. 用纯逻辑状态机 :class:`robomini_sim.rune_state_machine.RuneStateMachine` 做规则判定；
2. 通过 ``std_msgs/String`` JSON（经 ros_gz_bridge 与 gz 插件通信）：
   发布 ``/rune/<team>/mode``、``/rune/<team>/lights``；
   订阅 ``/rune/<team>/hit``、``/rune/<team>/state``；
3. 对上发布仿真/裁判接口 ``/rune/state``、``/rune/hit``、``/score``；
4. 提供服务 ``/rune/set_mode``、``/rune/reset``、``/rune/next_stage``、``/sim/reset``。

注意：这些是仿真/裁判侧接口，实车视觉↔下位机通信中没有对应项；自瞄算法不应依赖。

命中事件处理链路：gz 插件几何命中 -> 桥 -> 本节点 on_hit 判定 -> 立即发布
``/rune/hit`` + 灯效 ``/rune/<team>/lights``（延迟 < 50 ms，仅受 ROS/gz 传输影响）。
"""

import math
import subprocess

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robomini_interfaces.msg import GimbalToVision, RuneHit, RuneState, ScoreBoard
from robomini_interfaces.srv import SetRuneMode

from . import geometry, json_codec
from .rune_state_machine import RuneStateMachine


class RuneReferee(Node):
    """裁判节点：规则状态机 + gz JSON 编解码 + 接口。"""

    def __init__(self) -> None:
        super().__init__("rune_referee")

        # ------------------------------------------------------------ 参数
        self.declare_parameter("side", "blue")
        self.declare_parameter("auto_advance", True)
        self.declare_parameter("control_mode", "auto")
        self.declare_parameter("ammo_mode", "unlimited")
        self.declare_parameter("ammo_quota", 10)
        self.declare_parameter("switch_interval", 3.0)
        self.declare_parameter("big_a", 0.0)       # <=0 表示每轮在合法范围内随机
        self.declare_parameter("big_omega", 0.0)   # <=0 表示每轮在合法范围内随机
        self.declare_parameter("state_rate", 20.0)
        self.declare_parameter("refresh_period", 1.0)   # 启动/状态变化后的重发间隔（防桥启动晚丢帧）
        self.declare_parameter("refresh_window", 5.0)   # 只在启动/变化后 N 秒内周期重发，避免覆盖调试脚本的手动灯效
        self.declare_parameter("world_name", "robomini")
        self.declare_parameter("reset_world", True)

        side = str(self.get_parameter("side").value)
        if side not in geometry.TEAMS:
            raise RuntimeError(f"参数 side 必须是 {geometry.TEAMS}，收到 {side!r}")
        self.side = side
        self.world_name = str(self.get_parameter("world_name").value) or "robomini"
        self.reset_world = bool(self.get_parameter("reset_world").value)
        self.refresh_period = float(self.get_parameter("refresh_period").value)
        self.refresh_window = float(self.get_parameter("refresh_window").value)
        state_rate = max(1.0, float(self.get_parameter("state_rate").value))

        big_a = float(self.get_parameter("big_a").value)
        big_omega = float(self.get_parameter("big_omega").value)
        self.sm = RuneStateMachine(
            control_mode=str(self.get_parameter("control_mode").value),
            ammo_mode=str(self.get_parameter("ammo_mode").value),
            ammo_quota=int(self.get_parameter("ammo_quota").value),
            auto_advance=bool(self.get_parameter("auto_advance").value),
            switch_interval=float(self.get_parameter("switch_interval").value),
            big_a=big_a if big_a > 0.0 else None,
            big_omega=big_omega if big_omega > 0.0 else None,
            target_base_angles=geometry.base_angles_rad(side),
        )

        # ------------------------------------------------------------ 发布/订阅
        self.pub_state = self.create_publisher(RuneState, "/rune/state", 10)
        self.pub_hit = self.create_publisher(RuneHit, "/rune/hit", 10)
        self.pub_score = self.create_publisher(ScoreBoard, "/score", 10)
        self.pub_mode = self.create_publisher(String, f"/rune/{side}/mode", 10)
        self.pub_lights = self.create_publisher(String, f"/rune/{side}/lights", 10)

        self.create_subscription(String, f"/rune/{side}/hit", self._on_gz_hit, 10)
        self.create_subscription(String, f"/rune/{side}/state", self._on_gz_rune_state, 10)
        self.create_subscription(GimbalToVision, "/gimbal_to_vision", self._on_gimbal_to_vision, 10)

        # ------------------------------------------------------------ 服务
        self.create_service(SetRuneMode, "/rune/set_mode", self._srv_set_mode)
        self.create_service(Trigger, "/rune/reset", self._srv_reset)
        self.create_service(Trigger, "/rune/next_stage", self._srv_next_stage)
        self.create_service(Trigger, "/sim/reset", self._srv_sim_reset)

        # ------------------------------------------------------------ 内部状态
        self._last_log_time = {}
        self._last_mode_key = None
        self._last_lights = None
        self._last_broadcast = -1.0
        self._broadcast_until = -1.0
        self._last_bullet_count = None

        self.create_timer(1.0 / state_rate, self._on_timer)

        self._broadcast_until = self._sim_now() + self.refresh_window
        self._sync_outputs(force=True)
        self._publish_state()
        self._publish_score()
        self.get_logger().info(
            f"rune_referee 启动: side={side} control_mode={self.sm.control_mode} "
            f"ammo_mode={self.sm.ammo_mode}(quota={self.sm.ammo_quota}) "
            f"auto_advance={self.sm.auto_advance} world={self.world_name}"
        )

    # ------------------------------------------------------------------ 工具
    def _sim_now(self) -> float:
        """当前仿真时间（s）；未收到 /clock 时为 0。"""
        return float(self.get_clock().now().nanoseconds) * 1e-9

    @staticmethod
    def _to_time(seconds: float) -> Time:
        seconds = float(seconds)
        sec = int(math.floor(seconds))
        nanosec = int(round((seconds - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        if nanosec < 0:
            sec -= 1
            nanosec += 1_000_000_000
        return Time(sec=sec, nanosec=nanosec)

    def _log_throttled(self, level: str, key: str, message: str, period: float = 5.0) -> None:
        now = self._sim_now()
        last = self._last_log_time.get(key)
        if last is not None and now >= last and now - last < period:
            return
        self._last_log_time[key] = now
        getattr(self.get_logger(), level)(message)

    # ------------------------------------------------------------------ 输出同步
    def _sync_outputs(self, force: bool = False) -> None:
        """灯效/模式变化时立即下发给 gz 插件（period 重发防止桥启动晚丢首帧）。"""
        snap = self.sm.snapshot()
        changed = False
        mode_args = self.sm.mode_command_args()
        mode_key = (mode_args["mode"], mode_args["a"], mode_args["omega"], mode_args["t0"])
        if force or mode_key != self._last_mode_key:
            try:
                payload = json_codec.dumps(json_codec.encode_mode(**mode_args))
            except json_codec.RuneJsonError as exc:  # 理论上不会发生（状态机保证参数合法）
                self._log_throttled("error", "mode_payload", f"mode JSON 编码失败: {exc}", period=10.0)
            else:
                self.pub_mode.publish(String(data=payload))
                if mode_key != self._last_mode_key:
                    self.get_logger().info(
                        f"模式下发: {mode_args['mode']} "
                        f"(a={mode_args['a']}, omega={mode_args['omega']}, t0={mode_args['t0']:.3f})"
                    )
                self._last_mode_key = mode_key
                changed = True

        lights = tuple(int(s) for s in snap["target_states"])
        if force or lights != self._last_lights:
            payload = json_codec.dumps(json_codec.encode_lights(lights))
            self.pub_lights.publish(String(data=payload))
            self._last_lights = lights
            changed = True

        if changed:
            self._broadcast_until = max(self._broadcast_until, self._sim_now() + self.refresh_window)

    def _publish_state(self) -> None:
        snap = self.sm.snapshot()
        msg = RuneState()
        msg.mode = str(snap["mode"])
        msg.stage = int(snap["stage"])
        msg.round = int(snap["round"])
        msg.hub_angle = float(snap["hub_angle"])
        msg.speed = float(snap["speed"])
        msg.target_angles = [float(a) for a in snap["target_angles"]]
        msg.target_states = [int(s) for s in snap["target_states"]]
        msg.targets = [json_codec.target_state_name(int(s)) for s in snap["target_states"]]
        msg.score = int(snap["score"])
        msg.control_mode = str(snap["control_mode"])
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_state.publish(msg)

    def _publish_score(self) -> None:
        snap = self.sm.snapshot()
        msg = ScoreBoard()
        msg.total = int(snap["score"])
        msg.mode_hits = [int(v) for v in snap["mode_hits"]]
        msg.penalty = int(snap["penalty"])
        msg.balls_fired = int(snap["balls_fired"])
        self.pub_score.publish(msg)

    # ------------------------------------------------------------------ 定时器
    def _on_timer(self) -> None:
        now = self._sim_now()
        tick = self.sm.tick(now)
        if tick.switched:
            self.get_logger().info(f"3 s 未命中，待激活靶 {tick.old_pending} -> {tick.new_pending}")
        force = now < self._last_broadcast or (
            now < self._broadcast_until and now - self._last_broadcast >= self.refresh_period
        )
        self._sync_outputs(force=force)
        self._publish_state()
        self._publish_score()
        if force:
            self._last_broadcast = now

    # ------------------------------------------------------------------ gz 插件回调
    def _on_gz_rune_state(self, msg: String) -> None:
        try:
            state = json_codec.decode_rune_state(msg.data)
        except json_codec.RuneJsonError as exc:
            self._log_throttled("warn", "gz_state_json",
                                f"无法解析 /rune/{self.side}/state JSON（接口定义）: {exc}", period=5.0)
            return
        self.sm.on_plugin_state(hub_angle=state.hub_angle, speed=state.speed)
        if state.mode != self.sm.mode:
            self._log_throttled("warn", "gz_mode_mismatch",
                                f"插件回显 mode={state.mode} 与裁判 mode={self.sm.mode} 不一致（模式消息可能仍在传输）")

    def _on_gz_hit(self, msg: String) -> None:
        try:
            event = json_codec.decode_hit(msg.data)
        except json_codec.RuneJsonError as exc:
            self._log_throttled("warn", "gz_hit_json",
                                f"无法解析 /rune/{self.side}/hit JSON（接口定义）: {exc}", period=5.0)
            return
        now = self._sim_now()
        result = self.sm.on_hit(
            event.target,
            ball=event.ball,
            speed=event.speed,
            direct=event.direct,
            stamp=event.stamp,
            now=now,
        )
        self._publish_hit(result, event)
        if result.valid:
            self.get_logger().info(
                f"有效命中: target={result.target} +{result.score} 分 "
                f"(mode={self.sm.mode}, control={self.sm.control_mode}, 累计 {self.sm.points} 分)"
            )
        else:
            self.get_logger().info(
                f"命中无效: target={result.target} reason={result.reason} "
                f"was_pending={result.was_pending}"
            )
        if result.stage_completed:
            self.get_logger().info(
                f"阶段/轮完成! 当前 mode={self.sm.mode} round={self.sm.round} "
                f"auto_advanced={result.advanced}"
            )
        # 灯效/模式立即下发（<100 ms），随后刷新上层状态
        self._sync_outputs()
        self._publish_state()
        self._publish_score()

    def _publish_hit(self, result, event) -> None:
        msg = RuneHit()
        msg.target = int(result.target)
        msg.valid = bool(result.valid)
        msg.score = int(result.score)
        msg.was_pending = bool(result.was_pending)
        msg.ball = str(event.ball)
        msg.speed = float(event.speed)
        msg.stamp = self._to_time(event.stamp) if event.stamp > 0.0 else self.get_clock().now().to_msg()
        self.pub_hit.publish(msg)

    # ------------------------------------------------------------------ 发射额度
    def _on_gimbal_to_vision(self, msg: GimbalToVision) -> None:
        count = int(msg.bullet_count)
        if self._last_bullet_count is None:
            self._last_bullet_count = count     # 首帧只做基线，不计入发射
            return
        if count < self._last_bullet_count:
            self.get_logger().info(
                f"bullet_count 回退 {self._last_bullet_count} -> {count}（按复位处理）")
            self._last_bullet_count = count
            return
        delta = count - self._last_bullet_count
        self._last_bullet_count = count
        if delta <= 0:
            return
        for _ in range(delta):
            self._apply_shot(None, publish_score=False)
        self._publish_score()

    def _apply_shot(self, ball, publish_score: bool = True) -> None:
        result = self.sm.on_shot(ball, now=self._sim_now())
        if result.over_quota:
            self.get_logger().warn(
                f"超额发射: 累计 {result.shots_fired} 发 > 允许额度 {self.sm.ammo_quota}，"
                f"扣 {result.penalty_added} 分（累计扣分 {self.sm.penalty}）"
            )
            if publish_score:
                self._publish_score()

    # ------------------------------------------------------------------ 服务
    def _srv_set_mode(self, request: SetRuneMode.Request, response: SetRuneMode.Response):
        now = self._sim_now()
        a = float(request.a) if request.a and request.a > 0.0 else None
        omega = float(request.omega) if request.omega and request.omega > 0.0 else None
        try:
            advance = self.sm.set_mode(request.mode, a=a, omega=omega, now=now)
        except ValueError as exc:
            response.success = False
            response.message = f"切换失败: {exc}"
            self.get_logger().warn(f"/rune/set_mode 失败: {exc}")
            return response
        self._broadcast_until = now + self.refresh_window
        self._sync_outputs(force=True)
        self._publish_state()
        self._publish_score()
        detail = f"mode={advance.new_mode}, round={advance.round}"
        if advance.a is not None:
            detail += f", a={advance.a:.4f}, omega={advance.omega:.4f}"
        self.get_logger().info(f"/rune/set_mode 生效: {detail}")
        response.success = True
        response.message = f"已切换: {detail}"
        return response

    def _srv_reset(self, request: Trigger.Request, response: Trigger.Response):  # noqa: ARG002
        self.sm.reset(now=self._sim_now())
        self._last_bullet_count = 0
        self._broadcast_until = self._sim_now() + self.refresh_window
        self._sync_outputs(force=True)
        self._publish_state()
        self._publish_score()
        self.get_logger().info("/rune/reset: 已复位到固定靶阶段（记分/灯态清零）")
        response.success = True
        response.message = "已复位到固定靶阶段（记分/灯态清零）"
        return response

    def _srv_next_stage(self, request: Trigger.Request, response: Trigger.Response):  # noqa: ARG002
        try:
            advance = self.sm.advance(now=self._sim_now())
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            return response
        self._broadcast_until = self._sim_now() + self.refresh_window
        self._sync_outputs(force=True)
        self._publish_state()
        self._publish_score()
        detail = f"mode={advance.new_mode}, round={advance.round}"
        if advance.a is not None:
            detail += f", a={advance.a:.4f}, omega={advance.omega:.4f}"
        self.get_logger().info(f"/rune/next_stage 推进: {detail}")
        response.success = True
        response.message = f"已推进: {detail}"
        return response

    def _srv_sim_reset(self, request: Trigger.Request, response: Trigger.Response):  # noqa: ARG002
        self.sm.reset(now=self._sim_now())
        self._last_bullet_count = 0
        self._broadcast_until = self._sim_now() + self.refresh_window
        self._sync_outputs(force=True)
        self._publish_state()
        self._publish_score()
        message = "裁判状态机与记分已复位"
        if self.reset_world:
            ok, detail = self._reset_gazebo_world()
            message += "；" + detail
            if ok:
                self.get_logger().info(detail)
            else:
                self.get_logger().warn(detail)
        response.success = True
        response.message = message
        return response

    def _reset_gazebo_world(self):
        """调用 gz world control 服务复位整个仿真（球位姿/时间/机关角度）。"""
        cmd = [
            "gz", "service",
            "-s", f"/world/{self.world_name}/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", "reset: {all: true}",
        ]
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            return False, "未找到 gz 命令，仅复位了裁判状态机（仿真需手动重启）"
        return True, f"已请求 gz world reset（/world/{self.world_name}/control reset.all=true）"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RuneReferee()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
