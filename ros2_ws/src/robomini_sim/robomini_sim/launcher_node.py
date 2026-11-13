#!/usr/bin/env python3
"""云台封装节点（Tongji sp_vision_25 同款视觉↔云台协议 -> gz 插件）。

ROS 侧接口（与同济 sp_vision_25 的串口帧字段一一对应，去掉帧头/CRC）::

    /vision_to_gimbal (robomini_interfaces/VisionToGimbal)  ROS -> 云台
        mode, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc
    /gimbal_to_vision (robomini_interfaces/GimbalToVision)  云台 -> ROS
        mode, q[w,x,y,z], yaw, yaw_vel, pitch, pitch_vel, bullet_speed, bullet_count

节点内部把这些字段翻译成 gz 插件的 JSON 话题::

    /launcher/<team>/gimbal {"yaw","pitch","yaw_vel","yaw_acc","pitch_vel","pitch_acc"}
    /launcher/<team>/fire   {"speed"}
    /launcher/<team>/state  {"yaw","pitch","yaw_vel","pitch_vel","muzzle","balls_fired","ready"}

mode 语义:
* 0 不控制（忽略角度与开火）
* 1 控制云台，不开火
* 2 控制云台，且开火（按 ``fire_rate`` 节流，默认 3 发/s）

仿真中 GimbalToVision.q 直接来自 pitch_link 上的 gz IMU（200 Hz，ROS 内部话题
``/sim/launcher_imu``，不属于算法下发接口）；IMU 尚未就绪时才用关节角
q = Rz(yaw)·Rx(pitch) 兜底。bullet_count 取插件累计发球数。
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from robomini_interfaces.msg import GimbalToVision, VisionToGimbal

from . import geometry, json_codec

#: VisionToGimbal.mode
MODE_IDLE = 0
MODE_CONTROL = 1
MODE_CONTROL_AND_FIRE = 2
VALID_MODES = (MODE_IDLE, MODE_CONTROL, MODE_CONTROL_AND_FIRE)


def _quat_wxyz(yaw: float, pitch: float) -> list:
    """q = Rz(yaw)·Rx(pitch)，返回 w,x,y,z。"""
    c1, s1 = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    c2, s2 = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    return [c1 * c2, c1 * s2, s1 * s2, s1 * c2]


class LauncherNode(Node):
    """Tongji 协议字段 <-> gz 插件 JSON 的封装节点。"""

    def __init__(self) -> None:
        super().__init__("launcher_node")

        self.declare_parameter("side", "blue")
        self.declare_parameter("default_speed", 12.0)
        self.declare_parameter("fire_rate", 3.0)            # 发/s，与插件射速上限一致
        self.declare_parameter("clamp_to_limits", True)
        self.declare_parameter("yaw_limit_deg", 90.0)
        self.declare_parameter("pitch_min_deg", -10.0)
        self.declare_parameter("pitch_max_deg", 45.0)
        self.declare_parameter("control_timeout", 1.0)       # 无控制指令后回 idle 的时间
        self.declare_parameter("gimbal_to_vision_rate", 100.0)  # /gimbal_to_vision 发布频率 Hz

        side = str(self.get_parameter("side").value)
        if side not in geometry.TEAMS:
            raise RuntimeError(f"参数 side 必须是 {geometry.TEAMS}，收到 {side!r}")
        self.side = side
        self.default_speed = float(self.get_parameter("default_speed").value)
        self.fire_rate = max(0.1, float(self.get_parameter("fire_rate").value))
        self.clamp_to_limits = bool(self.get_parameter("clamp_to_limits").value)
        self.yaw_limit = math.radians(float(self.get_parameter("yaw_limit_deg").value))
        self.pitch_min = math.radians(float(self.get_parameter("pitch_min_deg").value))
        self.pitch_max = math.radians(float(self.get_parameter("pitch_max_deg").value))
        self.control_timeout = max(0.05, float(self.get_parameter("control_timeout").value))
        g2v_rate = max(1.0, float(self.get_parameter("gimbal_to_vision_rate").value))

        # ROS -> gz（bridge 转发）
        self.pub_gimbal = self.create_publisher(String, f"/launcher/{side}/gimbal", 10)
        self.pub_fire_cmd = self.create_publisher(String, f"/launcher/{side}/fire", 10)
        # gz -> ROS（bridge 转发进来）
        self.create_subscription(String, f"/launcher/{side}/state", self._on_gz_state, 10)
        # 仿真内部 IMU（200 Hz），只用于填充 GimbalToVision.q
        self.create_subscription(Imu, "/sim/launcher_imu", self._on_imu, 50)

        # 视觉 <-> 云台接口
        self.pub_to_vision = self.create_publisher(GimbalToVision, "/gimbal_to_vision", 10)
        self.create_subscription(VisionToGimbal, "/vision_to_gimbal", self._on_vision, 10)

        self._last_fire_time = None
        self._fire_requests = 0
        self._last_control_time = -1e9
        self._last_log_time = {}
        self._imu_q = None          # (w, x, y, z)
        self._imu_time = -1e9
        self._state = None          # 最新一帧云台状态（由 timer 按固定频率发布）

        # /gimbal_to_vision 以固定频率发布（默认 100 Hz），而不是跟随 20 Hz 的云台状态；
        # 姿态角在两次状态更新之间保持不变，四元数取最新 200 Hz IMU 值。
        self._g2v_rate = g2v_rate
        self.create_timer(1.0 / self._g2v_rate, self._publish_gimbal_to_vision)

        self.get_logger().info(
            f"launcher_node 启动: side={side} default_speed={self.default_speed:.2f} m/s "
            f"fire_rate={self.fire_rate:.1f}/s gimbal_to_vision={self._g2v_rate:.0f}Hz "
            f"clamp={self.clamp_to_limits}"
        )

    # ------------------------------------------------------------------ 工具
    def _now(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _log_throttled(self, level: str, key: str, message: str, period: float = 5.0) -> None:
        now = self._now()
        last = self._last_log_time.get(key)
        if last is not None and now >= last and now - last < period:
            return
        self._last_log_time[key] = now
        getattr(self.get_logger(), level)(message)

    # ------------------------------------------------------------------ 视觉 -> 云台
    def _on_vision(self, msg: VisionToGimbal) -> None:
        mode = int(msg.mode)
        if mode not in VALID_MODES:
            self._log_throttled("warn", "vision_mode",
                                f"/vision_to_gimbal 收到非法 mode={mode}（应为 0/1/2），已忽略")
            return
        if mode == MODE_IDLE:
            return

        yaw = float(msg.yaw)
        pitch = float(msg.pitch)
        yaw_vel = float(msg.yaw_vel)
        yaw_acc = float(msg.yaw_acc)
        pitch_vel = float(msg.pitch_vel)
        pitch_acc = float(msg.pitch_acc)
        values = (yaw, pitch, yaw_vel, yaw_acc, pitch_vel, pitch_acc)
        if not all(math.isfinite(v) for v in values):
            self._log_throttled(
                "warn", "nonfinite",
                f"/vision_to_gimbal 收到非有限值 yaw={yaw} pitch={pitch} "
                f"yaw_vel={yaw_vel} yaw_acc={yaw_acc} pitch_vel={pitch_vel} "
                f"pitch_acc={pitch_acc}，已忽略")
            return

        if self.clamp_to_limits:
            clamped_yaw = max(-self.yaw_limit, min(self.yaw_limit, yaw))
            clamped_pitch = max(self.pitch_min, min(self.pitch_max, pitch))
            if abs(clamped_yaw - yaw) > 1e-9 or abs(clamped_pitch - pitch) > 1e-9:
                self._log_throttled(
                    "warn", "clamp",
                    f"云台目标超限已裁剪: yaw {math.degrees(yaw):.2f}->{math.degrees(clamped_yaw):.2f} deg, "
                    f"pitch {math.degrees(pitch):.2f}->{math.degrees(clamped_pitch):.2f} deg")
            yaw, pitch = clamped_yaw, clamped_pitch

        self._last_control_time = self._now()
        payload = json_codec.dumps(
            json_codec.encode_gimbal(yaw, pitch, yaw_vel, yaw_acc, pitch_vel, pitch_acc))
        self.pub_gimbal.publish(String(data=payload))

        if mode == MODE_CONTROL_AND_FIRE:
            self._fire()

    def _fire(self) -> bool:
        """发一次火（按 fire_rate 节流）。"""
        now = self._now()
        if self._last_fire_time is not None and now >= self._last_fire_time:
            interval = now - self._last_fire_time
            min_interval = 1.0 / self.fire_rate
            if interval < min_interval:
                return False
        self._last_fire_time = now
        payload = json_codec.dumps(json_codec.encode_fire(self.default_speed))
        self.pub_fire_cmd.publish(String(data=payload))
        self._fire_requests += 1
        self.get_logger().info(
            f"发射指令已发送 #{self._fire_requests} speed={self.default_speed:.2f} m/s")
        return True

    # ------------------------------------------------------------------ 云台 -> 视觉
    def _on_imu(self, msg: Imu) -> None:
        q = (float(msg.orientation.w), float(msg.orientation.x),
             float(msg.orientation.y), float(msg.orientation.z))
        norm = math.sqrt(sum(v * v for v in q))
        if norm <= 1e-9:
            return
        self._imu_q = tuple(v / norm for v in q)
        self._imu_time = self._now()

    def _on_gz_state(self, msg: String) -> None:
        try:
            self._state = json_codec.decode_gimbal_state(msg.data)
        except json_codec.RuneJsonError as exc:
            self._log_throttled("warn", "gz_state_json",
                                f"无法解析 /launcher/{self.side}/state JSON（接口定义）: {exc}")

    def _publish_gimbal_to_vision(self) -> None:
        state = self._state
        if state is None:
            return
        out = GimbalToVision()
        now = self._now()
        controlling = (now - self._last_control_time) <= self.control_timeout
        out.mode = 1 if controlling else 0
        if self._imu_q is not None and (now - self._imu_time) <= 0.5:
            out.q = list(self._imu_q)
        else:
            out.q = _quat_wxyz(float(state.yaw), float(state.pitch))
        out.yaw = float(state.yaw)
        out.yaw_vel = float(state.yaw_vel)
        out.pitch = float(state.pitch)
        out.pitch_vel = float(state.pitch_vel)
        out.bullet_speed = self.default_speed
        out.bullet_count = int(state.balls_fired) & 0xFFFF
        self.pub_to_vision.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LauncherNode()
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
