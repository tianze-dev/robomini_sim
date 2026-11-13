"""gz 侧 JSON 编解码（接口唯一权威）。

这个模块是 ROS 2 节点与 gz 插件之间所有 JSON 载荷的**唯一**编解码入口。
字段名严格按接口定义，禁止在其它文件里手写 JSON 字段字符串。

gz 话题（经 ros_gz_bridge 以 ``gz.msgs.StringMsg`` <-> ``std_msgs/String`` 桥接）：

======================================  ========  ==========================================================
话题                                    方向      载荷
======================================  ========  ==========================================================
/rune/<team>/mode                       ROS→GZ    ``{"mode","a","omega","t0"}``
/rune/<team>/lights                     ROS→GZ    ``{"targets":["off"|"pending"|"active", x5]}``
/rune/<team>/hit                        GZ→ROS    ``{"target":i,"ball","speed","direct","pos","stamp"}``
/rune/<team>/state                      GZ→ROS    ``{"hub_angle","speed","mode","targets","valid"}``
/launcher/<team>/gimbal                 ROS→GZ    ``{"yaw","pitch","yaw_vel","yaw_acc","pitch_vel","pitch_acc"}``
/launcher/<team>/fire                   ROS→GZ    ``{"speed"}``
/launcher/<team>/state                  GZ→ROS    ``{"yaw","pitch","yaw_vel","pitch_vel","muzzle","balls_fired","ready"}``
======================================  ========  ==========================================================

约定：
* 编解码器不做“字段名兼容”：字段缺失/类型错误会抛 :class:`RuneJsonError`，让插件接口不一致时**立刻暴露**，
  而不是静默失效。
* 所有角度单位 rad、长度 m、时间 s（仿真时间）。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Optional, Sequence

__all__ = [
    "MODES",
    "TARGET_STATE_NAMES",
    "TEAM_NAMES",
    "TARGET_COUNT",
    "GIMBAL_FIELDS",
    "GIMBAL_FEEDFORWARD_FIELDS",
    "TARGET_OFF",
    "TARGET_PENDING",
    "TARGET_ACTIVE",
    "BIG_A_RANGE",
    "BIG_OMEGA_RANGE",
    "RuneJsonError",
    "RuneHitEvent",
    "RunePluginState",
    "GimbalStateEvent",
    "dumps",
    "loads",
    "target_state_name",
    "target_state_index",
    "encode_mode",
    "decode_mode",
    "encode_lights",
    "decode_lights",
    "decode_hit",
    "decode_rune_state",
    "encode_gimbal",
    "decode_gimbal",
    "encode_fire",
    "decode_fire",
    "decode_gimbal_state",
]

#: 四种打靶模式（与接口顺序一致）
MODES = ("fixed", "random_fixed", "small", "big")
#: 灯态枚举，下标即数值（0/1/2）
TARGET_STATE_NAMES = ("off", "pending", "active")
TARGET_OFF, TARGET_PENDING, TARGET_ACTIVE = 0, 1, 2
TEAM_NAMES = ("blue", "red")
TARGET_COUNT = 5

#: 云台 JSON 字段顺序（接口约定）。前馈扩展字段缺省 0（退化为旧接口行为）。
GIMBAL_FIELDS = ("yaw", "pitch", "yaw_vel", "yaw_acc", "pitch_vel", "pitch_acc")
#: ``encode_gimbal`` 里属于前馈扩展的字段（缺省 0，向后兼容旧调用）。
GIMBAL_FEEDFORWARD_FIELDS = ("yaw_vel", "yaw_acc", "pitch_vel", "pitch_acc")

#: 大能量机关参数合法范围（规则 5.5.2 / 规则）
BIG_A_RANGE = (0.780, 1.045)
BIG_OMEGA_RANGE = (1.884, 2.000)


class RuneJsonError(ValueError):
    """JSON 载荷不符合接口定义 时抛出。"""


# ---------------------------------------------------------------------------
# 基础工具
# ---------------------------------------------------------------------------


def dumps(payload: Mapping[str, Any]) -> str:
    """把 dict 编码为紧凑 UTF-8 JSON 字符串（std_msgs/String.data）。"""
    if not isinstance(payload, Mapping):
        raise RuneJsonError(f"JSON payload must be a mapping, got {type(payload).__name__}")
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def loads(text: Any) -> dict:
    """把 std_msgs/String.data（或已解析的 dict）解析成 dict。"""
    if isinstance(text, Mapping):
        return dict(text)
    if isinstance(text, (bytes, bytearray)):
        text = text.decode("utf-8")
    if not isinstance(text, str):
        raise RuneJsonError(f"JSON payload must be str/bytes/dict, got {type(text).__name__}")
    text = text.strip()
    if not text:
        raise RuneJsonError("empty JSON payload")
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise RuneJsonError(f"invalid JSON ({exc}): {text[:200]!r}") from exc
    if not isinstance(data, dict):
        raise RuneJsonError(f"JSON payload must be an object, got {type(data).__name__}")
    return data


def _require(data: Mapping[str, Any], key: str) -> Any:
    if key not in data:
        raise RuneJsonError(f"missing required JSON field '{key}' in {dict(data)!r}")
    return data[key]


def _as_float(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuneJsonError(f"field '{name}' must be a number, got {value!r}")
    return float(value)


def _as_finite_float(value: Any, name: str) -> float:
    """同 :func:`_as_float`，但拒绝 NaN/inf（这类值会生成非法 JSON，必须当场暴露）。"""
    result = _as_float(value, name)
    if not math.isfinite(result):
        raise RuneJsonError(f"field '{name}' must be finite, got {value!r}")
    return result


def _as_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        # 允许 1.0 这类整数值浮点，方便 gz 侧 round-trip
        if isinstance(value, float) and value.is_integer():
            return int(value)
        raise RuneJsonError(f"field '{name}' must be an integer, got {value!r}")
    return int(value)


def _as_bool(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise RuneJsonError(f"field '{name}' must be a bool, got {value!r}")
    return value


def _as_str(value: Any, name: str) -> str:
    if not isinstance(value, str):
        raise RuneJsonError(f"field '{name}' must be a string, got {value!r}")
    return value


def _as_vec3(value: Any, name: str) -> tuple:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise RuneJsonError(f"field '{name}' must be a 3-element array, got {value!r}")
    return tuple(_as_float(v, f"{name}[{i}]") for i, v in enumerate(value))


def _optional(data: Mapping[str, Any], key: str) -> Any:
    return data.get(key)


# ---------------------------------------------------------------------------
# 灯态
# ---------------------------------------------------------------------------


def target_state_name(state: Any) -> str:
    """数值或字符串灯态 -> 接口中的字符串（off/pending/active）。"""
    if isinstance(state, str):
        if state not in TARGET_STATE_NAMES:
            raise RuneJsonError(f"invalid target state {state!r}, expected one of {TARGET_STATE_NAMES}")
        return state
    index = _as_int(state, "target_state")
    if not 0 <= index < len(TARGET_STATE_NAMES):
        raise RuneJsonError(f"invalid target state index {index}, expected 0..2")
    return TARGET_STATE_NAMES[index]


def target_state_index(state: Any) -> int:
    """灯态字符串灯态 -> 数值灯态（0/1/2）。"""
    return TARGET_STATE_NAMES.index(target_state_name(state))


# ---------------------------------------------------------------------------
# /rune/<team>/mode  ROS→GZ
# ---------------------------------------------------------------------------


def encode_mode(
    mode: str,
    a: float | None = None,
    omega: float | None = None,
    t0: float | None = None,
) -> dict:
    """编码 ``/rune/<team>/mode``。

    * ``fixed``/``random_fixed``：插件忽略 a/omega，只带 mode（带 t0 也无害）。
    * ``small``：带 t0（可选）。
    * ``big``：必须带 a、omega、t0；a∈[0.780,1.045]，omega∈[1.884,2.000]。
      插件按 ``spd(t) = a*sin(omega*(t-t0)) + (2.090-a)`` 计算转速；省略 t0 时以收到消息时刻为相位起点。
    """
    if mode not in MODES:
        raise RuneJsonError(f"invalid mode {mode!r}, expected one of {MODES}")
    payload: dict = {"mode": mode}
    if mode == "big":
        if a is None or omega is None:
            raise RuneJsonError("big mode requires both 'a' and 'omega'")
        a = _as_float(a, "a")
        omega = _as_float(omega, "omega")
        if not BIG_A_RANGE[0] <= a <= BIG_A_RANGE[1]:
            raise RuneJsonError(f"'a'={a} outside [{BIG_A_RANGE[0]}, {BIG_A_RANGE[1]}]")
        if not BIG_OMEGA_RANGE[0] <= omega <= BIG_OMEGA_RANGE[1]:
            raise RuneJsonError(f"'omega'={omega} outside [{BIG_OMEGA_RANGE[0]}, {BIG_OMEGA_RANGE[1]}]")
        payload["a"] = a
        payload["omega"] = omega
    if t0 is not None:
        payload["t0"] = _as_float(t0, "t0")
    return payload


def decode_mode(payload: Any) -> dict:
    """解码 ``/rune/<team>/mode``（少量场景需要：独立测试/回放）。"""
    data = loads(payload)
    mode = _as_str(_require(data, "mode"), "mode")
    if mode not in MODES:
        raise RuneJsonError(f"invalid mode {mode!r}")
    result = {"mode": mode}
    if "a" in data:
        result["a"] = _as_float(data["a"], "a")
    if "omega" in data:
        result["omega"] = _as_float(data["omega"], "omega")
    if "t0" in data:
        result["t0"] = _as_float(data["t0"], "t0")
    return result


# ---------------------------------------------------------------------------
# /rune/<team>/lights  ROS→GZ
# ---------------------------------------------------------------------------


def encode_lights(states: Sequence[Any]) -> dict:
    """编码 ``/rune/<team>/lights``：``{"targets": [5 x "off"|"pending"|"active"]}``。"""
    if len(states) != TARGET_COUNT:
        raise RuneJsonError(f"lights requires exactly {TARGET_COUNT} target states, got {len(states)}")
    return {"targets": [target_state_name(s) for s in states]}


def decode_lights(payload: Any) -> list:
    """解码 ``/rune/<team>/lights``（调试/回放用）。"""
    data = loads(payload)
    targets = _require(data, "targets")
    if not isinstance(targets, (list, tuple)) or len(targets) != TARGET_COUNT:
        raise RuneJsonError(f"lights 'targets' must be an array of {TARGET_COUNT} strings")
    return [target_state_name(t) for t in targets]


# ---------------------------------------------------------------------------
# /rune/<team>/hit  GZ→ROS
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RuneHitEvent:
    """插件上报的一次几何命中（未判分）。"""

    target: int
    ball: str = ""
    speed: float = 0.0
    direct: bool = True
    pos: tuple = (0.0, 0.0, 0.0)
    stamp: float = 0.0


def decode_hit(payload: Any) -> RuneHitEvent:
    """解码 ``/rune/<team>/hit``。``direct``/``pos`` 缺失时给默认值，其余字段必须存在。"""
    data = loads(payload)
    target = _as_int(_require(data, "target"), "target")
    if not 0 <= target < TARGET_COUNT:
        raise RuneJsonError(f"hit 'target'={target} out of range 0..{TARGET_COUNT - 1}")
    ball = _as_str(data["ball"], "ball") if _optional(data, "ball") is not None else ""
    speed = _as_float(data["speed"], "speed") if _optional(data, "speed") is not None else 0.0
    direct = _as_bool(data["direct"], "direct") if _optional(data, "direct") is not None else True
    pos = _as_vec3(data["pos"], "pos") if _optional(data, "pos") is not None else (0.0, 0.0, 0.0)
    stamp = _as_float(data["stamp"], "stamp") if _optional(data, "stamp") is not None else 0.0
    return RuneHitEvent(target=target, ball=ball, speed=speed, direct=direct, pos=pos, stamp=stamp)


# ---------------------------------------------------------------------------
# /rune/<team>/state  GZ→ROS
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RunePluginState:
    """插件 20 Hz 上报的机关实际状态。

    ``stamp`` 为可选字段：world 开启 ``<state_include_stamp>true</state_include_stamp>`` 后
    JSON 末尾会带 ``"stamp":<sim 时间秒>``；未开启时为 None。
    """

    hub_angle: float
    speed: float
    mode: str
    targets: tuple
    valid: bool
    stamp: Optional[float] = None


def decode_rune_state(payload: Any) -> RunePluginState:
    """解码 ``/rune/<team>/state``。"""
    data = loads(payload)
    hub_angle = _as_float(_require(data, "hub_angle"), "hub_angle")
    speed = _as_float(_require(data, "speed"), "speed")
    mode = _as_str(data["mode"], "mode") if _optional(data, "mode") is not None else "fixed"
    targets = _require(data, "targets")
    if not isinstance(targets, (list, tuple)) or len(targets) != TARGET_COUNT:
        raise RuneJsonError(f"state 'targets' must be an array of {TARGET_COUNT} strings")
    names = tuple(target_state_name(t) for t in targets)
    valid = _as_bool(data["valid"], "valid") if _optional(data, "valid") is not None else True
    stamp = _as_float(data["stamp"], "stamp") if _optional(data, "stamp") is not None else None
    return RunePluginState(hub_angle=hub_angle, speed=speed, mode=mode, targets=names,
                           valid=valid, stamp=stamp)


# ---------------------------------------------------------------------------
# 发射架
# ---------------------------------------------------------------------------


def encode_gimbal(
    yaw: float,
    pitch: float,
    yaw_vel: float = 0.0,
    yaw_acc: float = 0.0,
    pitch_vel: float = 0.0,
    pitch_acc: float = 0.0,
) -> dict:
    """编码 ``/launcher/<team>/gimbal``（角度 + 可选前馈）。

    六个字段**始终全部存在**：``yaw``/``pitch``（rad）为必填，
    ``yaw_vel``/``yaw_acc``/``pitch_vel``/``pitch_acc``（rad/s、rad/s²）缺省 0，
    即只发角度时退化为旧接口行为。

    非有限值（NaN/inf）会抛 :class:`RuneJsonError`：``json.dumps`` 会把它们写成
    非法 JSON（``NaN``/``Infinity``），gz 侧解析会失败，因此必须当场拦截。
    """
    return {
        "yaw": _as_finite_float(yaw, "yaw"),
        "pitch": _as_finite_float(pitch, "pitch"),
        "yaw_vel": _as_finite_float(yaw_vel, "yaw_vel"),
        "yaw_acc": _as_finite_float(yaw_acc, "yaw_acc"),
        "pitch_vel": _as_finite_float(pitch_vel, "pitch_vel"),
        "pitch_acc": _as_finite_float(pitch_acc, "pitch_acc"),
    }


def decode_gimbal(payload: Any) -> dict:
    """解码 ``/launcher/<team>/gimbal`` -> 含全部六个字段的 dict。

    前馈字段缺失时补 0（兼容旧插件/旧数据）；``yaw``/``pitch`` 必须存在。
    """
    data = loads(payload)
    result = {
        "yaw": _as_float(_require(data, "yaw"), "yaw"),
        "pitch": _as_float(_require(data, "pitch"), "pitch"),
    }
    for key in GIMBAL_FEEDFORWARD_FIELDS:
        value = _optional(data, key)
        result[key] = 0.0 if value is None else _as_float(value, key)
    return result


def encode_fire(speed: float = 12.0) -> dict:
    """编码 ``/launcher/<team>/fire``：``{"speed":m/s}``（插件缺省 12.0）。"""
    return {"speed": _as_float(speed, "speed")}


def decode_fire(payload: Any) -> dict:
    """解码 ``/launcher/<team>/fire`` -> ``{"speed": m/s}``（测试/回放用）。"""
    data = loads(payload)
    return {"speed": _as_float(_require(data, "speed"), "speed")}


@dataclass(frozen=True)
class GimbalStateEvent:
    """插件 20 Hz 上报的发射架实际状态。

    ``yaw_vel``/``pitch_vel`` 为 新增的实测关节角速度（rad/s，插件按关节位置
    差分 + 低通），缺失时默认 0（兼容旧插件/旧 world）。
    ``stamp`` 为可选字段（world 开 ``state_include_stamp`` 时存在），缺失为 None。
    """

    yaw: float
    pitch: float
    muzzle: tuple
    balls_fired: int
    ready: bool
    yaw_vel: float = 0.0
    pitch_vel: float = 0.0
    stamp: Optional[float] = None


def decode_gimbal_state(payload: Any) -> GimbalStateEvent:
    """解码 ``/launcher/<team>/state``（yaw_vel/pitch_vel 为可选字段）。"""
    data = loads(payload)
    yaw = _as_float(_require(data, "yaw"), "yaw")
    pitch = _as_float(_require(data, "pitch"), "pitch")
    muzzle = _as_vec3(_require(data, "muzzle"), "muzzle")
    balls_fired = _as_int(_require(data, "balls_fired"), "balls_fired")
    if balls_fired < 0:
        raise RuneJsonError(f"'balls_fired' must be non-negative, got {balls_fired}")
    ready = _as_bool(_require(data, "ready"), "ready")
    yaw_vel = _as_float(data["yaw_vel"], "yaw_vel") if _optional(data, "yaw_vel") is not None else 0.0
    pitch_vel = (_as_float(data["pitch_vel"], "pitch_vel")
                 if _optional(data, "pitch_vel") is not None else 0.0)
    stamp = _as_float(data["stamp"], "stamp") if _optional(data, "stamp") is not None else None
    return GimbalStateEvent(yaw=yaw, pitch=pitch, muzzle=muzzle, balls_fired=balls_fired,
                              ready=ready, yaw_vel=yaw_vel, pitch_vel=pitch_vel, stamp=stamp)


