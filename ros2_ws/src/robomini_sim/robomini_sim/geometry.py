"""场地几何常量与弹道解算（纯 Python，无 ROS 依赖）。

数据来源：ROBOMINI 场地/机关几何（2026-11 版）与补充说明：

* 靶索引固定：``tgt_<team>_<i>`` 按 X-Z 极角升序编号；
* 靶世界位置：``p = axis + (R*cos(theta0 - phi), 0, R*sin(theta0 - phi))``，
  ``phi`` = ``/rune/<team>/state`` 里的 ``hub_angle``（关节角，绕世界 +Y，右手定则）；
* 半径 R=0.35 m，轴心 blue=(0,-0.116,0.928)、red=(0,+0.116,0.928)。

坐标系：世界系右手系，Z 向上，单位 m / rad。
"""

from __future__ import annotations

import math

__all__ = [
    "TEAM_BLUE",
    "TEAM_RED",
    "TEAMS",
    "TARGET_COUNT",
    "TARGET_RADIUS",
    "AXIS_HEIGHT",
    "TARGET_PLATE_SIZE",
    "TARGET_PLATE_THICKNESS",
    "BALL_RADIUS",
    "BALL_MASS",
    "BALL_SPEED_DEFAULT",
    "GRAVITY",
    "YAW_LIMIT",
    "PITCH_MIN",
    "PITCH_MAX",
    "RUNES",
    "check_team",
    "base_angles_rad",
    "wrap_pi",
    "target_angles",
    "target_position",
    "target_positions",
    "target_positions_from_angles",
    "forward_xy",
    "yaw_from_delta",
    "solve_ballistic",
    "muzzle_quaternion",
]

TEAM_BLUE = "blue"
TEAM_RED = "red"
TEAMS = (TEAM_BLUE, TEAM_RED)

TARGET_COUNT = 5          # 每侧 5 个靶
TARGET_RADIUS = 0.350     # m，五靶圆周半径（φ700）
AXIS_HEIGHT = 0.928       # m，机关轴心高度（图 4-9 928.07 mm 归一）
TARGET_PLATE_SIZE = 0.195 # m，灯板方形边长（φ195）
TARGET_PLATE_THICKNESS = 0.022  # m，靶板厚度
BALL_RADIUS = 0.020       # m，乒乓球半径（φ40）
BALL_MASS = 0.0027        # kg
BALL_SPEED_DEFAULT = 12.0 # m/s，枪口缺省速度
GRAVITY = 9.81            # m/s^2（gz 默认重力）

# 云台关节限位（约定）
YAW_LIMIT = math.pi / 2.0            # ±90 deg
PITCH_MIN = math.radians(-10.0)      # -10 deg
PITCH_MAX = math.radians(45.0)       # +45 deg

TABLE_Z = 0.31            # m，云台放置台台面高度

_BASE_ANGLES_DEG = {
    TEAM_BLUE: (-121.504, -49.504, 22.496, 94.496, 166.496),
    TEAM_RED: (-157.504, -85.504, -13.504, 58.496, 130.496),
}

RUNES = {
    TEAM_BLUE: {
        # 能量机关蓝面轴心（靶板中心平面 y）
        "axis": (0.0, -0.116, AXIS_HEIGHT),
        "base_angles_deg": _BASE_ANGLES_DEG[TEAM_BLUE],
        "base_angles_rad": tuple(math.radians(v) for v in _BASE_ANGLES_DEG[TEAM_BLUE]),
        # 云台放置台台面中心
        "launcher_xy": (0.0, -2.07),
        "table_z": TABLE_Z,
        # yaw=0 时枪管指向（世界 XY）
        "forward_xy": (0.0, 1.0),
        # 靶面法向（从靶面指向对应发射架）
        "target_normal": (0.0, -1.0, 0.0),
        # 灯色（约定）
        "light_color": (0.0, 0.25, 1.0),
        # 模型在世界系中的初始偏航（模型系里枪管轴 = +Y；蓝方模型未旋转，红方模型绕 +Z 转 pi）
        "base_yaw_rad": 0.0,
    },
    TEAM_RED: {
        "axis": (0.0, 0.116, AXIS_HEIGHT),
        "base_angles_deg": _BASE_ANGLES_DEG[TEAM_RED],
        "base_angles_rad": tuple(math.radians(v) for v in _BASE_ANGLES_DEG[TEAM_RED]),
        "launcher_xy": (0.0, 2.07),
        "table_z": TABLE_Z,
        "forward_xy": (0.0, -1.0),
        "target_normal": (0.0, 1.0, 0.0),
        "light_color": (1.0, 0.08, 0.05),
        "base_yaw_rad": math.pi,
    },
}


def check_team(team: str) -> str:
    """校验并返回合法队名。"""
    if team not in TEAMS:
        raise ValueError(f"team must be one of {TEAMS}, got {team!r}")
    return team


def base_angles_rad(team: str) -> tuple:
    """该侧 5 个靶的初始 X-Z 极角（rad，升序）。"""
    return RUNES[check_team(team)]["base_angles_rad"]


def wrap_pi(angle: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def target_angles(team: str, hub_angle: float = 0.0) -> list:
    """当前 5 靶的 X-Z 极角（rad）：``theta_i = theta0_i - hub_angle``，已归一到 (-pi, pi]。"""
    check_team(team)
    return [wrap_pi(theta0 - float(hub_angle)) for theta0 in base_angles_rad(team)]


def target_position(team: str, angle: float) -> tuple:
    """给定 X-Z 极角，返回靶板中心世界坐标（y 取轴心 y）。"""
    rune = RUNES[check_team(team)]
    axis = rune["axis"]
    return (
        axis[0] + TARGET_RADIUS * math.cos(angle),
        axis[1],
        axis[2] + TARGET_RADIUS * math.sin(angle),
    )


def target_positions(team: str, hub_angle: float = 0.0) -> list:
    """当前 5 靶靶板中心世界坐标：``[(x, y, z)] * 5``，下标即 tgt_<team>_<i>。"""
    return [target_position(team, theta) for theta in target_angles(team, hub_angle)]


def target_positions_from_angles(team: str, angles) -> list:
    """由已知 X-Z 极角列表直接算 5 个靶中心世界坐标（例如取 /rune/state.target_angles）。"""
    if len(angles) != TARGET_COUNT:
        raise ValueError(f"expected {TARGET_COUNT} angles, got {len(angles)}")
    return [target_position(team, float(a)) for a in angles]


def forward_xy(team: str) -> tuple:
    """yaw=0 时枪管方向的 XY 分量（蓝方 +Y，红方 -Y）。"""
    return RUNES[check_team(team)]["forward_xy"]


def yaw_from_delta(team: str, dx: float, dy: float) -> float:
    """由世界系水平位移 (dx, dy) 求接口 yaw。

    约定（接口约定）：yaw=0 沿本方初始朝向；逆时针（从上方看，即绕 +Z 正转）为正。
    """
    fx, fy = forward_xy(team)
    return math.atan2(fx * dy - fy * dx, fx * dx + fy * dy)


def solve_ballistic(
    team: str,
    muzzle,
    target,
    speed: float = BALL_SPEED_DEFAULT,
    gravity: float = GRAVITY,
) -> tuple:
    """由枪口世界坐标和目标世界坐标解算 yaw/pitch（含重力补偿）。

    物理模型与 gz 插件一致：枪口速度 speed、无空气阻力、重力 -Z。
    求解 ``dz = d*tan(theta) - g*d^2 / (2*v^2*cos^2(theta))``（d 为水平距离），
    在 [-10°, +45°] 内二分（区间内该函数单调递增）。

    :return: ``(yaw, pitch)``，rad，均已限制在云台关节限位内。
    """
    check_team(team)
    if speed <= 0:
        raise ValueError(f"speed must be positive, got {speed}")
    dx = float(target[0]) - float(muzzle[0])
    dy = float(target[1]) - float(muzzle[1])
    dz = float(target[2]) - float(muzzle[2])
    horizontal = math.hypot(dx, dy)
    yaw = yaw_from_delta(team, dx, dy)
    if horizontal < 1e-6:
        pitch = PITCH_MAX if dz > 0 else PITCH_MIN
        return yaw, pitch

    def height_error(theta: float) -> float:
        cos_t = math.cos(theta)
        return horizontal * math.tan(theta) - 0.5 * gravity * horizontal**2 / (speed * speed * cos_t * cos_t) - dz

    lo, hi = PITCH_MIN, PITCH_MAX
    f_lo = height_error(lo)
    f_hi = height_error(hi)
    if f_lo > 0.0:       # 目标比最低射角还低：压到下限
        return yaw, lo
    if f_hi < 0.0:       # +45° 仍够不到：抬到上限
        return yaw, hi
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if height_error(mid) < 0.0:
            lo = mid
        else:
            hi = mid
    return yaw, 0.5 * (lo + hi)


def muzzle_quaternion(team: str, yaw: float, pitch: float) -> tuple:
    """由云台角求枪口/相机朝向四元数 ``(x, y, z, w)``（世界系）。

    旋转 = Rz(本方模型初始偏航 + yaw) * Rx(pitch)，模型系里枪管轴为 +Y。
    即枪管世界方向 = q * (0, 1, 0)：蓝方 yaw=0,pitch=0 -> +Y；红方 -> -Y。
    """
    rune = RUNES[check_team(team)]
    azimuth = rune["base_yaw_rad"] + float(yaw)
    c1, s1 = math.cos(0.5 * azimuth), math.sin(0.5 * azimuth)
    c2, s2 = math.cos(0.5 * float(pitch)), math.sin(0.5 * float(pitch))
    return (c1 * s2, s1 * s2, s1 * c2, c1 * c2)
