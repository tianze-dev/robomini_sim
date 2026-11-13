"""启动 ROBOMINI 自瞄仿真（gz sim + ros_gz_bridge + 裁判/发射架节点 + 可选示例自瞄）。

启动顺序：gz sim server → ros_gz_bridge → rune_referee / launcher_node。

常用：
    # 无头（headless，服务器渲染）
    ros2 launch robomini_sim sim.launch.py
    # 带 gz GUI
    ros2 launch robomini_sim sim.launch.py gui:=true
    # 切红方、自动推进、人工分值
    ros2 launch robomini_sim sim.launch.py side:=red control_mode:=manual

注意：world 内的 model:// 资源通过 GZ_SIM_RESOURCE_PATH 找到；本 launch 会自动把
``<sim_root>/assets/models`` 等加进环境变量。插件 .so 目录会自动搜索
``<sim_root>/gz_plugins/**/libRobomini*.so`` 并加入 ``GZ_SIM_SYSTEM_PLUGIN_PATH``。
"""

import glob
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VALID_SIDES = ("blue", "red")
VALID_RENDER_BACKENDS = ("auto", "gpu", "cpu")
NVIDIA_EGL_JSON = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
VALID_CONTROL_MODES = ("manual", "auto")
VALID_AMMO_MODES = ("unlimited", "counting")


def _as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    return value in ("1", "true", "yes", "on")


def _nvidia_egl_available():
    """检测 NVIDIA headless EGL 是否可用（驱动 + EGL vendor 配置）。"""
    return os.path.exists("/dev/nvidia0") and os.path.exists(NVIDIA_EGL_JSON)


def _find_sim_root():
    """向上搜索包含 worlds/ 与 assets/ 的工程根目录（$SIM）。"""
    env_root = os.environ.get("ROBOMINI_SIM_ROOT") or os.environ.get("SIM_PLATFORM_ROOT")
    if env_root and os.path.isdir(os.path.join(os.path.abspath(env_root), "worlds")):
        return os.path.abspath(env_root)
    path = os.path.dirname(os.path.abspath(__file__))
    for _ in range(12):
        if os.path.isdir(os.path.join(path, "worlds")) and os.path.isdir(os.path.join(path, "assets")):
            return path
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    return None


def _find_plugin_dirs(sim_root):
    """搜索 gz 插件 .so 所在目录（供 GZ_SIM_SYSTEM_PLUGIN_PATH/LD_LIBRARY_PATH）。"""
    dirs = []
    if not sim_root:
        return dirs
    patterns = [
        os.path.join(sim_root, "gz_plugins", "install", "lib", "*.so"),
        os.path.join(sim_root, "gz_plugins", "build", "**", "librobomini*.so"),
        os.path.join(sim_root, "gz_plugins", "build", "**", "libRobomini*.so"),
        os.path.join(sim_root, "gz_plugins", "**", "librobomini*.so"),
        os.path.join(sim_root, "gz_plugins", "**", "libRobomini*.so"),
    ]
    for pattern in patterns:
        for so_path in sorted(glob.glob(pattern, recursive=True)):
            directory = os.path.dirname(so_path)
            if directory not in dirs:
                dirs.append(directory)
    return dirs


def _prepend_path_var(existing, additions):
    parts = [p for p in additions if p]
    if existing:
        parts.append(existing)
    return os.pathsep.join(parts)


def _find_bridge_yaml(sim_root):
    candidates = []
    if sim_root:
        candidates.append(os.path.join(sim_root, "ros2_ws", "src", "robomini_sim", "config", "bridge.yaml"))
    try:
        candidates.append(os.path.join(get_package_share_directory("robomini_sim"), "config", "bridge.yaml"))
    except Exception:  # noqa: BLE001 - 安装环境里取不到时退化为仅源码树
        pass
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    raise RuntimeError(
        "找不到 config/bridge.yaml（已尝试: " + ", ".join(candidates) + "）；"
        "请确认工程目录完整或设置 ROBOMINI_SIM_ROOT"
    )


def _render_bridge_yaml(sim_root, side, world_name, camera_image_topic, camera_info_topic):
    """按 side/world_name 生成 bridge 配置临时副本（ros_gz_bridge 不支持变量替换）。"""
    source = _find_bridge_yaml(sim_root)
    with open(source, "r", encoding="utf-8") as handle:
        text = handle.read()

    text = text.replace("launcher_blue", f"launcher_{side}")
    if side != "blue":
        text = text.replace("/blue/", f"/{side}/")
    if world_name != "robomini":
        text = text.replace("world/robomini", f"world/{world_name}")

    default_image = f"/world/{world_name}/model/launcher_{side}/link/pitch_link/sensor/camera/image"
    default_info = f"/world/{world_name}/model/launcher_{side}/link/pitch_link/sensor/camera/camera_info"
    if camera_image_topic and camera_image_topic != default_image:
        text = text.replace(default_image, camera_image_topic)
    if camera_info_topic and camera_info_topic != default_info:
        text = text.replace(default_info, camera_info_topic)

    out_dir = os.path.join(tempfile.gettempdir(), "robomini_sim")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"bridge_{side}_{world_name}.yaml")
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(text)
    return out_path


def _launch_setup(context, *args, **kwargs):  # noqa: ARG001 - launch 回调签名
    gui = _as_bool(context, "gui")
    side = LaunchConfiguration("side").perform(context).strip()
    world_name = LaunchConfiguration("world_name").perform(context).strip()
    control_mode = LaunchConfiguration("control_mode").perform(context).strip()
    ammo_mode = LaunchConfiguration("ammo_mode").perform(context).strip()
    auto_advance = _as_bool(context, "auto_advance")
    reset_world = _as_bool(context, "reset_world")
    verbosity = LaunchConfiguration("verbosity").perform(context).strip()
    world_arg = LaunchConfiguration("world").perform(context).strip()
    camera_image_topic = LaunchConfiguration("camera_image_topic").perform(context).strip()
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context).strip()
    partition = LaunchConfiguration("partition").perform(context).strip() or "robomini"
    fire_speed = float(LaunchConfiguration("fire_speed").perform(context))
    ammo_quota = int(LaunchConfiguration("ammo_quota").perform(context))

    if side not in VALID_SIDES:
        raise RuntimeError(f"side 必须是 {VALID_SIDES}，收到 {side!r}")
    if control_mode not in VALID_CONTROL_MODES:
        raise RuntimeError(f"control_mode 必须是 {VALID_CONTROL_MODES}，收到 {control_mode!r}")
    if ammo_mode not in VALID_AMMO_MODES:
        raise RuntimeError(f"ammo_mode 必须是 {VALID_AMMO_MODES}，收到 {ammo_mode!r}")

    sim_root = _find_sim_root()
    if world_arg:
        world_path = world_arg
        if not os.path.isabs(world_path) and sim_root:
            world_path = os.path.join(sim_root, world_path)
    elif sim_root:
        world_path = os.path.join(sim_root, "worlds", "robomini.sdf")
    else:
        world_path = os.path.join("worlds", "robomini.sdf")
    world_path = os.path.abspath(world_path)
    if not os.path.isfile(world_path):
        raise RuntimeError(
            f"世界文件不存在: {world_path}\n"
            "请确认 worlds/robomini.sdf 存在，或用 world:=<其他.sdf> 指定。"
        )

    plugin_path_arg = LaunchConfiguration("plugin_path").perform(context).strip()
    plugin_dirs = _find_plugin_dirs(sim_root)
    if plugin_path_arg:
        plugin_dirs = [os.path.abspath(plugin_path_arg)] + [d for d in plugin_dirs if d != os.path.abspath(plugin_path_arg)]
    resource_paths = []
    if sim_root:
        resource_paths = [
            os.path.join(sim_root, "assets", "models"),
            os.path.join(sim_root, "assets"),
            os.path.join(sim_root, "worlds"),
            sim_root,
        ]
    env_actions = [
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            _prepend_path_var(os.environ.get("GZ_SIM_RESOURCE_PATH", ""), resource_paths),
        ),
        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            _prepend_path_var(os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""), plugin_dirs),
        ),
        SetEnvironmentVariable(
            "LD_LIBRARY_PATH",
            _prepend_path_var(os.environ.get("LD_LIBRARY_PATH", ""), plugin_dirs),
        ),
    ]

    # 渲染后端：auto 检测到 NVIDIA GPU 时使用 headless EGL，避免 llvmpipe 软件渲染
    render_backend = LaunchConfiguration("render_backend").perform(context).strip().lower()
    if render_backend not in VALID_RENDER_BACKENDS:
        raise RuntimeError(f"render_backend 必须是 {VALID_RENDER_BACKENDS}，收到 {render_backend!r}")
    use_gpu = render_backend == "gpu" or (render_backend == "auto" and _nvidia_egl_available())
    if use_gpu and not os.path.exists(NVIDIA_EGL_JSON):
        use_gpu = False
        LogInfo(msg=f"[robomini_sim] render_backend=gpu 但缺少 {NVIDIA_EGL_JSON}，回退 CPU 渲染")
    if use_gpu:
        env_actions += [
            SetEnvironmentVariable("__EGL_VENDOR_LIBRARY_FILENAMES", NVIDIA_EGL_JSON),
            SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
        ]

    gz_command = ["gz", "sim", "-r", "-v", verbosity]
    if not gui:
        gz_command += ["-s", "--headless-rendering"]
    extra_gz_args = LaunchConfiguration("extra_gz_args").perform(context).strip()
    if extra_gz_args:
        gz_command += extra_gz_args.split()
    gz_command.append(world_path)

    bridge_yaml = _render_bridge_yaml(sim_root, side, world_name, camera_image_topic, camera_info_topic)

    # gz transport 分区：不同用户/不同仿真实例必须用不同 GZ_PARTITION，否则 /rune/* 等
    # 全局 gz 话题会在多个 gz 进程间互相串扰（比 world 名冲突更隐蔽）。
    env_actions.insert(0, SetEnvironmentVariable("GZ_PARTITION", partition))

    actions = list(env_actions)
    actions += [
        LogInfo(msg=f"[robomini_sim] sim_root={sim_root}"),
        LogInfo(msg=f"[robomini_sim] world={world_path} (name={world_name})"),
        LogInfo(msg=f"[robomini_sim] side={side} gui={gui} auto_advance={auto_advance} "
                    f"control_mode={control_mode} ammo_mode={ammo_mode}"),
        LogInfo(msg=f"[robomini_sim] render_backend={render_backend} "
                    f"-> {'GPU (NVIDIA EGL)' if use_gpu else 'CPU (llvmpipe)'}"),
        LogInfo(msg=f"[robomini_sim] bridge config={bridge_yaml} (GZ_PARTITION={partition})"),
        LogInfo(msg=f"[robomini_sim] plugin dirs={plugin_dirs or '(未发现，若 world 引用插件会加载失败)'}"),
        ExecuteProcess(
            cmd=gz_command,
            name="gz_sim",
            output="screen",
            on_exit=Shutdown(reason="gz sim 退出"),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            parameters=[{"config_file": bridge_yaml}],
        ),
        Node(
            package="robomini_sim",
            executable="rune_referee",
            name="rune_referee",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "side": side,
                "auto_advance": auto_advance,
                "control_mode": control_mode,
                "ammo_mode": ammo_mode,
                "ammo_quota": ammo_quota,
                "world_name": world_name,
                "reset_world": reset_world,
            }],
        ),
        Node(
            package="robomini_sim",
            executable="launcher_node",
            name="launcher_node",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "side": side,
                "default_speed": fire_speed,
            }],
        ),
    ]
    return actions


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="false", description="是否打开 gz GUI（需要 DISPLAY）"),
        DeclareLaunchArgument("render_backend", default_value="auto",
                              description="渲染后端：auto（检测到 NVIDIA 则用 GPU EGL）/ gpu / cpu"),
        DeclareLaunchArgument("side", default_value="blue", description="控制的一方：blue/red"),
        DeclareLaunchArgument("auto_advance", default_value="true", description="完成一阶段后是否自动推进"),
        DeclareLaunchArgument("control_mode", default_value="auto", description="manual/auto 分值档"),
        DeclareLaunchArgument("ammo_mode", default_value="unlimited", description="unlimited/counting 发射额度"),
        DeclareLaunchArgument("ammo_quota", default_value="10", description="counting 模式允许发弹量 A"),
        DeclareLaunchArgument("world", default_value="", description="世界文件路径（默认 <sim_root>/worlds/robomini.sdf）"),
        DeclareLaunchArgument("world_name", default_value="robomini", description="SDF 内 <world name>，用于 gz 服务/话题"),
        DeclareLaunchArgument("fire_speed", default_value="12.0", description="缺省枪口速度 m/s"),
        DeclareLaunchArgument("verbosity", default_value="3", description="gz sim -v 日志级别"),
        DeclareLaunchArgument("extra_gz_args", default_value="", description="附加给 gz sim 的参数（world 之前）"),
        DeclareLaunchArgument("camera_image_topic", default_value="", description="覆盖桥接的相机图像 gz 话题"),
        DeclareLaunchArgument("camera_info_topic", default_value="", description="覆盖桥接的相机 info gz 话题"),
        DeclareLaunchArgument("partition", default_value="robomini",
                              description="gz transport 分区（GZ_PARTITION）：多人/多实例并行时必须各不相同"),
        DeclareLaunchArgument("plugin_path", default_value="",
                              description="指定 gz 插件 .so 目录（默认自动搜索 gz_plugins/install/lib；调试补丁插件用）"),
        DeclareLaunchArgument("reset_world", default_value="true", description="/sim/reset 是否调用 gz world reset 服务"),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])
