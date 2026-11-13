#!/usr/bin/env bash
# 一键构建 Gazebo 系统插件 + ROS 2 工作区。
#
# 用法：
#   scripts/build.sh
#   scripts/build.sh --packages-up-to robomini_sim
set -euo pipefail

SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
JOBS="$(nproc 2>/dev/null || echo 4)"

# 1) Gazebo 插件
echo "[build] gz_plugins: cmake build + install"
cmake -S "$SIM_ROOT/gz_plugins" -B "$SIM_ROOT/gz_plugins/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$SIM_ROOT/gz_plugins/build" -j"$JOBS"
cmake --install "$SIM_ROOT/gz_plugins/build"

# 2) ROS 2 工作区
# ROS 2 setup.bash 在 set -u 下会因未绑定变量报错，临时关闭 nounset
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

cd "$SIM_ROOT/ros2_ws"
echo "[build] colcon build --symlink-install $*"
exec colcon build --symlink-install "$@"
