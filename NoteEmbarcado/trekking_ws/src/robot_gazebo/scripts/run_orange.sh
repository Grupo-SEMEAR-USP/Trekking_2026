#!/usr/bin/env bash
set -eo pipefail
ROBOT_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WORKSPACE="${ROBOT_WORKSPACE:-$ROBOT_SCRIPT_DIR}"
while [[ ! -f "$ROBOT_WORKSPACE/src/robot_gazebo/package.xml" && "$ROBOT_WORKSPACE" != / ]]; do
  ROBOT_WORKSPACE="$(dirname -- "$ROBOT_WORKSPACE")"
done
if [[ ! -f "$ROBOT_WORKSPACE/install/setup.bash" ]]; then
  printf '%s\n' 'Workspace não compilado. Execute preparar_linux.sh primeiro.' >&2
  exit 1
fi
ROBOT_DISTRO="${ROS_DISTRO:-jazzy}"
source "/opt/ros/$ROBOT_DISTRO/setup.bash"
if [[ -f "$ROBOT_WORKSPACE/.venv/bin/activate" ]]; then
  source "$ROBOT_WORKSPACE/.venv/bin/activate"
fi
source "$ROBOT_WORKSPACE/install/setup.bash"
ROBOT_MODE="${1:-preview}"
if [[ $# -gt 0 ]]; then shift; fi
exec ros2 launch robot_gazebo orange_robot.launch.py "mode:=$ROBOT_MODE" "$@"
