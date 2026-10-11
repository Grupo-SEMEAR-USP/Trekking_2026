#!/usr/bin/env bash
# Executar no Linux embarcado, com ROS 2 Jazzy previamente instalado.
set -eo pipefail
ROBOT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$ROBOT_ROOT/NoteEmbarcado/trekking_ws"
ROBOT_DISTRO="${ROS_DISTRO:-jazzy}"
if [[ ! -f "/opt/ros/$ROBOT_DISTRO/setup.bash" ]]; then
  printf '%s\n' "Instale ROS 2 $ROBOT_DISTRO antes de continuar." >&2
  exit 1
fi
source "/opt/ros/$ROBOT_DISTRO/setup.bash"
for ROBOT_PROGRAM in git python3 rosdep colcon; do
  command -v "$ROBOT_PROGRAM" >/dev/null || { printf '%s\n' "Comando ausente: $ROBOT_PROGRAM" >&2; exit 1; }
done
if [[ ! -d "$ROBOT_WS/src/sllidar_ros2" ]]; then
  git clone --depth 1 https://github.com/Slamtec/sllidar_ros2.git "$ROBOT_WS/src/sllidar_ros2"
fi
cd "$ROBOT_WS"
rosdep update
rosdep install --from-paths src --ignore-src -r -y
if [[ ! -f .venv/bin/activate ]]; then
  python3 -m venv --system-site-packages .venv
fi
touch .venv/COLCON_IGNORE
source .venv/bin/activate
python -m pip install -r src/robot_gazebo/requirements-vision.txt platformio
python -c 'import rclpy, cv2, numpy, ultralytics, openvino, pyrealsense2, serial'
colcon build --packages-select robot_interfaces robot_gazebo sllidar_ros2 --cmake-clean-cache --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select robot_gazebo
colcon test-result --verbose
pio run -d "$ROBOT_ROOT/Esp32/Esp_Firmware"
printf '%s\n' 'Compilação concluída. Grave o firmware com pio run -t upload --upload-port PORTA na pasta Esp32/Esp_Firmware.'
printf '%s\n' 'Depois execute: bash NoteEmbarcado/trekking_ws/src/robot_gazebo/scripts/run_orange.sh preview'
