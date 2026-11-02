#!/usr/bin/env bash
# Quick setup script to link and build sesame_description in ROS 2 Humble workspace (WSL or Linux)
set -e

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "Error: ROS 2 Humble not found in /opt/ros/humble."
  echo "Please install ROS 2 Humble before running this script."
  exit 1
fi

source /opt/ros/humble/setup.bash

echo "=== Creating ~/ros2_ws/src ==="
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=== Linking sesame_description from ${REPO_DIR}/src/sesame_description ==="
ln -sfn "${REPO_DIR}/src/sesame_description" sesame_description

echo "=== Building with colcon ==="
cd ~/ros2_ws
colcon build --symlink-install --packages-select sesame_description

source install/setup.bash

if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
  echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

echo "=== Setup complete! ==="
echo "To view in RViz:   ros2 launch sesame_description vision_nav_rviz.launch.py"
echo "To view in Gazebo: ros2 launch sesame_description gazebo.launch.py"
