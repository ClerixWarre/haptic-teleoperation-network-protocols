#!/bin/bash
set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "Starting QUIC Haptic Teleoperation Experiment..."

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/workspaces/quic_haptic_ws/install/setup.bash"

# Launch the system
ros2 launch quic_haptic_teleoperation quic_haptic_system.launch.py
