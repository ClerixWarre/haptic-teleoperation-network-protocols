#!/bin/bash
set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "Building DCCP Haptic workspace..."
cd "$ROOT_DIR/workspaces/dccp_haptic_ws"
colcon build --symlink-install
source install/setup.bash
echo "DCCP workspace ready!"
