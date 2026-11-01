#!/usr/bin/env bash
# Build the ROS2 code. Run on the Jetson, then restart any running nodes:
#
#     ./scripts/build_ros2.sh
set -eo pipefail

REPO=$(cd "$(dirname "$0")/.." && pwd)

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
cd "$REPO/ros2"
colcon build --packages-select irx_driver
echo "Done."
