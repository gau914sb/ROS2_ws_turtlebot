#!/usr/bin/env bash
# run_case.sh <Case-1b|Case-2|Case-3|Case-4>
#
# One command, zero manual steps: sources the ROS2/Gazebo workspace and runs
# the given case end to end (Gazebo + controller + cameras + logging + plots
# + commit/push). See UBUNTU_GAZEBO_TASK.md section 4(c).
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <Case-1b|Case-2|Case-3|Case-4>"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/install/setup.bash"

exec python3 "$SCRIPT_DIR/run_case.py" "$1"
