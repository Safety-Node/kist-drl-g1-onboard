#!/usr/bin/env bash
# run_onboard.sh — launches every onboard ROS 2 node together (for bench / dev).
#
# For production deployment on the Orin NX, safety_monitor and motor_controller
# are run as systemd services instead — see src/{safety_monitor,motor_controller}/systemd/.

set -euo pipefail

ROS_DISTRO=${ROS_DISTRO:-humble}
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${REPO_ROOT}/install/setup.bash"

# Apply CycloneDDS config (partition filtering for /onboard/* topics).
export CYCLONEDDS_URI="file://${REPO_ROOT}/config/cyclonedds.xml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

declare -a PIDS=()

cleanup() {
  echo "[run_onboard.sh] stopping…"
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

ros2 launch sensors           sensors.launch.py           & PIDS+=( $! )
ros2 launch comm_bridge       comm_bridge.launch.py       & PIDS+=( $! )
ros2 launch safety_monitor    safety_monitor.launch.py    & PIDS+=( $! )
ros2 launch motor_controller  motor_controller.launch.py  & PIDS+=( $! )

echo "[run_onboard.sh] launched PIDs: ${PIDS[*]}"
wait
