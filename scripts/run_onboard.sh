#!/usr/bin/env bash
# run_onboard.sh — launches every onboard ROS 2 node together (for bench / dev).
#
# For production deployment on the Orin NX, safety_monitor and motor_controller
# are run as systemd services instead — see src/{safety_monitor,motor_controller}/systemd/.

set -eo pipefail

ROS_DISTRO=${ROS_DISTRO:-humble}
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# ROS / overlay setup references unset vars (AMENT_TRACE_SETUP_FILES)
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${REPO_ROOT}/install/setup.bash"
set -u

# Apply CycloneDDS config (partition filtering for /onboard/* topics).
export CYCLONEDDS_URI="file://${REPO_ROOT}/config/cyclonedds.xml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
# Workstation IP for unicast peer discovery on the bridge domain (domain 1).
export DDS_PEER_IP=${DDS_PEER_IP:-192.168.123.222}

declare -a PIDS=()

# Kill any leftover node processes from a previous run.
pkill -f "uwb_node\|imu_node\|comm_bridge_node\|safety_monitor\|motor_controller" 2>/dev/null || true
sleep 0.5

cleanup() {
  echo "[run_onboard.sh] stopping…"
  for pid in "${PIDS[@]}"; do
    # Kill the launch process and its entire process group
    kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
  done
  # Wait a moment then force-kill any stragglers
  sleep 1
  pkill -f "uwb_node\|imu_node\|comm_bridge_node\|safety_monitor\|motor_controller" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

setsid ros2 launch sensors           sensors.launch.py           & PIDS+=( $! )
setsid ros2 launch comm_bridge       comm_bridge.launch.py       & PIDS+=( $! )
setsid ros2 launch safety_monitor    safety_monitor.launch.py    & PIDS+=( $! )
setsid ros2 launch motor_controller  motor_controller.launch.py  & PIDS+=( $! )

echo "[run_onboard.sh] launched PIDs: ${PIDS[*]}"
wait
