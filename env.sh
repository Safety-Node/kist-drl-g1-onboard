#!/usr/bin/env bash
# Source this file to activate the onboard ROS 2 workspace with the same
# CycloneDDS config used by run_onboard.sh.
#
#   source env.sh
#
# After sourcing, ros2 CLI commands (topic echo, node list, etc.) will use
# the same DDS participant settings as the running nodes.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced: source env.sh" >&2
  exit 1
fi

_env_sh_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_ros_distro="${ROS_DISTRO:-humble}"
_ros_setup="/opt/ros/${_ros_distro}/setup.bash"
_ws_setup="${_env_sh_dir}/install/setup.bash"
_cyclonedds_xml="${_env_sh_dir}/config/cyclonedds.xml"

if [[ ! -f "${_ros_setup}" ]]; then
  echo "ROS 2 ${_ros_distro} setup not found at ${_ros_setup}" >&2
  unset _env_sh_dir _ros_distro _ros_setup _ws_setup _cyclonedds_xml
  return 1
fi

if [[ ! -f "${_cyclonedds_xml}" ]]; then
  echo "CycloneDDS config not found at ${_cyclonedds_xml}" >&2
  unset _env_sh_dir _ros_distro _ros_setup _ws_setup _cyclonedds_xml
  return 1
fi

source "${_ros_setup}"

if [[ -f "${_ws_setup}" ]]; then
  source "${_ws_setup}"
fi

export CYCLONEDDS_URI="file://${_cyclonedds_xml}"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export DDS_PEER_IP=${DDS_PEER_IP:-192.168.123.222}

echo "Activated ROS ${ROS_DISTRO} with ${RMW_IMPLEMENTATION} (${CYCLONEDDS_URI})"

unset _env_sh_dir _ros_distro _ros_setup _ws_setup _cyclonedds_xml
