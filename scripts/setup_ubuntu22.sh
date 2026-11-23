#!/usr/bin/env bash
set -euo pipefail

source /etc/os-release
if [[ ${ID:-} != ubuntu || ${VERSION_CODENAME:-} != jammy ]]; then
    echo 'Use Ubuntu 22.04 (Jammy) for this workspace.' >&2
    exit 1
fi

if (( EUID == 0 )); then
    admin=()
else
    admin=(sudo)
fi

export DEBIAN_FRONTEND=noninteractive
apt_command=("${admin[@]}" apt-get -o Acquire::Retries=4)
"${apt_command[@]}" update
"${apt_command[@]}" install -y --no-install-recommends ca-certificates curl software-properties-common
"${admin[@]}" add-apt-repository -y -n universe

if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'ok installed'; then
    setup_tmp=$(mktemp -d)
    trap 'rm -f -- "$setup_tmp/ros2-apt-source.deb"; rmdir -- "$setup_tmp"' EXIT
    ros_source_version=$(curl -fsSL --retry 3 \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
    curl -fsSL --retry 3 -o "$setup_tmp/ros2-apt-source.deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_source_version}/ros2-apt-source_${ros_source_version}.jammy_all.deb"
    "${admin[@]}" dpkg -i "$setup_tmp/ros2-apt-source.deb"
fi

"${apt_command[@]}" update
"${apt_command[@]}" install -y --no-install-recommends \
    python3-colcon-common-extensions python3-yaml python3-opencv \
    ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
    ros-humble-gz-ros2-control ros-humble-diff-drive-controller ros-humble-joint-state-broadcaster \
    ros-humble-nav2-amcl ros-humble-nav2-map-server ros-humble-nav2-bt-navigator \
    ros-humble-nav2-controller ros-humble-nav2-planner ros-humble-nav2-navfn-planner \
    ros-humble-nav2-lifecycle-manager ros-humble-nav2-collision-monitor ros-humble-dwb-core \
    ros-humble-dwb-critics ros-humble-dwb-plugins ros-humble-rviz2 ros-humble-tf2-tools \
    ros-humble-rosbag2-storage-default-plugins
