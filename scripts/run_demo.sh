#!/usr/bin/env bash
set -eo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
scenario=${1:-static}
if (( $# > 0 )); then
    shift
fi
case "$scenario" in
    static|crossing|blocking|detour|scan_loss) ;;
    *) printf 'Unknown scenario: %s\n' "$scenario" >&2; exit 2 ;;
esac

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo 'Install dependencies first with bash scripts/setup_ubuntu22.sh.' >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
if grep -qi microsoft /proc/sys/kernel/osrelease; then
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-53}
    export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
    export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
fi
colcon build --merge-install --base-paths src
source install/local_setup.bash

mkdir -p artifacts
run_dir=$(mktemp -d "$root/artifacts/$scenario-XXXXXX")
log_file="$run_dir.launch.log"
echo "Run output: $run_dir"
status=0
ros2 launch smelter_bringup full_system.launch.py "$@" "scenario:=$scenario" "output_dir:=$run_dir" \
    2>&1 | tee "$log_file" || status=$?
mv -- "$log_file" "$run_dir/launch.log"
if (( status != 0 )); then
    exit "$status"
fi
python3 scripts/check_results.py "$run_dir/metrics.json"
