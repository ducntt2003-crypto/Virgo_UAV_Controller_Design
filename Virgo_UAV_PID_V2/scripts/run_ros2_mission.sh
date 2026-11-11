#!/usr/bin/env bash
# ==============================================================================
# Helper Script to Launch ROS 2 Virgo Mission Runner (H-Frame)
# ==============================================================================

set -e

WS_DIR="/home/duc/Omini_UAV/Virgo_UAV_PID_V2/ros2_ws"

MISSION="translation"
ALT="1.5"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mission|-m)
            MISSION="$2"
            shift 2
            ;;
        --alt|-a)
            ALT="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: ./run_ros2_mission.sh [--mission translation|contact_inspection|narrow_spaces|visual_inspection|all] [--alt 1.5]"
            exit 1
            ;;
    esac
done

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

echo "[Virgo ROS 2] Thực thi Nhiệm vụ '${MISSION}' tại độ cao ${ALT}m..."
ros2 run virgo_missions mission_node --ros-args -p mission:="${MISSION}" -p target_alt:="${ALT}"
