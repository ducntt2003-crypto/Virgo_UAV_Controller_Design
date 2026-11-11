#!/usr/bin/env bash
# ==============================================================================
# Helper Script to Launch ROS 2 Virgo Tilt-Hold Node (H-Frame)
# ==============================================================================

set -e

WS_DIR="/home/duc/Omini_UAV/Virgo_UAV_PID_V2/ros2_ws"

ROLL="0.0"
PITCH="0.0"
ALT="1.5"
HOLD="0.0"
INTERACTIVE="false"

if [[ $# -eq 0 ]]; then
    INTERACTIVE="true"
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        --roll|-r)
            ROLL="$2"
            shift 2
            ;;
        --pitch|-p)
            PITCH="$2"
            shift 2
            ;;
        --alt|-a)
            ALT="$2"
            shift 2
            ;;
        --hold|-t)
            HOLD="$2"
            shift 2
            ;;
        --interactive|-i)
            INTERACTIVE="true"
            shift 1
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: ./run_tilt_ros2.sh [--roll <deg>] [--pitch <deg>] [--hold <sec>] [--alt 1.5] [--interactive]"
            exit 1
            ;;
    esac
done

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

echo "[Virgo ROS 2] Khởi chạy Tilt Node..."
echo "  Target Altitude : ${ALT} m"
echo "  Roll            : ${ROLL} deg"
echo "  Pitch           : ${PITCH} deg"
echo "  Hold Duration   : ${HOLD} s"
echo "  Interactive     : ${INTERACTIVE}"

ros2 run virgo_missions tilt_node --ros-args \
    -p roll:="${ROLL}" \
    -p pitch:="${PITCH}" \
    -p target_alt:="${ALT}" \
    -p hold_sec:="${HOLD}" \
    -p interactive:="${INTERACTIVE}"
