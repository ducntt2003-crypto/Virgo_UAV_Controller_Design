#!/usr/bin/env bash
# ==============================================================================
# Launch Lie Group SE(3) + L1 Adaptive Controller Node for Virgo OMAV V2
# ==============================================================================

set -e

WS_DIR="/home/duc/Omini_UAV/Virgo_UAV_PID_V2/ros2_ws"
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

ROLL="${1:-25.0}"
PITCH="${2:-15.0}"
HOLD="${3:-10.0}"

echo "[SE3-L1 Controller] Launching SE(3) Tracking & L1 Adaptive Augmentation..."
echo "  Target Attitude : Roll=${ROLL}°, Pitch=${PITCH}°"
echo "  Hold Duration   : ${HOLD}s"
echo "  CAD Airframe    : AUV_USDF_V2"

ros2 run virgo_missions se3_node --ros-args \
    -p roll:="${ROLL}" \
    -p pitch:="${PITCH}" \
    -p hold_sec:="${HOLD}" \
    -p enable_l1:=true
