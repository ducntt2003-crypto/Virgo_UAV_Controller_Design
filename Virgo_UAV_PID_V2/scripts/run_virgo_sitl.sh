#!/usr/bin/env bash
# ==============================================================================
# Helper Script to Launch PX4 SITL & Gazebo Harmonic for Virgo OMAV V2 (AUV_USDF_V2)
# Framework: PX4 Autopilot v1.15 + Gazebo Harmonic (gz-sim 8.15)
# Paper: MorphQuad (arXiv:2607.02764v4)
# ==============================================================================

set -e

PX4_DIR="/home/duc/PX4/PX4-Autopilot"
export DISPLAY="${DISPLAY:-:0}"
export GZ_SIM_RESOURCE_PATH="/home/duc/Omini_UAV/Virgo_UAV_PID_V2/models:${PX4_DIR}/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}"

echo "[Virgo V2 SITL] Starting PX4 Autopilot v1.15 SITL with Gazebo Harmonic..."
echo "  CAD Model : AUV_USDF_V2 (SolidWorks URDF/SDF)"
echo "  Airframe  : 4027_gz_virgo / 4028_gz_auv_usdf_v2 (CA_AIRFRAME=18, Tilting OMAV)"
echo "  Control   : Null-Space Gimbal-Lock & Downwash Avoidance Allocation"

cd "${PX4_DIR}"

if [[ "$1" == "--gui" || "$1" == "-g" ]]; then
    make px4_sitl gz_virgo
else
    HEADLESS=1 make px4_sitl gz_virgo
fi
