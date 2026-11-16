#!/usr/bin/env bash
# ==============================================================================
# Virgo OMAV SE(3) NMPC - Automated Gazebo Flight Test Launcher
# Launches PX4 SITL + Gazebo Harmonic and runs the 25m Decoupled NMPC Benchmark
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_DIR="/home/duc/PX4/PX4-Autopilot"

GUI="${1:-}"        # Pass --gui or -g to open Gazebo 3D Window

echo -e "\033[1;36m==============================================================================\033[0m"
echo -e "\033[1;36m  VIRGO OMAV SE(3) NMPC - GAZEBO HARMONIC BENCHMARK LAUNCHER\033[0m"
echo -e "\033[1;36m==============================================================================\033[0m"
echo -e "  • Airframe          : \033[1;33mAUV_USDF_V2 (4.56kg, 8 Motors, 8 Servos)\033[0m"
echo -e "  • Bộ điều khiển     : \033[1;33mNonlinear Model Predictive Control (NMPC on SE(3))\033[0m"
echo -e "  • Giao diện Gazebo  : \033[1;33m$([ "$GUI" == "--gui" ] || [ "$GUI" == "-g" ] && echo "Bật 3D GUI Window" || echo "Headless (Không GUI)")\033[0m"
echo -e "\033[1;36m==============================================================================\033[0m\n"

# 1. Dọn dẹp tiến trình cũ nếu có
pkill -9 -f "bin/px4" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gz-sim" 2>/dev/null || true
sleep 1

# 2. Khởi động PX4 SITL + Gazebo Harmonic
echo -e "\033[1;34m[1/3] Đang khởi động PX4 SITL & Gazebo Harmonic...\033[0m"
export DISPLAY="${DISPLAY:-:0}"
export GZ_SIM_RESOURCE_PATH="/home/duc/Omini_UAV/Virgo_UAV_PID_V2/models:${PX4_DIR}/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}"

SITL_LOG="/tmp/px4_sitl_virgo_nmpc.log"
rm -f "${SITL_LOG}"

if [[ "$GUI" == "--gui" || "$GUI" == "-g" ]]; then
    (cd "${PX4_DIR}" && make px4_sitl gz_virgo > "${SITL_LOG}" 2>&1) &
else
    (cd "${PX4_DIR}" && HEADLESS=1 make px4_sitl gz_virgo > "${SITL_LOG}" 2>&1) &
fi
SITL_PID=$!

cleanup() {
    echo -e "\n\033[1;33m[Dọn dẹp] Dừng tiến trình SITL (PID ${SITL_PID})...\033[0m"
    kill "${SITL_PID}" 2>/dev/null || true
    pkill -9 -f "bin/px4" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "gz-sim" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 3. Chờ EKF2 hội tụ và sẵn sàng cất cánh
echo -e "\033[1;34m[2/3] Chờ EKF2 hội tụ cảm biến (Ready for takeoff!)...\033[0m"
for i in {1..35}; do
    if grep -q "Ready for takeoff!" "${SITL_LOG}" 2>/dev/null; then
        echo -e "\033[1;32m  -> EKF2 đã sẵn sàng cất cánh an toàn!\033[0m\n"
        break
    fi
    sleep 1
    echo -n "."
done

if ! grep -q "Ready for takeoff!" "${SITL_LOG}" 2>/dev/null; then
    echo -e "\n\033[1;31m[LỖI] PX4 SITL không đạt trạng thái Ready for takeoff sau 35s. Kiểm tra log tại ${SITL_LOG}\033[0m"
    exit 1
fi

sleep 2

# 4. Thực thi bài bay NMPC
echo -e "\033[1;35m[3/3] Bắt đầu thực thi 25m Decoupled 6-DOF Square Trajectory với NMPC...\033[0m\n"
python3 "${ROOT_DIR}/ros2_px4_bridge/px4_mavlink_nmpc_bridge.py" --side 25.0 --speed 1.5 --alt 1.5

echo -e "\n\033[1;32m==============================================================================\033[0m"
echo -e "\033[1;32m  HOÀN THÀNH BÀI TEST GAZEBO THÀNH CÔNG! Đã lưu đồ thị và telemetry tại Virgo_NMPC_v3/docs\033[0m"
echo -e "\033[1;32m==============================================================================\033[0m\n"
