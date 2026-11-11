#!/usr/bin/env bash
# ==============================================================================
# Virgo OMAV V2 - All-in-One Automated Flight Test Launcher
# Airframe: AUV_USDF_V2 (SolidWorks CAD)
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="/home/duc/PX4/PX4-Autopilot"

MODE="${1:-square}" # square | tilt | straight | se3 | 360
GUI="${2:-}"        # --gui or empty

echo -e "\033[1;36m==============================================================================\033[0m"
echo -e "\033[1;36m  VIRGO OMAV V2 - AUTOMATED TEST LAUNCHER (CAD Model: AUV_USDF_V2)\033[0m"
echo -e "\033[1;36m==============================================================================\033[0m"
echo -e "  • Chế độ bay test : \033[1;33m${MODE}\033[0m (square | tilt | straight | se3 | 360)"
echo -e "  • Giao diện Gazebo: \033[1;33m$([ "$GUI" == "--gui" ] && echo "Bật 3D GUI" || echo "Headless (Không GUI)")\033[0m"
echo -e "\033[1;36m==============================================================================\033[0m\n"

# 1. Dọn dẹp tiến trình cũ nếu có xung đột
pkill -9 -f "bin/px4" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gz-sim" 2>/dev/null || true
sleep 1

# 2. Khởi động PX4 SITL + Gazebo Harmonic
echo -e "\033[1;34m[1/3] Đang khởi động PX4 SITL & Gazebo Harmonic...\033[0m"
export DISPLAY="${DISPLAY:-:0}"
export GZ_SIM_RESOURCE_PATH="/home/duc/Omini_UAV/Virgo_UAV_PID_V2/models:${PX4_DIR}/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}"

SITL_LOG="/tmp/px4_sitl_virgo.log"
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
for i in {1..30}; do
    if grep -q "Ready for takeoff!" "${SITL_LOG}" 2>/dev/null; then
        echo -e "\033[1;32m  -> EKF2 đã sẵn sàng cất cánh an toàn!\033[0m\n"
        break
    fi
    sleep 1
    echo -n "."
done

if ! grep -q "Ready for takeoff!" "${SITL_LOG}" 2>/dev/null; then
    echo -e "\n\033[1;31m[LỖI] PX4 SITL không đạt trạng thái Ready for takeoff sau 30s. Kiểm tra log tại ${SITL_LOG}\033[0m"
    exit 1
fi

sleep 2

# 4. Thực thi kịch bản bay test được chọn
echo -e "\033[1;35m[3/3] Bắt đầu thực thi bài test: ${MODE}...\033[0m\n"

case "${MODE}" in
    square)
        python3 "${SCRIPT_DIR}/test_square_decoupled_6dof.py" --side 25.0 --speed 1.5 --alt 1.5
        ;;
    tilt)
        python3 "${SCRIPT_DIR}/virgo_tilt_hold.py" --roll 20.0 --pitch -15.0 --hold 5.0
        ;;
    straight)
        python3 "${SCRIPT_DIR}/test_straight_x_zero_tilt.py" --target-x 2.0 --alt 1.5 --speed 0.25 --hold 5.0
        ;;
    se3)
        "${SCRIPT_DIR}/run_se3_ros2.sh" 25.0 15.0 10.0
        ;;
    360)
        python3 "${SCRIPT_DIR}/test_omav_rotation_360.py" --mode both
        ;;
    *)
        echo -e "\033[1;31mChế độ không hợp lệ. Vui lòng chọn: square | tilt | straight | se3 | 360\033[0m"
        exit 1
        ;;
esac

echo -e "\n\033[1;32m==============================================================================\033[0m"
echo -e "\033[1;32m  HOÀN THÀNH BÀI TEST THÀNH CÔNG! Đã xuất đồ thị và telemetry CSV.\033[0m"
echo -e "\033[1;32m==============================================================================\033[0m\n"
