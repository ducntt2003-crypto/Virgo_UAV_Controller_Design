#!/usr/bin/env python3
"""
==============================================================================
  Virgo OMAV - 360-Degree Roll Rotation Experiment (Baseline PID)
  Tests full 360-degree Roll body rotation in flight.
  Evaluates decoupled hover limits, servo saturation, and inverted dynamics.
==============================================================================
"""

import argparse
import math
import os
import sys
import threading
import time
import numpy as np
import pandas as pd
from pymavlink import mavutil

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.append(SCRIPT_DIR)
LOG_DIR = os.path.join(SCRIPT_DIR, "../docs/flight_logs")
PLOT_DIR = os.path.join(SCRIPT_DIR, "../docs/flight_plots")
os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(PLOT_DIR, exist_ok=True)


class VirgoRoll360Experiment:
    def __init__(self, mavlink_url="udp:127.0.0.1:14540", target_alt=2.5, roll_rate_dps=30.0, max_angle=360.0):
        self.url = mavlink_url
        self.target_alt = float(target_alt)
        self.roll_rate = math.radians(float(roll_rate_dps))  # rad/s
        self.max_angle = float(max_angle)

        print(f"\033[1;36m[Virgo 360 Roll]\033[0m Đang kết nối tới PX4 SITL qua {self.url}...")
        self.mav = mavutil.mavlink_connection(self.url)
        self.mav.wait_heartbeat(timeout=15)
        self.target_sys = self.mav.target_system
        self.target_comp = self.mav.target_component
        print(f"\033[1;32m[Virgo 360 Roll]\033[0m Kết nối thành công! SysID={self.target_sys}, CompID={self.target_comp}")

        # Set parameter limits for full tilt
        self._set_param("MC_DES_ROLL_MAX", 360.0)
        self._set_param("MC_DES_ROLL_MIN", -360.0)
        self._set_param("MC_DES_PITCH_MAX", 360.0)
        self._set_param("MC_DES_PITCH_MIN", -360.0)
        self._set_param("MC_MAX_FXY", 1.0)
        self._set_param("MPC_THR_MAX", 1.0)
        self._set_param("MC_YAW_WEIGHT", 1.0)
        self._set_param("FD_FAIL_R", 0, is_int=True)
        self._set_param("FD_FAIL_P", 0, is_int=True)

        # High-frequency data streams
        self.mav.mav.request_data_stream_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 25, 1
        )

        # State Telemetry
        self.pos = np.zeros(3)  # Local NED: [x, y, z]
        self.vel = np.zeros(3)
        self.att = np.zeros(3)  # [roll, pitch, yaw] (rad)
        self.motors = np.zeros(8)
        self.servos = np.zeros(8)
        self.armed = False
        self.has_att = False
        self.has_pos = False

        # Setpoints
        self.sp_pos = np.array([0.0, 0.0, -self.target_alt])
        self.sp_vel = np.zeros(3)
        self.sp_acc = np.zeros(3)
        self.sp_att = np.zeros(3)  # [roll, pitch, yaw] (rad)

        self.running = True
        self.streaming = False
        self.log_data = []

        # Threads
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.telemetry_thread.start()

        self.control_thread = threading.Thread(target=self._control_stream_loop, daemon=True)
        self.control_thread.start()

    def _set_param(self, name, val, is_int=False):
        try:
            ptype = mavutil.mavlink.MAV_PARAM_TYPE_INT32 if is_int else mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            self.mav.mav.param_set_send(
                self.target_sys, self.target_comp,
                name.encode('utf-8'), int(val) if is_int else float(val),
                ptype
            )
            time.sleep(0.04)
        except Exception as e:
            print(f"Warning setting param {name}: {e}")

    def _telemetry_loop(self):
        while self.running:
            msg = self.mav.recv_match(blocking=True, timeout=0.1)
            if not msg:
                continue

            msg_type = msg.get_type()
            if msg_type == 'LOCAL_POSITION_NED':
                self.pos = np.array([msg.x, msg.y, msg.z])
                self.vel = np.array([msg.vx, msg.vy, msg.vz])
                self.has_pos = True
            elif msg_type == 'ATTITUDE':
                self.att = np.array([msg.roll, msg.pitch, msg.yaw])
                self.has_att = True
            elif msg_type == 'HEARTBEAT':
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif msg_type == 'SERVO_OUTPUT_RAW':
                port = getattr(msg, 'port', 0)
                raw = [getattr(msg, f'servo{i}_raw') for i in range(1, 9)]
                if port == 0:
                    self.motors = np.array([
                        (v - 100.0) / 1100.0 if (100 <= v <= 1200)
                        else (v / 1000.0 if v <= 1.0 else (v - 1000.0) / 1000.0)
                        for v in raw
                    ])
                elif port == 1:
                    def _decode_sv(v, is_roll=False):
                        scale = 2.0 * math.pi if is_roll else math.pi
                        if abs(v) <= 1.05:
                            return v * scale
                        elif 900 <= v <= 2100:
                            return (v - 1500.0) / 500.0 * scale
                        elif 0 <= v < 900:
                            return (v - 500.0) / 500.0 * scale
                        return 0.0
                    self.servos = np.array([_decode_sv(raw[i], is_roll=(i % 2 == 1)) for i in range(8)])

            # Telemetry Log
            t_now = time.time()
            row = {
                'time': t_now,
                'pos_x': self.pos[0], 'pos_y': self.pos[1], 'pos_z': self.pos[2],
                'vel_x': self.vel[0], 'vel_y': self.vel[1], 'vel_z': self.vel[2],
                'roll': self.att[0], 'pitch': self.att[1], 'yaw': self.att[2],
                'sp_x': self.sp_pos[0], 'sp_y': self.sp_pos[1], 'sp_z': self.sp_pos[2],
                'sp_vx': self.sp_vel[0], 'sp_vy': self.sp_vel[1], 'sp_vz': self.sp_vel[2],
                'sp_roll': self.sp_att[0], 'sp_pitch': self.sp_att[1], 'sp_yaw': self.sp_att[2]
            }
            for i in range(8):
                row[f'motor_{i}'] = self.motors[i]
            for i in range(4):
                row[f'servo_alpha_{i}'] = self.servos[2 * i]
                row[f'servo_beta_{i}'] = self.servos[2 * i + 1]
            self.log_data.append(row)

    def _control_stream_loop(self):
        """Streams Position Target and Decoupled Attitude at 25Hz."""
        # Bitmask: use position (0..2), velocity (3..5), yaw (10). Ignore yaw_rate (11).
        type_mask = 0b0000100000000000
        start_time = time.time()
        dt = 0.04  # 25 Hz

        while self.running:
            if self.streaming:
                ms = int((time.time() - start_time) * 1000) & 0xFFFFFFFF
                self.mav.mav.set_position_target_local_ned_send(
                    ms, self.target_sys, self.target_comp,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask,
                    float(self.sp_pos[0]), float(self.sp_pos[1]), float(self.sp_pos[2]),
                    float(self.sp_vel[0]), float(self.sp_vel[1]), float(self.sp_vel[2]),
                    0.0, 0.0, 0.0,
                    float(self.sp_att[2]), 0.0
                )

                # Send decoupled Euler angles via debug_vect ('tilt_angle')
                t_us = int(time.time() * 1e6) & 0xFFFFFFFFFFFFFFFF
                self.mav.mav.debug_vect_send(
                    b'tilt_angle',
                    t_us,
                    float(self.sp_att[0]),
                    float(self.sp_att[1]),
                    float(self.sp_att[2])
                )

            time.sleep(dt)

    def takeoff(self):
        print("\033[1;36m[1. CẤT CÁNH]\033[0m Chờ đồng bộ cảm biến và hướng EKF2...")
        t_wait = time.time()
        while (not self.has_att or not self.has_pos) and (time.time() - t_wait < 10.0) and self.running:
            time.sleep(0.1)
        time.sleep(1.0)
        self.initial_x = float(self.pos[0])
        self.initial_y = float(self.pos[1])
        self.initial_yaw = float(self.att[2])

        print(f"\033[1;33m[1. CẤT CÁNH]\033[0m Khóa vị trí ({self.initial_x:.2f}, {self.initial_y:.2f})m, cất cánh lên {self.target_alt}m...")
        self.sp_pos = np.array([self.initial_x, self.initial_y, -self.target_alt])
        self.sp_vel = np.zeros(3)
        self.sp_att = np.array([0.0, 0.0, self.initial_yaw])
        self.streaming = True
        time.sleep(0.5)

        # Switch OFFBOARD & ARM
        t_arm_start = time.time()
        while not self.armed and (time.time() - t_arm_start < 20.0) and self.running:
            self.mav.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6, 0, 0, 0, 0, 0
            )
            time.sleep(0.1)
            self.mav.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0
            )
            time.sleep(0.4)

        if not self.armed:
            print("\033[1;31m[LỖI]\033[0m Không thể ARM máy bay!")
            return False

        print("\033[1;32m[1. CẤT CÁNH]\033[0m Đã ARM! Đang bay lên độ cao...")
        t0 = time.time()
        while time.time() - t0 < 12.0 and self.running:
            alt_err = abs(-self.pos[2] - self.target_alt)
            vel_mag = np.linalg.norm(self.vel)
            r_deg = math.degrees(self.att[0])
            p_deg = math.degrees(self.att[1])
            y_deg = math.degrees(self.att[2])
            print(f"\r  -> Đang lên: Alt = {-self.pos[2]:.2f}/{self.target_alt:.1f}m | Roll={r_deg:+.1f}° | Pitch={p_deg:+.1f}° | Yaw={y_deg:+.1f}°", end="")
            if alt_err < 0.15 and vel_mag < 0.2 and (time.time() - t0 > 3.0):
                break
            time.sleep(0.1)

        print(f"\n\033[1;32m[HOVER ỔN ĐỊNH]\033[0m Đạt độ cao {-self.pos[2]:.2f}m. Giữ thăng bằng 2 giây trước khi xoay Roll...")
        time.sleep(2.0)
        return True

    def execute_roll_360(self):
        print("\n" + "="*80)
        print("          BẮT ĐẦU THÍ NGHIỆM QUAY GÓC ROLL 360 ĐỘ (VIRGO OMAV)")
        print("="*80)
        print(f"  • Vận tốc góc Roll: {math.degrees(self.roll_rate):.1f} °/s")
        print(f"  • Góc quay mục tiêu: {self.max_angle:.1f}° (1 vòng hoàn chỉnh)")
        print(f"  • Độ cao thử nghiệm: {self.target_alt:.1f} m")
        print("="*80 + "\n")

        total_angle_rad = math.radians(self.max_angle)
        duration = total_angle_rad / self.roll_rate
        t_start = time.time()

        initial_yaw = float(self.sp_att[2])
        current_roll_cum = 0.0

        while (time.time() - t_start < duration + 0.5) and self.running:
            elapsed = time.time() - t_start
            current_roll_cum = min(elapsed * self.roll_rate, total_angle_rad)
            deg_cum = math.degrees(current_roll_cum)

            # Continuous monotonic rotation without atan2 modulo wrap jump
            roll_cmd = current_roll_cum
            self.sp_att[0] = roll_cmd
            self.sp_att[1] = 0.0
            self.sp_att[2] = initial_yaw

            # Live telemetry display
            actual_roll_deg = math.degrees(self.att[0])
            actual_alt = -self.pos[2]
            sv_beta_deg = math.degrees(self.servos[1])  # Roll servo of pod 0
            m0_pwr = self.motors[0]

            print(f"\r[Roll 360 Progress: {deg_cum:5.1f}° / {self.max_angle:3.0f}°] "
                  f"Cmd Roll: {deg_cum:+6.1f}° | Act Roll: {actual_roll_deg:+6.1f}° | "
                  f"Alt: {actual_alt:4.2f}m | Servo Beta: {sv_beta_deg:+5.1f}° | Motor: {m0_pwr:.2f}", end="")

            # Safety check: if altitude drops too low during inverted phase (< 0.35m), abort to save frame
            if actual_alt < 0.35 and (time.time() - t_start > 3.0):
                print(f"\n\033[1;31m[CẢNH BÁO AN TOÀN]\033[0m Độ cao xuống thấp ({actual_alt:.2f}m)! Phục hồi góc phẳng 0°!")
                break

            time.sleep(0.04)

        print(f"\n\033[1;32m[KẾT THÚC VÒNG ROLL]\033[0m Đã hoàn tất lệnh xoay Roll {deg_cum:.1f}°.")
        print("\033[1;36m[PHỤC HỒI]\033[0m Khóa góc phẳng (Roll=0°, Pitch=0°) và ổn định vị trí hover...")
        self.sp_att[0] = 0.0
        self.sp_att[1] = 0.0
        self.sp_pos[0] = self.initial_x
        self.sp_pos[1] = self.initial_y
        self.sp_pos[2] = -self.target_alt
        time.sleep(3.0)

    def land_and_disarm(self):
        print("\n\033[1;33m[HẠ CÁNH]\033[0m Bắt đầu hạ cánh thẳng đứng...")
        self.sp_att[0] = 0.0
        self.sp_att[1] = 0.0
        t_land = time.time()
        while self.running and (time.time() - t_land < 15.0):
            current_alt = -self.pos[2]
            print(f"\r  -> Đang hạ độ cao: Alt = {current_alt:.2f}m", end="")
            if current_alt > 0.3:
                self.sp_pos[2] += 0.25 * 0.1  # descend at 0.25 m/s
            else:
                self.sp_pos[2] = 0.1  # drive onto ground

            if current_alt < 0.15:
                break
            time.sleep(0.1)

        print("\n\033[1;32m[TIẾP ĐẤT]\033[0m Chạm đất thành công! DISARM động cơ...")
        t_disarm = time.time()
        while self.armed and (time.time() - t_disarm < 5.0):
            self.mav.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 21196, 0, 0, 0, 0, 0
            )
            time.sleep(0.3)
        self.streaming = False
        print("\033[1;32m[HOÀN THÀNH]\033[0m Máy bay đã disarm an toàn.")

    def save_log(self):
        # Stop telemetry thread logging first to avoid race condition
        self.running = False
        time.sleep(0.1)
        if not self.log_data:
            print("[WARN] Không có dữ liệu telemetry.")
            return None
        snapshot = [dict(d) for d in self.log_data]
        df = pd.DataFrame(snapshot)
        csv_path = os.path.join(LOG_DIR, f"virgo_roll_360_{int(time.time())}.csv")
        df.to_csv(csv_path, index=False)
        print(f"\n\033[1;32m[LƯU DỮ LIỆU]\033[0m Đã lưu {len(df)} mẫu telemetry tại:\n  -> {csv_path}")
        return csv_path


def main():
    parser = argparse.ArgumentParser(description="Virgo OMAV 360-Degree Roll Rotation Experiment")
    parser.add_argument("--url", default="udp:127.0.0.1:14540", help="PX4 MAVLink URL")
    parser.add_argument("--alt", type=float, default=2.5, help="Độ cao thử nghiệm (m)")
    parser.add_argument("--rate", type=float, default=36.0, help="Vận tốc góc Roll (°/s)")
    parser.add_argument("--angle", type=float, default=360.0, help="Góc quay tối đa (°)")
    args = parser.parse_args()

    exp = VirgoRoll360Experiment(
        mavlink_url=args.url,
        target_alt=args.alt,
        roll_rate_dps=args.rate,
        max_angle=args.angle
    )

    try:
        if exp.takeoff():
            exp.execute_roll_360()
            exp.land_and_disarm()
            csv_path = exp.save_log()

            # Generate Publication Quality 6-Panel Analysis Plot
            if csv_path:
                print("\n\033[1;36m[XUẤT ĐỒ THỊ]\033[0m Đang tạo đồ thị phân tích 6-panel...")
                from plot_roll_360_analysis import generate_roll_360_plot
                plot_path = generate_roll_360_plot(csv_path)
                if plot_path:
                    artifact_dir = "/home/duc/.gemini/antigravity/brain/063c79dc-5a8e-480b-8076-b8daeeb23834"
                    artifact_plot = os.path.join(artifact_dir, "virgo_roll_360_optimized_analysis.png")
                    os.system(f"cp {plot_path} {artifact_plot}")
                    print(f"\033[1;32m[ARTIFACT COPIED]\033[0m Đã sao chép biểu đồ vào artifact directory:\n  -> {artifact_plot}")

    except KeyboardInterrupt:
        print("\n\033[1;31m[NGẮT BỞI NGƯỜI DÙNG]\033[0m Đang khẩn cấp hạ cánh...")
        exp.sp_att = np.zeros(3)
        exp.land_and_disarm()
        exp.save_log()
    finally:
        exp.running = False


if __name__ == '__main__':
    main()
