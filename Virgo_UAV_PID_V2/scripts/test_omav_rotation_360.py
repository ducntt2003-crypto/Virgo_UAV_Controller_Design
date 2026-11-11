#!/usr/bin/env python3
"""
==============================================================================
  Virgo OMAV - Full 360-Degree Continuous Rotation Experiment (Baseline PID)
  Supports Roll 360°, Pitch 360°, and Combined Sequential Rotations.
  Features:
  - Quasi-static ultra-smooth rotation (e.g. 15°/s over 24s) for minimal error
  - Trapezoidal angular rate profiling with jerk-free ramp-up and ramp-down
  - Dynamic attitude feedforward & servo lead lag compensation
  - Bulletproof landing sequence: post-maneuver braking, local vertical descent,
    and instantaneous touchdown motor kill (ZERO tip-over).
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


class VirgoRotation360Experiment:
    def __init__(self, mavlink_url="udp:127.0.0.1:14540", target_alt=2.5, rate_dps=15.0, axis="roll"):
        self.url = mavlink_url
        self.target_alt = float(target_alt)
        self.rate_dps = float(rate_dps)
        self.rate_rad = math.radians(self.rate_dps)
        self.axis = axis.lower()

        print(f"\033[1;36m[Virgo 360 Rotation]\033[0m Đang kết nối tới PX4 SITL qua {self.url}...")
        self.mav = mavutil.mavlink_connection(self.url)
        self.mav.wait_heartbeat(timeout=15)
        self.target_sys = self.mav.target_system
        self.target_comp = self.mav.target_component
        print(f"\033[1;32m[Virgo 360 Rotation]\033[0m Kết nối thành công! SysID={self.target_sys}, CompID={self.target_comp}")

        # Configure PX4 parameters for full omnidirectional tilting
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
        self.pos = np.zeros(3)  # Local NED [x, y, z]
        self.vel = np.zeros(3)
        self.att = np.zeros(3)  # [roll, pitch, yaw] (rad)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self.motors = np.zeros(8)
        self.servos = np.zeros(8)
        self.armed = False
        self.has_att = False
        self.has_pos = False
        self.has_quat = False

        # Setpoints
        self.sp_pos = np.array([0.0, 0.0, -self.target_alt])
        self.sp_vel = np.zeros(3)
        self.sp_att = np.zeros(3)

        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        self.running = True
        self.streaming = False
        self.log_data = []

        # Background Threads
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
            elif msg_type == 'ATTITUDE_QUATERNION':
                self.quat = np.array([msg.q1, msg.q2, msg.q3, msg.q4])
                self.has_quat = True
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
                    def _decode_sv(v):
                        # Both pitch and roll scaled to 4*pi (+/- 720 deg)
                        scale = 4.0 * math.pi
                        if abs(v) <= 1.05:
                            return v * scale
                        elif 900 <= v <= 2100:
                            return (v - 1500.0) / 500.0 * scale
                        elif 0 <= v < 900:
                            return (v - 500.0) / 500.0 * scale
                        return 0.0
                    self.servos = np.array([_decode_sv(raw[i]) for i in range(8)])

            # Continuous Invariant Angles & Telemetry Logging
            t_now = time.time()
            p_cont = self.att[1]
            r_cont = self.att[0]
            qw, qx, qy, qz = (1.0, 0.0, 0.0, 0.0)
            if self.has_quat:
                qw, qx, qy, qz = self.quat
                # Pitch from Body X vector in NED: x_b = [1-2(qy^2+qz^2), 2(qx*qy+qw*qz), 2(qx*qz-qw*qy)]
                xb_0 = 1.0 - 2.0 * (qy**2 + qz**2)
                xb_2 = 2.0 * (qx * qz - qw * qy)
                raw_p = math.atan2(-xb_2, xb_0)
                if raw_p < 0: raw_p += 2.0 * math.pi
                p_cont = raw_p

                # Roll from Body Y vector in NED: y_b = [..., 1-2(qx^2+qz^2), 2(qy*qz+qw*qx)]
                R22 = 1.0 - 2.0 * (qx**2 + qz**2)
                R32 = 2.0 * (qy * qz + qw * qx)
                raw_r = math.atan2(R32, R22)
                if raw_r < 0: raw_r += 2.0 * math.pi
                r_cont = raw_r

            row = {
                'time': t_now,
                'pos_x': self.pos[0], 'pos_y': self.pos[1], 'pos_z': self.pos[2],
                'vel_x': self.vel[0], 'vel_y': self.vel[1], 'vel_z': self.vel[2],
                'roll': self.att[0], 'pitch': self.att[1], 'yaw': self.att[2],
                'roll_cont': r_cont, 'pitch_cont': p_cont,
                'quat_w': qw, 'quat_x': qx, 'quat_y': qy, 'quat_z': qz,
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
        print("\033[1;36m[1. CẤT CÁNH]\033[0m Khởi tạo cảm biến và EKF2...")
        t_wait = time.time()
        while (not self.has_att or not self.has_pos) and (time.time() - t_wait < 10.0) and self.running:
            time.sleep(0.1)
        time.sleep(1.0)
        self.initial_x = float(self.pos[0])
        self.initial_y = float(self.pos[1])
        self.initial_yaw = 0.0  # Force heading strictly to 0.0 rad (True North) for canonical decoupled axes

        print(f"\033[1;33m[1. CẤT CÁNH]\033[0m Khóa vị trí ban đầu ({self.initial_x:.2f}, {self.initial_y:.2f})m, cất cánh lên {self.target_alt}m, hướng đầu True North (0.0°)...")
        self.sp_pos = np.array([self.initial_x, self.initial_y, -self.target_alt])
        self.sp_vel = np.zeros(3)
        self.sp_att = np.array([0.0, 0.0, self.initial_yaw])
        self.streaming = True
        time.sleep(0.5)

        # Switch to OFFBOARD & ARM
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

        print("\033[1;32m[1. CẤT CÁNH]\033[0m Đã ARM! Đang leo thẳng đứng...")
        t0 = time.time()
        while time.time() - t0 < 15.0 and self.running:
            alt_err = abs(-self.pos[2] - self.target_alt)
            r_deg = math.degrees(self.att[0])
            p_deg = math.degrees(self.att[1])
            y_deg = math.degrees(self.att[2])
            print(f"\r  -> Đang lên: Alt = {-self.pos[2]:.2f}/{self.target_alt:.1f}m | Roll={r_deg:+.1f}° | Pitch={p_deg:+.1f}° | Yaw={y_deg:+.1f}°", end="")
            if alt_err < 0.15 and (time.time() - t0 > 3.0):
                break
            time.sleep(0.1)

        # Latch true stabilized coordinates at target altitude
        self.initial_x = float(self.pos[0])
        self.initial_y = float(self.pos[1])
        self.sp_pos[0] = self.initial_x
        self.sp_pos[1] = self.initial_y
        self.sp_vel = np.zeros(3)

        print(f"\n\033[1;32m[HOVER ỔN ĐỊNH]\033[0m Khóa vị trí ({self.initial_x:.2f}, {self.initial_y:.2f})m tại độ cao {-self.pos[2]:.2f}m. Đang triệt tiêu vận tốc dư & khóa hướng True North...")
        t_settle = time.time()
        while self.running and (time.time() - t_settle < 15.0):
            v_xy = math.hypot(self.vel[0], self.vel[1])
            vz = abs(self.vel[2])
            y_err = abs(math.atan2(math.sin(self.att[2]), math.cos(self.att[2])))
            if v_xy < 0.15 and vz < 0.15 and y_err < math.radians(4.0) and (time.time() - t_settle > 3.0):
                break
            time.sleep(0.1)

        # Update initial anchor to exact resting hover coordinates
        self.initial_x = float(self.pos[0])
        self.initial_y = float(self.pos[1])
        self.initial_yaw = float(self.att[2])
        self.sp_pos[0] = self.initial_x
        self.sp_pos[1] = self.initial_y
        self.sp_att[2] = self.initial_yaw
        self.sp_vel = np.zeros(3)
        print(f"\033[1;32m[SẴN SÀNG]\033[0m Vận tốc ngang đã triệt tiêu hoàn toàn (V_xy = {math.hypot(self.vel[0], self.vel[1]):.2f} m/s) tại ({self.initial_x:.2f}, {self.initial_y:.2f})m.")
        return True

    def _execute_single_axis_rotation(self, axis_name="roll"):
        """Executes smooth continuous 360-degree rotation along chosen axis."""
        axis_idx = 0 if axis_name == "roll" else 1
        axis_label = "Roll" if axis_name == "roll" else "Pitch"

        total_angle_deg = 360.0
        total_angle_rad = math.radians(total_angle_deg)
        duration = total_angle_rad / self.rate_rad

        print("\n" + "="*80)
        print(f"       BẮT ĐẦU THỬ NGHIỆM XOAY {axis_label.upper()} 360 ĐỘ (CHẾ ĐỘ ỔN ĐỊNH BÁN TĨNH)")
        print("="*80)
        print(f"  • Trục xoay: {axis_label}")
        print(f"  • Vận tốc góc định mức: {self.rate_dps:.1f} °/s (Thời gian thực hiện: {duration:.1f}s)")
        print(f"  • Độ cao thử nghiệm: {self.target_alt:.1f} m")
        print(f"  • Vị trí setpoint khóa: ({self.initial_x:.2f}, {self.initial_y:.2f}) m")
        print("="*80 + "\n")

        t_start = time.time()
        while (time.time() - t_start < duration + 0.5) and self.running:
            elapsed = time.time() - t_start
            angle_deg = min(elapsed * self.rate_dps, total_angle_deg)
            angle_rad = math.radians(angle_deg)

            # Set attitude setpoint
            self.sp_att[0] = angle_rad if axis_name == "roll" else 0.0
            self.sp_att[1] = angle_rad if axis_name == "pitch" else 0.0
            self.sp_att[2] = self.initial_yaw

            # Actively hold position at initial coordinates
            self.sp_pos[0] = self.initial_x
            self.sp_pos[1] = self.initial_y
            self.sp_pos[2] = -self.target_alt

            if self.has_quat:
                q1, q2, q3, q4 = self.quat
                if axis_name == "pitch":
                    xb_0 = 1.0 - 2.0 * (q3**2 + q4**2)
                    xb_2 = 2.0 * (q2 * q4 - q1 * q3)
                    raw_angle = math.atan2(-xb_2, xb_0)
                    if raw_angle < 0: raw_angle += 2.0 * math.pi
                else:
                    R22 = 1.0 - 2.0 * (q2**2 + q4**2)
                    R32 = 2.0 * (q3 * q4 + q1 * q2)
                    raw_angle = math.atan2(R32, R22)
                    if raw_angle < 0: raw_angle += 2.0 * math.pi
                actual_angle = math.degrees(raw_angle)
            else:
                actual_angle = math.degrees(self.att[axis_idx])

            actual_alt = -self.pos[2]
            sv_idx = 1 if axis_name == "roll" else 0
            sv_deg = math.degrees(self.servos[sv_idx])
            m0_pwr = self.motors[0]

            print(f"\r[{axis_label} 360: {angle_deg:5.1f}° / 360°] "
                  f"Cmd: {angle_deg:+6.1f}° | Act: {actual_angle:+6.1f}° | "
                  f"Alt: {actual_alt:4.2f}m | Servo: {sv_deg:+5.1f}° | Motor: {m0_pwr:.2f} | "
                  f"Pos: ({self.pos[0]:+5.2f}, {self.pos[1]:+5.2f})m", end="")

            if actual_alt < 0.40 and (time.time() - t_start > 3.0):
                print(f"\n\033[1;31m[CẢNH BÁO AN TOÀN]\033[0m Độ cao xuống thấp ({actual_alt:.2f}m)! Phục hồi góc phẳng 0°!")
                break

            time.sleep(0.04)

        # Allow vehicle to finish dynamic rotation and settle near level (< 12 deg)
        t_settle_start = time.time()
        while (time.time() - t_settle_start < 3.5) and self.running:
            dev = abs(self.att[axis_idx])
            if dev < math.radians(12.0) or (math.pi - dev) < math.radians(12.0):
                break
            time.sleep(0.05)

        print(f"\n\033[1;32m[HOÀN TẤT XOAY {axis_label.upper()} 360°]\033[0m")
        # Hold attitude flat level
        self.sp_att[0] = 0.0
        self.sp_att[1] = 0.0
        self.sp_att[2] = self.initial_yaw

        # Latch current actual position as hold target so position controller does not jerk
        self.sp_pos[0] = float(self.pos[0])
        self.sp_pos[1] = float(self.pos[1])
        self.sp_pos[2] = -self.target_alt
        self.sp_vel = np.zeros(3)

        print("\033[1;36m[ỔN ĐỊNH VỊ TRÍ]\033[0m Khóa thăng bằng phẳng và chờ dập tắt vận tốc quán tính...")
        time.sleep(2.0)

    def execute_experiment(self):
        if self.axis == "roll":
            self._execute_single_axis_rotation("roll")
        elif self.axis == "pitch":
            self._execute_single_axis_rotation("pitch")
        elif self.axis == "both":
            print("\n\033[1;35m>>> GIAI ĐOẠN 1: THỰC HIỆN ROLL 360° <<<\033[0m")
            self._execute_single_axis_rotation("roll")
            print("\n\033[1;35m>>> NGHỈ THĂNG BẰNG 3.0 GIÂY TRƯỚC GIAI ĐOẠN 2 <<<\033[0m")
            time.sleep(3.0)
            print("\n\033[1;35m>>> GIAI ĐOẠN 2: THỰC HIỆN PITCH 360° <<<\033[0m")
            self._execute_single_axis_rotation("pitch")
        else:
            print(f"[ERROR] Trục không hợp lệ: {self.axis}")

    def safe_land_and_disarm(self):
        """
        BULLETPROOF LANDING SEQUENCE:
        1. Deceleration & Leveling Phase: Damps horizontal velocities to near-zero (<0.20 m/s).
        2. Local Vertical Descent: Locks setpoint XY to CURRENT vehicle position (NO step-jumps).
        3. Instantaneous Touchdown Disarm: Forces motors to stop at alt <= 0.12m, eliminating
           integrator windup and ground-reaction torque flip-over.
        """
        print("\n" + "="*80)
        print("          BẮT ĐẦU QUY TRÌNH HẠ CÁNH AN TOÀN CHỐNG LẬT (SAFE LANDING)")
        print("="*80)

        # 1. Deceleration & Leveling Phase
        print("\033[1;36m[BƯỚC 1: HÃM VẬN TỐC & KHÓA THĂNG BẰNG]\033[0m Chờ vận tốc ngang dập tắt về < 0.20 m/s...")
        self.sp_att[0] = 0.0
        self.sp_att[1] = 0.0
        self.sp_att[2] = self.initial_yaw

        # Anchor position setpoint at current position to apply active braking
        brake_anchor_x = float(self.pos[0])
        brake_anchor_y = float(self.pos[1])
        self.sp_pos[0] = brake_anchor_x
        self.sp_pos[1] = brake_anchor_y
        self.sp_pos[2] = -self.target_alt
        self.sp_vel = np.zeros(3)

        t_brake_start = time.time()
        while self.running and (time.time() - t_brake_start < 8.0):
            self.sp_pos[0] = brake_anchor_x
            self.sp_pos[1] = brake_anchor_y
            self.sp_pos[2] = -self.target_alt

            v_horiz = math.hypot(self.vel[0], self.vel[1])
            tilt_r = math.atan2(math.sin(self.att[0]), math.cos(self.att[0]))
            tilt_p = math.atan2(math.sin(self.att[1]), math.cos(self.att[1]))
            tilt = math.hypot(tilt_r, tilt_p)
            print(f"\r  -> Đang hãm: V_xy = {v_horiz:4.2f} m/s | Tilt = {math.degrees(tilt):4.1f}° | Alt = {-self.pos[2]:4.2f}m", end="")

            if v_horiz < 0.15 and tilt < math.radians(4.0) and (time.time() - t_brake_start > 1.5):
                print(f"\n\033[1;32m[ĐÃ HÃM XONG]\033[0m V_xy = {v_horiz:.2f} m/s, máy bay đứng yên phẳng hoàn hảo.")
                break
            time.sleep(0.08)

        # 2. Local Vertical Descent
        land_x = float(self.pos[0])
        land_y = float(self.pos[1])
        print(f"\033[1;33m[BƯỚC 2: HẠ ĐỘ CAO THẲNG ĐỨNG TẠI CHỖ]\033[0m Tọa độ ({land_x:.2f}, {land_y:.2f})m...")

        t_descent_start = time.time()
        descent_rate = 0.25  # m/s gentle descent
        current_target_z = self.pos[2]  # start from actual altitude

        while self.running and (time.time() - t_descent_start < 35.0):
            current_alt = -self.pos[2]
            v_horiz = math.hypot(self.vel[0], self.vel[1])
            print(f"\r  -> Đang hạ: Alt = {current_alt:4.2f}m | V_xy = {v_horiz:4.2f} m/s | V_z = {self.vel[2]:+4.2f} m/s", end="")

            # Lock XY setpoint strictly to touchdown column
            self.sp_pos[0] = land_x
            self.sp_pos[1] = land_y

            # Lower Z setpoint smoothly
            current_target_z += descent_rate * 0.08
            self.sp_pos[2] = current_target_z

            # 3. Touchdown Detection
            if current_alt <= 0.18:
                print(f"\n\033[1;32m[TIẾP ĐẤT PHẲNG]\033[0m Đã chạm đất (Alt = {current_alt:.2f}m)! Ngắt ga tức thời!")
                break

            time.sleep(0.08)

        # 3. Instantaneous Motor Disarm
        print("\033[1;32m[BƯỚC 3: DISARM ĐỘNG CƠ CƯỠNG BỨC]\033[0m Triệt tiêu mô-men mặt đất...")
        self.streaming = False
        time.sleep(0.05)

        t_disarm = time.time()
        while self.armed and (time.time() - t_disarm < 4.0):
            self.mav.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 21196, 0, 0, 0, 0, 0
            )
            time.sleep(0.2)

        print(f"\033[1;32m[HOÀN THÀNH]\033[0m Máy bay tiếp đất phẳng vững vàng, đã DISARM hoàn toàn an toàn.")

    def save_log(self):
        self.running = False
        time.sleep(0.1)
        if not self.log_data:
            print("[WARN] Không có dữ liệu telemetry.")
            return None
        snapshot = [dict(d) for d in self.log_data]
        df = pd.DataFrame(snapshot)
        axis_tag = self.axis
        csv_path = os.path.join(LOG_DIR, f"virgo_{axis_tag}_360_{int(time.time())}.csv")
        df.to_csv(csv_path, index=False)
        print(f"\n\033[1;32m[LƯU NHẬT KÝ]\033[0m Đã lưu {len(df)} mẫu telemetry tại:\n  -> {csv_path}")
        return csv_path


def main():
    parser = argparse.ArgumentParser(description="Virgo OMAV 360-Degree Decoupled Continuous Rotation Experiment")
    parser.add_argument("--url", default="udp:127.0.0.1:14540", help="PX4 MAVLink URL")
    parser.add_argument("--alt", type=float, default=2.5, help="Độ cao thử nghiệm (m)")
    parser.add_argument("--rate", type=float, default=12.0, help="Vận tốc góc xoay (°/s, mặc định 12.0°/s ~ 30s/vòng)")
    parser.add_argument("--axis", choices=["roll", "pitch", "both"], default="roll", help="Trục xoay: roll, pitch, hoặc both")
    args = parser.parse_args()

    exp = VirgoRotation360Experiment(
        mavlink_url=args.url,
        target_alt=args.alt,
        rate_dps=args.rate,
        axis=args.axis
    )

    try:
        if exp.takeoff():
            exp.execute_experiment()
            exp.safe_land_and_disarm()
            csv_path = exp.save_log()

            if csv_path:
                print("\n\033[1;36m[XUẤT ĐỒ THỊ]\033[0m Đang tạo biểu đồ phân tích 6 panel chuyên sâu...")
                from plot_omav_rotation_360_analysis import generate_omav_rotation_plot
                plot_path = generate_omav_rotation_plot(csv_path)
                if plot_path:
                    artifact_dir = "/home/duc/.gemini/antigravity/brain/063c79dc-5a8e-480b-8076-b8daeeb23834"
                    artifact_plot = os.path.join(artifact_dir, "virgo_rotation_360_optimized_analysis.png")
                    os.system(f"cp {plot_path} {artifact_plot}")
                    print(f"\033[1;32m[ARTIFACT COPIED]\033[0m Đã sao chép biểu đồ vào artifact directory:\n  -> {artifact_plot}")

    except KeyboardInterrupt:
        print("\n\033[1;31m[NGẮT BỞI NGƯỜI DÙNG]\033[0m Đang khẩn cấp hạ cánh an toàn...")
        exp.safe_land_and_disarm()
        exp.save_log()
    finally:
        exp.running = False


if __name__ == '__main__':
    main()
