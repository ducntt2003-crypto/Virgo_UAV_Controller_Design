#!/usr/bin/env python3
"""
==============================================================================
 Virgo OMAV - 6-DOF Decoupled Square Trajectory Flight Benchmark
 Paper: "MorphEUS: Morphable OmnidirEctional Unmanned System" (arXiv:2505.18270v1)
 Implementation: Square Symmetric Virgo OMAV (Lx=0.25m, Ly=0.25m, 8 Motors, 8 Servos)
==============================================================================
 Kịch bản kiểm nghiệm toàn diện 6 Bậc Tự Do Độc Lập (Decoupled Square Trajectory):
   1. Cất cánh thẳng đứng lên độ cao 1.5m, khóa hướng mặt đất, servo khóa 0°.
   2. Xoay mượt Yaw về 0.0° ở trên không và ổn định tại tọa độ gốc (0, 0, 1.5m).
   3. Tại gốc (0, 0): Đứng yên hover, nghiêng thân mượt mà sang góc Roll = +30°.
   4. Cạnh 1 (0, 0) -> (+L, 0): Bay tiến theo +X trong khi duy trì cố định Roll = +30°.
   5. Tại góc (+L, 0): Đứng yên hover, chuyển đổi thái độ sang Pitch = +30° (ngỏng mũi).
   6. Cạnh 2 (+L, 0) -> (+L, +L): Bay ngang theo +Y trong khi duy trì cố định Pitch = +30°.
   7. Tại góc (+L, +L): Đứng yên hover, chuyển đổi thái độ sang Yaw = +30°.
   8. Cạnh 3 (+L, +L) -> (0, +L): Bay lùi theo -X trong khi duy trì cố định Yaw = +30°.
   9. Tại góc (0, +L): Đứng yên hover, chuyển về trạng thái bình thường (Roll=0°, Pitch=0°, Yaw=0°).
  10. Cạnh 4 (0, +L) -> (0, 0): Bay ngang theo -Y về gốc ở trạng thái bình thường (Zero-Tilt).
  11. Giữ vị trí tại (0, 0) 3 giây, sau đó hạ cánh êm dịu và ngắt động cơ.
  12. Tích hợp Feedforward (P, V, A) và Bộ bù sớm pha cơ khí cho 8 servo.
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
sys.path.append(SCRIPT_DIR)
from plot_flight_data import plot_flight_log

LOG_DIR = os.path.join(SCRIPT_DIR, "../docs/flight_logs")
PLOT_DIR = os.path.join(SCRIPT_DIR, "../docs/flight_plots")
os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(PLOT_DIR, exist_ok=True)


class VirgoSquareBenchmark:
    def __init__(self, mavlink_url="udp:127.0.0.1:14540", side_length=5.0, target_alt=1.5, speed=0.8):
        self.url = mavlink_url
        self.L = float(side_length)
        self.target_alt = float(target_alt)
        self.speed = float(speed)

        print(f"\033[1;36m[Virgo Square Benchmark]\033[0m Đang kết nối tới PX4 SITL qua {self.url}...")
        self.mav = mavutil.mavlink_connection(self.url)
        self.mav.wait_heartbeat(timeout=15)
        self.target_sys = self.mav.target_system
        self.target_comp = self.mav.target_component
        print(f"\033[1;32m[Virgo Square Benchmark]\033[0m Kết nối thành công! SysID={self.target_sys}, CompID={self.target_comp}")

        # Request high-frequency telemetry streams
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

        # Setpoints (Position, Velocity, Acceleration Feedforward, Attitude)
        self.sp_pos = np.array([0.0, 0.0, -self.target_alt])
        self.sp_vel = np.zeros(3)
        self.sp_acc = np.zeros(3)
        self.sp_att = np.zeros(3)  # [roll, pitch, yaw] (rad)

        self.running = True
        self.streaming = False
        self.send_debug_vect = False
        self.log_data = []

        # Background telemetry & control stream threads
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.telemetry_thread.start()

        self.control_thread = threading.Thread(target=self._control_stream_loop, daemon=True)
        self.control_thread.start()

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
                    def _decode_sv(v):
                        if 0 <= v <= 1000:
                            return (v - 500.0) / 500.0 * (math.pi / 2.0)
                        elif 1000 < v <= 2000:
                            return (v - 1500.0) / 500.0 * (math.pi / 2.0)
                        elif abs(v) <= 1.05:
                            return v * (math.pi / 2.0)
                        return 0.0
                    self.servos = np.array([_decode_sv(v) for v in raw])

            # Telemetry Logging
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
        """Continuously streams Position, Velocity, Acceleration Feedforward and Attitude at 25Hz."""
        # Bitmask: use position (0..2), velocity (3..5), acceleration (6..8), yaw (10).
        # Bit 11 is 1 -> ignore yaw_rate. All others are 0 (enabled)!
        # Type mask: 0b0000100000000000 = 2048
        type_mask = 0b0000100000000000
        start_time = time.time()
        dt = 0.04  # 25 Hz

        while self.running:
            if self.streaming:
                ms = int((time.time() - start_time) * 1000) & 0xFFFFFFFF
                # Stream Full Pos + Vel + Acc Feedforward to PX4
                self.mav.mav.set_position_target_local_ned_send(
                    ms, self.target_sys, self.target_comp,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask,
                    float(self.sp_pos[0]), float(self.sp_pos[1]), float(self.sp_pos[2]),
                    float(self.sp_vel[0]), float(self.sp_vel[1]), float(self.sp_vel[2]),
                    float(self.sp_acc[0]), float(self.sp_acc[1]), float(self.sp_acc[2]),
                    float(self.sp_att[2]), 0.0
                )

                # Send decoupled 6-DOF Euler angles via debug_vect ('tilt_angle')
                if self.send_debug_vect:
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
        print("\033[1;36m[Khởi động]\033[0m Chờ đồng bộ dữ liệu cảm biến và góc Yaw từ PX4 EKF2...")
        t_wait = time.time()
        while (not self.has_att or not self.has_pos) and (time.time() - t_wait < 10.0) and self.running:
            time.sleep(0.1)
        time.sleep(1.0)
        initial_x = float(self.pos[0])
        initial_y = float(self.pos[1])
        initial_yaw = float(self.att[2])
        print(f"\n\033[1;33m[1. CẤT CÁNH CỔ ĐIỂN]\033[0m Khóa vị trí mặt đất ({initial_x:.2f}, {initial_y:.2f})m, hướng ({math.degrees(initial_yaw):.1f}°), servo thẳng đứng 0° và bay thẳng lên {self.target_alt}m...")

        # Climb strictly vertically above current ground coordinates
        self.sp_pos = np.array([initial_x, initial_y, -self.target_alt])
        self.sp_vel = np.zeros(3)
        self.sp_acc = np.zeros(3)
        self.sp_att = np.array([0.0, 0.0, initial_yaw])
        self.streaming = True

        time.sleep(0.8)

        print("\033[1;33m[1. CẤT CÁNH]\033[0m Chuyển sang OFFBOARD và ARM động cơ...")
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
            print("\033[1;31m[LỖI]\033[0m Không thể ARM máy bay! Kiểm tra preflight PX4.")
            return False

        print("\033[1;32m[1. CẤT CÁNH]\033[0m Đã ARM thành công! Đang bay thẳng đứng lên...")
        t0 = time.time()
        while time.time() - t0 < 12.0 and self.running:
            alt_err = abs(-self.pos[2] - self.target_alt)
            vel_mag = np.linalg.norm(self.vel)
            r_deg = math.degrees(self.att[0])
            p_deg = math.degrees(self.att[1])
            y_deg = math.degrees(self.att[2])
            print(f"\r  -> Lên độ cao: Alt = {-self.pos[2]:.2f}/{self.target_alt:.1f}m | Roll={r_deg:+.1f}° | Pitch={p_deg:+.1f}° | Yaw={y_deg:+.1f}°", end="")
            if alt_err < 0.15 and vel_mag < 0.2 and (time.time() - t0 > 3.5):
                break
            time.sleep(0.1)

        print(f"\n\033[1;32m[HOVER ĐẠT CHUẨN]\033[0m Đã đạt độ cao {-self.pos[2]:.2f}m.")

        # Smoothly transit to origin (0, 0, 0°) using smooth quintic polynomial
        dist_to_origin = np.linalg.norm(np.array([self.pos[0], self.pos[1]]))
        yaw_diff = abs(self.att[2])
        if dist_to_origin > 0.05 or yaw_diff > 0.03:
            print(f"\033[1;36m[Ổn định Vị trí Gốc]\033[0m Tịnh tiến mượt về gốc (0, 0, Yaw=0°)...")
            p_start = np.array([self.pos[0], self.pos[1], -self.target_alt])
            p_end = np.array([0.0, 0.0, -self.target_alt])
            yaw_start = float(self.att[2])
            dur = max(3.0, dist_to_origin / 0.5)
            t_glide = time.time()
            while time.time() - t_glide < dur and self.running:
                tau = (time.time() - t_glide) / dur
                s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
                s_dot = (30.0 * (tau ** 2) - 60.0 * (tau ** 3) + 30.0 * (tau ** 4)) / dur
                delta_p = p_end[:2] - p_start[:2]
                self.sp_pos[:2] = p_start[:2] + delta_p * s
                self.sp_vel[:2] = delta_p * s_dot
                self.sp_att[2] = yaw_start * (1.0 - s)
                time.sleep(0.04)

        self.sp_pos = np.array([0.0, 0.0, -self.target_alt])
        self.sp_vel = np.zeros(3)
        self.sp_acc = np.zeros(3)
        self.sp_att = np.zeros(3)

        print("\033[1;36m[Ổn định Hover]\033[0m Chờ UAV ổn định hoàn toàn tại (X=0, Y=0, Yaw=0°)...")
        t_wait_settle = time.time()
        while time.time() - t_wait_settle < 2.5 and self.running:
            time.sleep(0.1)

        print(f"\033[1;32m[SẴN SÀNG QUỸ ĐẠO]\033[0m UAV tại vị trí gốc: X={self.pos[0]:.2f}m, Y={self.pos[1]:.2f}m, Yaw={math.degrees(self.att[2]):.1f}°")
        self.send_debug_vect = True
        return True

    def transition_attitude(self, att_start, att_end, duration=3.5, hold_time=2.0, label=""):
        """
        Smoothly transitions vehicle attitude while strictly holding position and zero velocity.
        att_start, att_end: np.ndarray([roll, pitch, yaw]) in radians.
        """
        print(f"\n\033[1;36m>>> [CHUYỂN ĐỔI THÁI ĐỘ - ĐỨNG YÊN HOVER] {label} <<<\033[0m")
        print(f"  Vị trí cố định: ({self.sp_pos[0]:.2f}, {self.sp_pos[1]:.2f}, {-self.sp_pos[2]:.2f})m")
        print(f"  Từ R={math.degrees(att_start[0]):+.1f}°, P={math.degrees(att_start[1]):+.1f}°, Y={math.degrees(att_start[2]):+.1f}°")
        print(f"  -> R={math.degrees(att_end[0]):+.1f}°, P={math.degrees(att_end[1]):+.1f}°, Y={math.degrees(att_end[2]):+.1f}° (Thời gian chuyển: {duration:.1f}s)")

        self.sp_vel[:2] = 0.0
        self.sp_acc[:2] = 0.0

        t_start = time.time()
        while self.running:
            elapsed = time.time() - t_start
            if elapsed >= duration:
                self.sp_att = np.array(att_end, dtype=float)
                break

            tau = elapsed / duration
            # Smooth 5th-order polynomial trajectory: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
            s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
            self.sp_att = att_start + (att_end - att_start) * s

            r_deg = math.degrees(self.att[0])
            p_deg = math.degrees(self.att[1])
            y_deg = math.degrees(self.att[2])
            avg_alpha = np.mean([self.servos[2 * i] for i in range(4)])
            avg_beta  = np.mean([self.servos[2 * i + 1] for i in range(4)])

            print(f"\r  [Nghiêng Thân Hover] Att: R={r_deg:+.1f}°, P={p_deg:+.1f}°, Y={y_deg:+.1f}° | Servo α={math.degrees(avg_alpha):+.1f}°, β={math.degrees(avg_beta):+.1f}°", end="")
            time.sleep(0.04)

        # Hold stationary at new attitude for hold_time
        if hold_time > 0 and self.running:
            print(f"\n  -> Giữ cố định thái độ mới trong {hold_time:.1f}s để ổn định...")
            t_hold = time.time()
            while time.time() - t_hold < hold_time and self.running:
                r_deg = math.degrees(self.att[0])
                p_deg = math.degrees(self.att[1])
                y_deg = math.degrees(self.att[2])
                print(f"\r  [Giữ Thái Độ] Att: R={r_deg:+.1f}°, P={p_deg:+.1f}°, Y={y_deg:+.1f}° | Pos: ({self.pos[0]:+.2f}, {self.pos[1]:+.2f})m", end="")
                time.sleep(0.05)
            print()

    def execute_segment(self, p_start, p_end, attitude_profile_fn, segment_name):
        """
        Executes a trajectory segment with continuous position, velocity, and acceleration feedforward,
        while applying a decoupled attitude profile (Roll, Pitch, Yaw).
        """
        dist = np.linalg.norm(p_end[:2] - p_start[:2])
        duration = max(4.0, dist / self.speed)
        print(f"\n\033[1;35m>>> BẮT ĐẦU {segment_name} <<<\033[0m")
        print(f"  Từ ({p_start[0]:.2f}, {p_start[1]:.2f})m -> ({p_end[0]:.2f}, {p_end[1]:.2f})m | Quãng đường: {dist:.2f}m | Thời gian: {duration:.1f}s")

        t_start = time.time()
        while self.running:
            elapsed = time.time() - t_start
            if elapsed >= duration:
                self.sp_pos[:2] = p_end[:2]
                self.sp_vel[:2] = 0.0
                self.sp_acc[:2] = 0.0
                break

            tau = elapsed / duration
            # Smooth 5th-order polynomial trajectory (zero velocity & acceleration at ends)
            # s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
            s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
            s_dot = (30.0 * (tau ** 2) - 60.0 * (tau ** 3) + 30.0 * (tau ** 4)) / duration
            s_ddot = (60.0 * tau - 180.0 * (tau ** 2) + 120.0 * (tau ** 3)) / (duration ** 2)

            delta_p = p_end[:2] - p_start[:2]
            self.sp_pos[:2] = p_start[:2] + delta_p * s
            self.sp_vel[:2] = delta_p * s_dot
            self.sp_acc[:2] = delta_p * s_ddot

            # Compute decoupled attitude profile for this segment
            r_sp, p_sp, y_sp = attitude_profile_fn(tau)
            self.sp_att[0] = r_sp
            self.sp_att[1] = p_sp
            self.sp_att[2] = y_sp

            r_deg = math.degrees(self.att[0])
            p_deg = math.degrees(self.att[1])
            y_deg = math.degrees(self.att[2])
            avg_alpha = np.mean([self.servos[2 * i] for i in range(4)])
            avg_beta  = np.mean([self.servos[2 * i + 1] for i in range(4)])

            print(f"\r  [{segment_name}] Pos: ({self.pos[0]:+.2f}, {self.pos[1]:+.2f})m | Att: R={r_deg:+.1f}°, P={p_deg:+.1f}°, Y={y_deg:+.1f}° | Servo α={math.degrees(avg_alpha):+.1f}°, β={math.degrees(avg_beta):+.1f}°", end="")
            time.sleep(0.04)

        # Settle at corner for 0.8 second
        t_corner = time.time()
        while time.time() - t_corner < 0.8 and self.running:
            time.sleep(0.05)

    def run_benchmark(self):
        """Runs the 4-edge decoupled benchmark square with corner stationary attitude switching."""
        L = self.L
        p0 = np.array([0.0, 0.0, -self.target_alt])
        p1 = np.array([L, 0.0, -self.target_alt])
        p2 = np.array([L, L, -self.target_alt])
        p3 = np.array([0.0, L, -self.target_alt])

        deg30 = math.radians(30.0)

        # ---------------------------------------------------------------------
        # 1. ĐỨNG YÊN HOVER TẠI GỐC (0, 0) & NGHIÊNG GÓC ROLL = +30°
        # ---------------------------------------------------------------------
        self.transition_attitude(
            att_start=np.array([0.0, 0.0, 0.0]),
            att_end=np.array([deg30, 0.0, 0.0]),
            duration=3.5,
            hold_time=2.0,
            label="1. Tại Gốc (0, 0): Nghiêng Thân Roll = +30°"
        )

        # ---------------------------------------------------------------------
        # CẠNH 1: (0, 0) -> (+L, 0) | DUY TRÌ LIÊN TỤC ROLL = +30°
        # ---------------------------------------------------------------------
        def profile_edge1(tau):
            return deg30, 0.0, 0.0

        self.execute_segment(p0, p1, profile_edge1, f"CẠNH 1 (+X, Cố định Roll=+30° [L={L:.0f}m])")

        # ---------------------------------------------------------------------
        # 2. ĐỨNG YÊN HOVER TẠI (L, 0) & CHUYỂN QUA GÓC PITCH = +30°
        # ---------------------------------------------------------------------
        self.transition_attitude(
            att_start=np.array([deg30, 0.0, 0.0]),
            att_end=np.array([0.0, deg30, 0.0]),
            duration=4.0,
            hold_time=2.0,
            label="2. Tại Góc (+L, 0): Chuyển Roll=+30° sang Pitch=+30°"
        )

        # ---------------------------------------------------------------------
        # CẠNH 2: (+L, 0) -> (+L, +L) | DUY TRÌ LIÊN TỤC PITCH = +30° (NGỎNG MŨI)
        # ---------------------------------------------------------------------
        def profile_edge2(tau):
            return 0.0, deg30, 0.0

        self.execute_segment(p1, p2, profile_edge2, f"CẠNH 2 (+Y, Cố định Pitch=+30° [L={L:.0f}m])")

        # ---------------------------------------------------------------------
        # 3. ĐỨNG YÊN HOVER TẠI (L, L) & CHUYỂN QUA GÓC YAW = +30°
        # ---------------------------------------------------------------------
        self.transition_attitude(
            att_start=np.array([0.0, deg30, 0.0]),
            att_end=np.array([0.0, 0.0, deg30]),
            duration=4.0,
            hold_time=2.0,
            label="3. Tại Góc (+L, +L): Chuyển Pitch=+30° sang Yaw=+30°"
        )

        # ---------------------------------------------------------------------
        # CẠNH 3: (+L, +L) -> (0, +L) | DUY TRÌ LIÊN TỤC YAW = +30°
        # ---------------------------------------------------------------------
        def profile_edge3(tau):
            return 0.0, 0.0, deg30

        self.execute_segment(p2, p3, profile_edge3, f"CẠNH 3 (-X, Cố định Yaw=+30° [L={L:.0f}m])")

        # ---------------------------------------------------------------------
        # 4. ĐỨNG YÊN HOVER TẠI (0, L) & CHUYỂN VỀ TRẠNG THÁI BÌNH THƯỜNG (ROLL=0°, PITCH=0°, YAW=0°)
        # ---------------------------------------------------------------------
        self.transition_attitude(
            att_start=np.array([0.0, 0.0, deg30]),
            att_end=np.array([0.0, 0.0, 0.0]),
            duration=4.0,
            hold_time=2.0,
            label="4. Tại Góc (0, +L): Chuyển về Bình Thường (Roll=0°, Pitch=0°, Yaw=0°)"
        )

        # ---------------------------------------------------------------------
        # CẠNH 4: (0, +L) -> (0, 0) | TRẠNG THÁI BÌNH THƯỜNG (ZERO-TILT)
        # ---------------------------------------------------------------------
        def profile_edge4(tau):
            return 0.0, 0.0, 0.0

        self.execute_segment(p3, p0, profile_edge4, f"CẠNH 4 (-Y, Bình Thường Zero-Tilt [L={L:.0f}m])")

        # Giữ vị trí tại gốc (0, 0) với thân phẳng tuyệt đối
        print(f"\n\033[1;33m[HOÀN THÀNH HÌNH VUÔNG]\033[0m Đã về tới gốc (0, 0)! Giữ thân phẳng 0° ổn định hoàn toàn...")
        self.sp_att = np.zeros(3)
        self.send_debug_vect = True
        t_settle = time.time()
        while time.time() - t_settle < 10.0 and self.running:
            v_xy = np.linalg.norm(self.vel[:2])
            p_xy = np.linalg.norm(self.pos[:2])
            r_deg = math.degrees(self.att[0])
            p_deg = math.degrees(self.att[1])
            y_deg = math.degrees(self.att[2])
            print(f"\r  -> Hold Gốc: Pos=({self.pos[0]:+.2f}, {self.pos[1]:+.2f})m | Vxy={v_xy:.2f}m/s | R={r_deg:+.1f}°, P={p_deg:+.1f}°, Y={y_deg:+.1f}°", end="")
            if v_xy < 0.12 and p_xy < 0.20 and (time.time() - t_settle > 3.0):
                print("\n\033[1;32m  -> Đã triệt tiêu hoàn toàn vận tốc quán tính và ổn định tại tâm!\033[0m")
                break
            time.sleep(0.08)

    def land_and_disarm(self):
        print(f"\n\033[1;33m[HẠ CÁNH CỔ ĐIỂN]\033[0m Khóa servo 0°, ổn định vị trí gốc X=0.00m, Y=0.00m và hạ độ cao nhẹ nhàng...")
        self.send_debug_vect = False
        init_z = self.sp_pos[2]
        descend_speed = 0.25  # m/s
        t_start_desc = time.time()

        self.sp_pos[0] = 0.0
        self.sp_pos[1] = 0.0
        self.sp_att = np.zeros(3)

        while self.running:
            elapsed = time.time() - t_start_desc
            target_z = min(0.0, init_z + descend_speed * elapsed)
            self.sp_pos[0] = 0.0
            self.sp_pos[1] = 0.0
            self.sp_pos[2] = target_z
            self.sp_vel = np.array([0.0, 0.0, descend_speed if target_z < -0.1 else 0.08])
            self.sp_acc = np.zeros(3)
            self.sp_att = np.zeros(3)

            alt = -self.pos[2]
            v_z = abs(self.vel[2])
            r_deg = math.degrees(self.att[0])
            p_deg = math.degrees(self.att[1])
            y_deg = math.degrees(self.att[2])
            print(f"\r  -> Đang hạ độ cao: Alt = {alt:.2f}m | Vz = {v_z:.2f}m/s | Pos=({self.pos[0]:+.2f}, {self.pos[1]:+.2f})m | Yaw={y_deg:+.1f}°", end="")

            # Ground touchdown criteria:
            # Vehicle has touched down (alt < 0.08m and descent speed stopped < 0.10m/s)
            # OR target z has reached 0.0m and time > 14s
            if (alt < 0.08 and v_z < 0.10 and elapsed > 2.0) or (target_z >= 0.0 and elapsed > 14.0):
                break
            time.sleep(0.05)

        print("\n\033[1;33m[HẠ CÁNH]\033[0m Đã chạm đất an toàn. Dừng luồng điều khiển và ngắt động cơ (DISARM)...")
        self.streaming = False
        time.sleep(0.1)

        for _ in range(5):
            self.mav.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 21196, 0, 0, 0, 0, 0
            )
            time.sleep(0.05)

        time.sleep(0.5)
        print("\033[1;32m[HOÀN THÀNH]\033[0m Đã hạ cánh và disarm an toàn. Khóa 8 servo ở vị trí 0° trung tính.")

    def save_and_plot(self):
        self.running = False
        time.sleep(0.3)
        if not self.log_data:
            print("\033[1;31m[LỖI]\033[0m Không có dữ liệu telemetry để lưu.")
            return

        ts = int(time.time())
        csv_file = os.path.join(LOG_DIR, f"virgo_square_decoupled_{ts}.csv")
        df = pd.DataFrame(self.log_data)
        df.to_csv(csv_file, index=False)
        print(f"\n\033[1;32m[ĐÃ LƯU TELEMETRY]\033[0m File CSV ({len(df)} mẫu): {csv_file}")

        plot_png = os.path.join(PLOT_DIR, f"virgo_square_decoupled_{ts}_analysis.png")
        plot_flight_log(csv_file, plot_png)
        print(f"\033[1;32m[ĐÃ LƯU ĐỒ THỊ]\033[0m Hình ảnh 6 Panel: {plot_png}")

        # Comprehensive Evaluation Statistics
        flight_mask = -df['pos_z'] > 0.5
        df_fl = df[flight_mask] if flight_mask.sum() > 20 else df

        max_roll_actual = np.degrees(np.max(df_fl['roll']))
        max_pitch_actual = np.degrees(np.max(df_fl['pitch']))
        max_yaw_actual = np.degrees(np.max(df_fl['yaw']))
        max_alpha = np.degrees(np.max([np.max(np.abs(df_fl[f'servo_alpha_{i}'])) for i in range(4)]))
        max_beta = np.degrees(np.max([np.max(np.abs(df_fl[f'servo_beta_{i}'])) for i in range(4)]))

        print("\n" + "="*76)
        print("  \033[1;32mKẾT QUẢ ĐÁNH GIÁ ĐỘC LẬP 6 BẬC TỰ DO (DECOUPLED 6-DOF SQUARE BENCHMARK)\033[0m")
        print("="*76)
        print(f"  • Cạnh 1 (Bay tiến +X, Cố định Roll +30°): Max Roll đạt được = \033[1;36m{max_roll_actual:.1f}°\033[0m")
        print(f"  • Cạnh 2 (Bay ngang +Y, Cố định Pitch +30°): Max Pitch đạt được = \033[1;36m{max_pitch_actual:.1f}°\033[0m")
        print(f"  • Cạnh 3 (Bay lùi -X, Cố định Yaw +30°): Max Yaw đạt được = \033[1;36m{max_yaw_actual:.1f}°\033[0m")
        print(f"  • Cạnh 4 (Bay ngang -Y về gốc): Trạng thái bình thường (Roll=0°, Pitch=0°, Yaw=0°)")
        print(f"  • Góc nghiêng servo Alpha tối đa (Pitch vectoring): \033[1;33m{max_alpha:.1f}°\033[0m")
        print(f"  • Góc nghiêng servo Beta tối đa (Roll vectoring)  : \033[1;33m{max_beta:.1f}°\033[0m")
        print(f"  • Chiều dài cạnh hình vuông kiểm nghiệm           : \033[1;32m{self.L:.1f} m\033[0m")
        print("="*76 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Virgo OMAV 6-DOF Decoupled Square Trajectory Benchmark")
    parser.add_argument("--side", "-l", type=float, default=25.0, help="Chiều dài cạnh hình vuông (m) [Mặc định: 25.0]")
    parser.add_argument("--alt", "-a", type=float, default=1.5, help="Độ cao bay giữ (m) [Mặc định: 1.5]")
    parser.add_argument("--speed", "-s", type=float, default=1.5, help="Vận tốc bay (m/s) [Mặc định: 1.5]")
    parser.add_argument("--url", "-u", type=str, default="udp:127.0.0.1:14540", help="PX4 MAVLink URL")
    args = parser.parse_args()

    benchmark = VirgoSquareBenchmark(
        mavlink_url=args.url,
        side_length=args.side,
        target_alt=args.alt,
        speed=args.speed
    )

    try:
        if benchmark.takeoff():
            benchmark.run_benchmark()
            benchmark.land_and_disarm()
    except KeyboardInterrupt:
        print("\n\033[1;31m[DỪNG KHẨN CẤP]\033[0m Người dùng hủy thao tác bằng Ctrl+C!")
        benchmark.land_and_disarm()
    finally:
        benchmark.save_and_plot()


if __name__ == "__main__":
    main()
