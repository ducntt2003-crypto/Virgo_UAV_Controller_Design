#!/usr/bin/env python3
"""
==============================================================================
 PX4 MAVLink SE(3) NMPC Offboard Flight Runner for Virgo OMAV
 Connects VirgoNMPCController directly to PX4 Autopilot SITL & Gazebo Harmonic
 Benchmark: 25m x 25m Decoupled 6-DOF Square Trajectory with Corner Transitions
==============================================================================
"""

import argparse
import gzip
import math
import os
import sys
import threading
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pymavlink import mavutil

sys.path.append(os.path.join(os.path.dirname(__file__), "../models"))
sys.path.append(os.path.join(os.path.dirname(__file__), "../controllers"))
sys.path.append(os.path.join(os.path.dirname(__file__), "../simulation"))

from virgo_nmpc_controller import VirgoNMPCController
from se3_kinematics import euler_to_quat, quat_to_euler
from run_nmpc_standalone_sim import plot_nmpc_telemetry


class PX4MavlinkNMPCFlightRunner:
    """
    Executes the 25m decoupled benchmark flight in Gazebo Harmonic
    using the SE(3) NMPC optimal controller.
    """

    def __init__(self, connection_str="udp:127.0.0.1:14540", side=25.0, speed=1.5, altitude=1.5):
        print(f"\033[1;36m[NMPC Gazebo Runner]\033[0m Connecting to PX4 at {connection_str}...")
        self.mav = mavutil.mavlink_connection(connection_str)
        self.mav.wait_heartbeat()
        print(f"\033[1;32m[NMPC Gazebo Runner]\033[0m Heartbeat received from System {self.mav.target_system}, Component {self.mav.target_component}!")

        self.target_sys = self.mav.target_system
        self.target_comp = self.mav.target_component
        self.L = float(side)
        self.cruise_speed = float(speed)
        self.target_alt = float(altitude)

        # Initialize NMPC Controller
        self.controller = VirgoNMPCController()

        # Telemetry State
        self.pos = np.zeros(3)      # NED (m)
        self.vel = np.zeros(3)      # NED (m/s)
        self.att = np.zeros(3)      # Roll, Pitch, Yaw (rad)
        self.omega = np.zeros(3)    # p, q, r (rad/s)
        self.servos = np.zeros(8)   # 4 alpha (pitch), 4 beta (roll) (rad)
        self.motors = np.zeros(8)   # 8 motor PWMs

        self.sp_pos = np.zeros(3)
        self.sp_vel = np.zeros(3)
        self.sp_acc = np.zeros(3)
        self.sp_att = np.zeros(3)

        self.has_pos = False
        self.has_att = False
        self.armed = False
        self.streaming = False
        self.send_debug_vect = False
        self.running = True

        self.log_data = []

        # Start Telemetry & Streaming Threads
        self.rx_thread = threading.Thread(target=self._telemetry_receiver_loop, daemon=True)
        self.stream_thread = threading.Thread(target=self._setpoint_stream_loop, daemon=True)
        self.rx_thread.start()
        self.stream_thread.start()

    def _telemetry_receiver_loop(self):
        """Continuously decodes MAVLink messages from PX4."""
        while self.running:
            msg = self.mav.recv_match(blocking=True, timeout=0.1)
            if not msg:
                continue

            msg_type = msg.get_type()

            if msg_type == 'HEARTBEAT':
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            elif msg_type == 'LOCAL_POSITION_NED':
                self.pos = np.array([msg.x, msg.y, msg.z], dtype=float)
                self.vel = np.array([msg.vx, msg.vy, msg.vz], dtype=float)
                self.has_pos = True

            elif msg_type == 'ATTITUDE':
                self.att = np.array([msg.roll, msg.pitch, msg.yaw], dtype=float)
                self.omega = np.array([msg.rollspeed, msg.pitchspeed, msg.yawspeed], dtype=float)
                self.has_att = True

            elif msg_type == 'SERVO_OUTPUT_RAW':
                raw = [getattr(msg, f'servo{i+1}_raw', 0) for i in range(16)]
                def _decode_sv(v):
                    if 0 < v <= 1000:
                        return (v - 500.0) / 500.0 * (math.pi / 2.0)
                    elif 1000 < v <= 2000:
                        return (v - 1500.0) / 500.0 * (math.pi / 2.0)
                    elif abs(v) <= 1.05:
                        return v * (math.pi / 2.0)
                    return 0.0

                self.motors = np.array([raw[i] for i in range(8)])
                self.servos = np.array([_decode_sv(raw[i + 8]) for i in range(8)])

    def _setpoint_stream_loop(self):
        """Continuously streams setpoints and tilt angles to PX4 at 25 Hz, and logs telemetry."""
        type_mask = 0b0000100000000000  # Pos + Vel + Acc + Yaw
        dt = 0.04
        start_time = time.time()

        while self.running:
            if self.streaming:
                ms = int((time.time() - start_time) * 1000) & 0xFFFFFFFF
                self.mav.mav.set_position_target_local_ned_send(
                    ms, self.target_sys, self.target_comp,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask,
                    float(self.sp_pos[0]), float(self.sp_pos[1]), float(self.sp_pos[2]),
                    float(self.sp_vel[0]), float(self.sp_vel[1]), float(self.sp_vel[2]),
                    float(self.sp_acc[0]), float(self.sp_acc[1]), float(self.sp_acc[2]),
                    float(self.sp_att[2]), 0.0
                )

                if self.send_debug_vect:
                    t_us = int(time.time() * 1e6) & 0xFFFFFFFFFFFFFFFF
                    self.mav.mav.debug_vect_send(
                        b'tilt_angle',
                        t_us,
                        float(self.sp_att[0]),
                        float(self.sp_att[1]),
                        float(self.sp_att[2])
                    )

                # Synchronized 25 Hz Telemetry Logging
                t_now = time.time()
                row = {
                    'time': t_now,
                    'pos_x': float(self.pos[0]), 'pos_y': float(self.pos[1]), 'pos_z': float(self.pos[2]),
                    'vel_x': float(self.vel[0]), 'vel_y': float(self.vel[1]), 'vel_z': float(self.vel[2]),
                    'roll': float(self.att[0]), 'pitch': float(self.att[1]), 'yaw': float(self.att[2]),
                    'sp_x': float(self.sp_pos[0]), 'sp_y': float(self.sp_pos[1]), 'sp_z': float(self.sp_pos[2]),
                    'sp_vx': float(self.sp_vel[0]), 'sp_vy': float(self.sp_vel[1]), 'sp_vz': float(self.sp_vel[2]),
                    'sp_roll': float(self.sp_att[0]), 'sp_pitch': float(self.sp_att[1]), 'sp_yaw': float(self.sp_att[2])
                }
                for i in range(8):
                    row[f'motor_{i}'] = float(self.motors[i])
                for i in range(4):
                    row[f'servo_alpha_{i}'] = float(self.servos[2 * i])
                    row[f'servo_beta_{i}'] = float(self.servos[2 * i + 1])
                self.log_data.append(row)

            time.sleep(dt)

    def takeoff(self):
        print("\033[1;36m[1. Khởi động]\033[0m Chờ đồng bộ cảm biến EKF2 từ PX4...")
        t_wait = time.time()
        while (not self.has_att or not self.has_pos) and (time.time() - t_wait < 15.0) and self.running:
            time.sleep(0.1)

        time.sleep(1.0)
        init_x = float(self.pos[0])
        init_y = float(self.pos[1])
        init_yaw = float(self.att[2])

        print(f"\033[1;33m[2. Cất cánh]\033[0m Khóa vị trí ({init_x:.2f}, {init_y:.2f})m, bay thẳng đứng lên {self.target_alt:.1f}m...")
        self.sp_pos = np.array([init_x, init_y, -self.target_alt])
        self.sp_vel = np.zeros(3)
        self.sp_acc = np.zeros(3)
        self.sp_att = np.array([0.0, 0.0, init_yaw])
        self.streaming = True

        time.sleep(0.8)

        # Switch to OFFBOARD and ARM
        t_arm = time.time()
        while not self.armed and (time.time() - t_arm < 15.0) and self.running:
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
            time.sleep(0.3)

        if not self.armed:
            print("\033[1;31m[LỖI]\033[0m Không thể ARM máy bay! Kiểm tra SITL.")
            return False

        # Climb to target altitude
        t0 = time.time()
        while time.time() - t0 < 12.0 and self.running:
            alt_err = abs(-self.pos[2] - self.target_alt)
            vel_mag = np.linalg.norm(self.vel)
            if alt_err < 0.15 and vel_mag < 0.2 and (time.time() - t0 > 3.0):
                break
            time.sleep(0.1)

        print(f"\033[1;32m[Hover Sẵn sàng]\033[0m Độ cao {-self.pos[2]:.2f}m. Khởi tạo quỹ đạo NMPC.")
        self.send_debug_vect = True
        return True

    def transition_attitude(self, att_start, att_end, duration=3.5, hold_time=1.5, label=""):
        """Performs smooth stationary attitude transition."""
        print(f"\n\033[1;36m>>> [Chuyển thái độ Hover] {label} <<<\033[0m")
        self.sp_vel[:2] = 0.0
        self.sp_acc[:2] = 0.0

        t_start = time.time()
        while self.running:
            elapsed = time.time() - t_start
            if elapsed >= duration:
                self.sp_att = np.array(att_end, dtype=float)
                break
            tau = elapsed / duration
            s = 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)
            self.sp_att = att_start + (att_end - att_start) * s
            time.sleep(0.04)

        t_hold = time.time()
        while time.time() - t_hold < hold_time and self.running:
            time.sleep(0.05)

    def execute_segment(self, p_start, p_end, attitude_fn, segment_name):
        """Executes a straight segment while evaluating NMPC look-ahead."""
        delta_p = p_end - p_start
        dist = np.linalg.norm(delta_p[:2])
        dur = max(4.0, dist / self.cruise_speed)

        print(f"\n\033[1;32m>>> {segment_name} (Chiều dài: {dist:.1f}m, Thời gian: {dur:.1f}s) <<<\033[0m")

        t_start = time.time()
        while self.running:
            elapsed = time.time() - t_start
            if elapsed >= dur:
                self.sp_pos = p_end.copy()
                self.sp_vel = np.zeros(3)
                self.sp_acc = np.zeros(3)
                break

            tau = elapsed / dur
            # Quintic polynomial
            s = 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)
            s_dot = (30.0 * (tau**2) - 60.0 * (tau**3) + 30.0 * (tau**4)) / dur
            s_ddot = (60.0 * tau - 180.0 * (tau**2) + 120.0 * (tau**3)) / (dur**2)

            self.sp_pos = p_start + delta_p * s
            self.sp_vel = delta_p * s_dot
            self.sp_acc = delta_p * s_ddot

            r_sp, p_sp, y_sp = attitude_fn(tau)
            self.sp_att[0] = r_sp
            self.sp_att[1] = p_sp
            self.sp_att[2] = y_sp

            # Update NMPC with current telemetry & compute optimal lookahead
            quat = euler_to_quat(self.att[0], self.att[1], self.att[2])
            alpha_curr = np.array([self.servos[2*i] for i in range(4)])
            beta_curr  = np.array([self.servos[2*i+1] for i in range(4)])
            self.controller.update_state(self.pos, self.vel, quat, self.omega, alpha_curr, beta_curr)

            ctrl = self.controller.compute_control(elapsed)

            r_deg = math.degrees(self.att[0])
            p_deg = math.degrees(self.att[1])
            y_deg = math.degrees(self.att[2])
            print(f"\r  [{segment_name}] Pos: ({self.pos[0]:+.2f}, {self.pos[1]:+.2f})m | "
                  f"Att: R={r_deg:+.1f}°, P={p_deg:+.1f}°, Y={y_deg:+.1f}° | "
                  f"NMPC: {ctrl['solve_time_ms']:4.1f}ms", end="")
            time.sleep(0.04)

        time.sleep(0.5)

    def run_benchmark(self):
        L = self.L
        p0 = np.array([0.0, 0.0, -self.target_alt])
        p1 = np.array([L, 0.0, -self.target_alt])
        p2 = np.array([L, L, -self.target_alt])
        p3 = np.array([0.0, L, -self.target_alt])
        deg30 = math.radians(30.0)

        # 1. Tại gốc (0, 0): Nghiêng Roll +30°
        self.transition_attitude(np.zeros(3), np.array([deg30, 0.0, 0.0]), 3.5, 1.5, "1. Tại Gốc (0, 0): Roll +30°")

        # Cạnh 1: (0, 0) -> (+L, 0) giữ cố định Roll +30°
        self.execute_segment(p0, p1, lambda t: (deg30, 0.0, 0.0), f"CẠNH 1 (+X, Roll=+30° [L={L:.0f}m])")

        # 2. Tại (+L, 0): Chuyển Roll +30° sang Pitch +30°
        self.transition_attitude(np.array([deg30, 0.0, 0.0]), np.array([0.0, deg30, 0.0]), 3.5, 1.5, "2. Tại (+L, 0): Roll 30° -> Pitch 30°")

        # Cạnh 2: (+L, 0) -> (+L, +L) giữ cố định Pitch +30°
        self.execute_segment(p1, p2, lambda t: (0.0, deg30, 0.0), f"CẠNH 2 (+Y, Pitch=+30° [L={L:.0f}m])")

        # 3. Tại (+L, +L): Chuyển Pitch +30° sang Yaw +30°
        self.transition_attitude(np.array([0.0, deg30, 0.0]), np.array([0.0, 0.0, deg30]), 3.5, 1.5, "3. Tại (+L, +L): Pitch 30° -> Yaw 30°")

        # Cạnh 3: (+L, +L) -> (0, +L) giữ cố định Yaw +30°
        self.execute_segment(p2, p3, lambda t: (0.0, 0.0, deg30), f"CẠNH 3 (-X, Yaw=+30° [L={L:.0f}m])")

        # 4. Tại (0, +L): Chuyển về Flat 0°
        self.transition_attitude(np.array([0.0, 0.0, deg30]), np.zeros(3), 3.5, 1.5, "4. Tại (0, +L): Yaw 30° -> Flat 0°")

        # Cạnh 4: (0, +L) -> (0, 0) bay bình thường Flat 0° về gốc
        self.execute_segment(p3, p0, lambda t: (0.0, 0.0, 0.0), f"CẠNH 4 (-Y, Flat 0° [L={L:.0f}m])")

    def land_and_disarm(self):
        print(f"\n\033[1;33m[Hạ cánh]\033[0m Khóa servo 0° thẳng đứng, hạ cánh êm tại gốc (0, 0)...")
        self.send_debug_vect = False
        init_z = self.sp_pos[2]
        descend_speed = 0.25
        t_desc = time.time()

        self.sp_pos[0] = 0.0
        self.sp_pos[1] = 0.0
        self.sp_att = np.zeros(3)

        while self.running:
            elapsed = time.time() - t_desc
            target_z = min(0.0, init_z + descend_speed * elapsed)
            self.sp_pos[2] = target_z
            self.sp_vel = np.array([0.0, 0.0, descend_speed if target_z < -0.1 else 0.08])
            self.sp_acc = np.zeros(3)

            alt = -self.pos[2]
            v_z = abs(self.vel[2])
            if (alt < 0.08 and v_z < 0.10 and elapsed > 2.0) or (target_z >= 0.0 and elapsed > 14.0):
                break
            time.sleep(0.05)

        print("\n\033[1;32m[Chạm đất]\033[0m Đã hạ cánh an toàn. Ngắt động cơ (DISARM)...")
        self.streaming = False
        time.sleep(0.1)

        for _ in range(5):
            self.mav.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 21196, 0, 0, 0, 0, 0
            )
            time.sleep(0.05)

        print("\033[1;32m[DISARM THÀNH CÔNG]\033[0m Toàn bộ 8 servo khóa vị trí 0° trung tính.")

    def save_and_plot(self):
        self.running = False
        time.sleep(0.3)
        if not self.log_data:
            print("[Lỗi] Không có dữ liệu telemetry.")
            return

        out_dir = os.path.join(os.path.dirname(__file__), "../docs")
        os.makedirs(out_dir, exist_ok=True)
        ts = int(time.time())
        csv_file = os.path.join(out_dir, f"gazebo_nmpc_flight_{ts}.csv.gz")

        log_snapshot = [dict(r) for r in self.log_data]
        df = pd.DataFrame(log_snapshot)
        with gzip.open(csv_file, "wt", encoding="utf-8") as f:
            df.to_csv(f, index=False)
        print(f"\n\033[1;32m[ĐÃ LƯU TELEMETRY]\033[0m {csv_file} ({len(df)} mẫu)")

        plot_file = os.path.join(out_dir, f"gazebo_nmpc_flight_{ts}_analysis.png")
        plot_nmpc_telemetry(df, plot_file)
        print(f"\033[1;32m[ĐÃ LƯU ĐỒ THỊ 6-PANEL]\033[0m {plot_file}")

        # Summary statistics
        fl_mask = -df['pos_z'] > 0.5
        df_fl = df[fl_mask] if fl_mask.sum() > 20 else df

        pos_err = np.sqrt((df_fl['pos_x'] - df_fl['sp_x'])**2 + (df_fl['pos_y'] - df_fl['sp_y'])**2)
        roll_err = np.abs(np.degrees(df_fl['roll'] - df_fl['sp_roll']))
        pitch_err = np.abs(np.degrees(df_fl['pitch'] - df_fl['sp_pitch']))
        yaw_err = np.abs(np.degrees(df_fl['yaw'] - df_fl['sp_yaw']))

        print("\n" + "=" * 76)
        print("  \033[1;32mKẾT QUẢ BAY KIỂM NGHIỆM TRONG GAZEBO HARMONIC (SE(3) NMPC RUNNER)\033[0m")
        print("=" * 76)
        print(f"  • Sai số vị trí mặt phẳng trung bình: {pos_err.mean():.2f} m (Cực đại: {pos_err.max():.2f} m)")
        print(f"  • Sai số góc Roll trung bình:         {roll_err.mean():.2f}°")
        print(f"  • Sai số góc Pitch trung bình:        {pitch_err.mean():.2f}°")
        print(f"  • Sai số góc Yaw trung bình:          {yaw_err.mean():.2f}°")
        print("=" * 76 + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Virgo OMAV PX4 MAVLink NMPC Flight Runner")
    parser.add_argument("--conn", type=str, default="udp:127.0.0.1:14540")
    parser.add_argument("--side", type=float, default=25.0)
    parser.add_argument("--speed", type=float, default=1.5)
    parser.add_argument("--alt", type=float, default=1.5)
    args = parser.parse_args()

    runner = PX4MavlinkNMPCFlightRunner(args.conn, args.side, args.speed, args.alt)
    try:
        if runner.takeoff():
            runner.run_benchmark()
            runner.land_and_disarm()
            runner.save_and_plot()
    except KeyboardInterrupt:
        print("\n[Runner] Bị ngắt bởi người dùng.")
        runner.land_and_disarm()
        runner.save_and_plot()
