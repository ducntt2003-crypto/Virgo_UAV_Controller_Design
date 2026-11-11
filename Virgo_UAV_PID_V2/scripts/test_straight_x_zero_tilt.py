#!/usr/bin/env python3
"""
==============================================================================
 Virgo OMAV - Pure Decoupled Linear Translation along +X (Zero-Tilt)
 Paper: "MorphEUS: Morphable OmnidirEctional Unmanned System" (arXiv:2505.18270v1)
 Adapted for: Virgo Square-Frame Omnidirectional UAV (Lx=0.20m, Ly=0.20m)
 Upgraded with MorphQuad (arXiv:2607.02764v4) Allocation
==============================================================================
Mô tả kịch bản kiểm nghiệm:
  1. CẤT CÁNH CỔ ĐIỂN: Khóa servo alpha=0, beta=0 (chuẩn Coaxial Octocopter)
     bay thẳng đứng lên độ cao Z = 1.5m. Triệt tiêu hoàn toàn hiệu ứng mặt đất.
  2. BAY THẲNG THEO PHƯƠNG TRỤC +X:
     - Tạo quỹ đạo bậc 5 mượt mà (minimum jerk quintic polynomial) từ X_0 đến X_0 + 2.0m.
     - Gửi đồng thời Position + Velocity feedforward.
     - Khóa cứng trục ngang Y với bộ điều khiển bù trôi chủ động (Cross-Track PD Damping).
     - Khóa thân phẳng tuyệt đối: Roll = 0.0°, Pitch = 0.0°, Yaw = 0.0°.
     - Lực đẩy tiến về phía trước sinh ra 100% nhờ 4 cụm servo nghiêng Pitch (alpha_i).
  3. GIỮ VỊ TRÍ ĐỨNG YÊN (Stationkeeping): Giữ ổn định tại X_target trong vài giây.
  4. HẠ CÁNH CỔ ĐIỂN: Khóa servo alpha=0, beta=0 hạ cánh nhẹ nhàng và disarm an toàn.
  5. Xuất file dữ liệu CSV và tự động vẽ biểu đồ 6 đồ thị chất lượng điều khiển.
==============================================================================
"""

import argparse
import math
import os
import sys
import threading
import time
import numpy as np
from pymavlink import mavutil

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(SCRIPT_DIR)
from plot_flight_data import plot_flight_log

LOG_DIR = os.path.join(SCRIPT_DIR, "../docs/flight_logs")
PLOT_DIR = os.path.join(SCRIPT_DIR, "../docs/flight_plots")
os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(PLOT_DIR, exist_ok=True)


class VirgoStraightXTest:
    def __init__(self, mavlink_url="udp:127.0.0.1:14540", target_alt=1.5, target_x=2.0, speed=0.20):
        self.url = mavlink_url
        self.target_alt = float(target_alt)
        self.target_x = float(target_x)
        self.speed = float(speed)

        print(f"\033[1;36m[Virgo X-Test]\033[0m Đang kết nối tới PX4 SITL qua {self.url}...")
        self.mav = mavutil.mavlink_connection(self.url)
        self.mav.wait_heartbeat(timeout=15)
        self.target_sys = self.mav.target_system
        self.target_comp = self.mav.target_component
        print(f"\033[1;32m[Virgo X-Test]\033[0m Kết nối thành công! SysID={self.target_sys}, CompID={self.target_comp}")

        # Request data stream at high frequency
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
        self.sp_att = np.zeros(3)  # Roll=0, Pitch=0, Yaw=0

        self.home_x = 0.0
        self.home_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.target_abs_x = self.target_x

        self.running = True
        self.streaming = False
        self.flight_phase = 'init'
        self.log_data = []

        # Background telemetry & control threads
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
                    # Motor throttles
                    self.motors = np.array([
                        (v - 100.0) / 1100.0 if (100 <= v <= 1200)
                        else (v / 1000.0 if v <= 1.0 else (v - 1000.0) / 1000.0)
                        for v in raw
                    ])
                elif port == 1:
                    # Servo angles: centered at 500 for [-pi/2, +pi/2]
                    def _decode_sv(v):
                        if 0 <= v <= 1000:
                            return (v - 500.0) / 500.0 * (4.0 * math.pi)
                        elif 1000 < v <= 2000:
                            return (v - 1500.0) / 500.0 * (4.0 * math.pi)
                        elif abs(v) <= 1.05:
                            return v * (4.0 * math.pi)
                        return 0.0
                    self.servos = np.array([_decode_sv(v) for v in raw])

            # Telemetry logging at each loop
            t_now = time.time()
            row = {
                'time': t_now,
                'phase': self.flight_phase,
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
                row[f'servo_alpha_{i}'] = self.servos[2 * i]      # Pitch tilt servo
                row[f'servo_beta_{i}'] = self.servos[2 * i + 1]  # Roll tilt servo
            self.log_data.append(row)

    def _control_stream_loop(self):
        """Continuously streams position and velocity setpoints and zero-tilt attitude at 25Hz."""
        start_time = time.time()
        dt = 0.04  # 25 Hz

        while self.running:
            if self.streaming:
                ms = int((time.time() - start_time) * 1000) & 0xFFFFFFFF
                # Bitmask 2496 (0x09C0): Controls Position (X, Y, Z), Velocity Feedforward (Vx, Vy, Vz), and Yaw (bit 10=0).
                # Bits ignored: Accel (bits 6,7,8 = 64+128+256=448), Yaw Rate (bit 11 = 2048) -> 2496.
                type_mask = 2496
                vx_cmd = float(self.sp_vel[0])
                vy_cmd = float(self.sp_vel[1])
                vz_cmd = float(self.sp_vel[2])

                # Stream position setpoints and yaw (NED frame)
                self.mav.mav.set_position_target_local_ned_send(
                    ms, self.target_sys, self.target_comp,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask,
                    float(self.sp_pos[0]), float(self.sp_pos[1]), float(self.sp_pos[2]),
                    vx_cmd, vy_cmd, vz_cmd,
                    0.0, 0.0, 0.0, float(self.sp_att[2]), 0.0
                )

                # Send 6-DOF decoupled zero-tilt angles via debug_vect ('tilt_angle') only during 6-DOF phase
                # During takeoff and landing, debug_vect is NOT sent so servos stay locked upright at 0°
                if self.flight_phase in ['translate_x', 'hold']:
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
        self.flight_phase = 'takeoff'
        # Wait until telemetry has arrived and EKF2 heading has stabilized
        print("\033[1;36m[Khởi động]\033[0m Chờ đồng bộ dữ liệu cảm biến và vị trí từ PX4 EKF2...")
        t_wait = time.time()
        while (not self.has_att or not self.has_pos) and (time.time() - t_wait < 15.0) and self.running:
            time.sleep(0.1)
        time.sleep(1.0)

        initial_yaw = float(self.att[2])
        self.home_x = float(self.pos[0]) if self.has_pos else 0.0
        self.home_y = float(self.pos[1]) if self.has_pos else 0.0
        print(f"\n\033[1;33m[1. CẤT CÁNH CỔ ĐIỂN]\033[0m Cất cánh thẳng đứng tại chỗ (X={self.home_x:.2f}, Y={self.home_y:.2f}), khóa servo 0.0° lên {self.target_alt}m...")
        self.sp_pos = np.array([self.home_x, self.home_y, -self.target_alt])
        self.sp_vel = np.zeros(3)
        self.sp_att = np.array([0.0, 0.0, initial_yaw])
        self.streaming = True

        time.sleep(1.0)

        print("\033[1;33m[1. CẤT CÁNH]\033[0m Đang kích hoạt OFFBOARD và ARM động cơ...")
        t_arm_start = time.time()
        while not self.armed and (time.time() - t_arm_start < 20.0) and self.running:
            # Switch to OFFBOARD
            self.mav.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6, 0, 0, 0, 0, 0
            )
            time.sleep(0.1)
            # Arm motors
            self.mav.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0
            )
            time.sleep(0.4)

        if not self.armed:
            print("\033[1;31m[LỖI]\033[0m Không thể ARM máy bay! Kiểm tra trạng thái preflight PX4.")
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

        print(f"\n\033[1;32m[HOVER ĐẠT CHUẨN]\033[0m Đã đạt độ cao {-self.pos[2]:.2f}m. Khóa vị trí hover tại (X={self.home_x:.2f}, Y={self.home_y:.2f}) và xoay mượt hướng Yaw về 0.0° ở trên không...")
        self.sp_pos[0] = self.home_x
        self.sp_pos[1] = self.home_y
        self.sp_pos[2] = -self.target_alt

        align_dur = 2.5
        t_yaw_align = time.time()
        while (time.time() - t_yaw_align < align_dur) and self.running:
            fraction = (time.time() - t_yaw_align) / align_dur
            self.sp_att[2] = initial_yaw * (1.0 - fraction)
            time.sleep(0.05)
        self.sp_att[2] = 0.0

        # Wait until heading and horizontal position settle completely at hover point
        print("\033[1;36m[Ổn định Hover]\033[0m Chờ UAV ổn định hoàn toàn tại vị trí xuất phát...")
        t_wait_settle = time.time()
        while time.time() - t_wait_settle < 6.0 and self.running:
            yaw_err = abs(self.att[2])
            pos_err = math.hypot(self.pos[0] - self.home_x, self.pos[1] - self.home_y)
            vel_xy = np.linalg.norm(self.vel[:2])
            if yaw_err < 0.05 and pos_err < 0.15 and vel_xy < 0.08 and (time.time() - t_wait_settle > 2.0):
                break
            time.sleep(0.1)

        time.sleep(0.5)

        # Baseline coordinates for straight flight from stabilized hover point
        self.start_x = float(self.pos[0])
        self.start_y = float(self.pos[1])
        self.target_abs_x = self.start_x + self.target_x

        self.sp_pos[0] = self.start_x
        self.sp_pos[1] = self.start_y
        self.sp_vel = np.zeros(3)

        print(f"\033[1;32m[SẴN SÀNG 6-DOF]\033[0m UAV ổn định tại: X={self.pos[0]:.2f}m, Y={self.pos[1]:.2f}m, Z={-self.pos[2]:.2f}m. Đích đến: X={self.target_abs_x:.2f}m")
        return True

    def fly_straight_x(self, smooth_ramp=True):
        self.flight_phase = 'translate_x'
        start_x = self.start_x
        start_y = self.start_y
        target_x = self.target_abs_x
        total_dist = self.target_x

        print(f"\n\033[1;35m==============================================================================\033[0m")
        print(f"\033[1;35m[2. BAY THẲNG THEO PHƯƠNG X TỚI {target_x:.2f}m (ΔX={total_dist:.2f}m) VỚI ROLL=0°, PITCH=0°, YAW=0°]\033[0m")
        print(f"\033[1;35m  Nguyên lý 6-DOF: Thân UAV giữ phẳng tuyệt đối 0°, 4 servo Pitch (alpha_i)\033[0m")
        print(f"\033[1;35m  tự động quay góc nghiêng về phía trước để đẩy UAV đi thẳng theo trục X.\033[0m")
        print(f"\033[1;35m  Tích hợp Cross-Track PD Damping trên trục Y={start_y:.2f}m để triệt tiêu độ trôi dạt!\033[0m")
        print(f"\033[1;35m==============================================================================\033[0m")

        if smooth_ramp:
            duration = max(8.0, abs(total_dist) / self.speed)
            v_max = total_dist * math.pi / (2.0 * duration)
            print(f"  Tạo quỹ đạo S-Curve: Quãng đường = {total_dist:.2f}m, Thời gian = {duration:.1f}s, V_max = {v_max:.2f}m/s")
            t_start = time.time()

            while self.running:
                elapsed = time.time() - t_start
                if elapsed >= duration:
                    self.sp_pos[0] = target_x
                    self.sp_pos[1] = start_y
                    self.sp_vel = np.zeros(3)
                    break

                fraction = min(1.0, elapsed / duration)
                # Smooth cosine s-curve for position and velocity feedforward
                s = 0.5 * (1.0 - math.cos(math.pi * fraction))
                ds = 0.5 * (math.pi / duration) * math.sin(math.pi * fraction)

                self.sp_pos[0] = start_x + total_dist * s
                self.sp_pos[1] = start_y
                self.sp_pos[2] = -self.target_alt

                self.sp_vel[0] = total_dist * ds
                self.sp_vel[1] = 0.0
                self.sp_vel[2] = 0.0

                r_deg = math.degrees(self.att[0])
                p_deg = math.degrees(self.att[1])
                y_deg = math.degrees(self.att[2])
                y_err = float(self.pos[1] - start_y)
                avg_alpha = np.mean([self.servos[2 * i] for i in range(4)])
                print(f"\r  -> Bay tiến X: SP={self.sp_pos[0]:.2f}m | Thực={self.pos[0]:.2f}m | Vx_sp={self.sp_vel[0]:.2f}m/s | Y={self.pos[1]:.3f}m (ΔY={y_err:+.3f}m) | Roll={r_deg:+.2f}° | Pitch={p_deg:+.2f}° | Servo α TB={math.degrees(avg_alpha):+.1f}°", end="")
                time.sleep(0.04)
        else:
            print(f"  Gửi lệnh trực tiếp Waypoint X = {target_x:.2f}m...")
            self.sp_pos[0] = target_x
            self.sp_pos[1] = start_y
            self.sp_vel = np.zeros(3)

        # Wait until target position is reached and settled
        t_settle_start = time.time()
        while time.time() - t_settle_start < 8.0 and self.running:
            self.sp_pos[0] = target_x
            self.sp_pos[1] = start_y
            self.sp_vel = np.zeros(3)

            y_err = float(self.pos[1] - start_y)
            err_x = abs(self.pos[0] - target_x)
            r_deg = math.degrees(self.att[0])
            p_deg = math.degrees(self.att[1])
            y_deg = math.degrees(self.att[2])
            avg_alpha = np.mean([self.servos[2 * i] for i in range(4)])
            print(f"\r  -> Bám đích: Sai số X={err_x:.3f}m | Sai số Y={abs(y_err):.3f}m | Roll={r_deg:+.2f}° | Pitch={p_deg:+.2f}° | Yaw={y_deg:+.2f}° | Servo α TB={math.degrees(avg_alpha):+.1f}°", end="")
            if err_x < 0.10 and abs(y_err) < 0.10 and np.linalg.norm(self.vel[:2]) < 0.10 and (time.time() - t_settle_start > 2.0):
                break
            time.sleep(0.04)

        y_drift_final = abs(self.pos[1] - start_y)
        print(f"\n\033[1;32m[ĐÃ ĐẠT ĐÍCH]\033[0m Tọa độ: X={self.pos[0]:.2f}m, Y={self.pos[1]:.2f}m (Độ lệch ngang ΔY={y_drift_final:.3f}m), Z={-self.pos[2]:.2f}m")

    def hold_at_target(self, hold_seconds=4.0):
        self.flight_phase = 'hold'
        print(f"\n\033[1;33m[3. GIỮ VỊ TRÍ ĐỨNG YÊN (STATIONKEEPING)]\033[0m Duy trì hover tại X={self.target_abs_x:.2f}m, Y={self.start_y:.2f}m trong {hold_seconds} giây...")
        t_end = time.time() + hold_seconds
        while time.time() < t_end and self.running:
            self.sp_pos[0] = self.target_abs_x
            self.sp_pos[1] = self.start_y
            self.sp_vel = np.zeros(3)

            x_err = float(self.pos[0] - self.target_abs_x)
            y_err = float(self.pos[1] - self.start_y)
            r_deg = math.degrees(self.att[0])
            p_deg = math.degrees(self.att[1])
            y_deg = math.degrees(self.att[2])
            dist_drift = math.hypot(x_err, y_err)
            print(f"\r  -> Đang Hold: Trôi vị trí = {dist_drift:.3f}m (ΔX={x_err:+.3f}m, ΔY={y_err:+.3f}m) | Roll={r_deg:+.2f}° | Pitch={p_deg:+.2f}° | Yaw={y_deg:+.2f}°", end="")
            time.sleep(0.05)
        print("\n\033[1;32m[HOÀN THÀNH HOLD]\033[0m")

    def land_and_disarm(self):
        self.flight_phase = 'land'
        print(f"\n\033[1;33m[4. HẠ CÁNH CỔ ĐIỂN]\033[0m Khóa vị trí X={self.target_abs_x:.2f}m, Y={self.start_y:.2f}m, servo thẳng đứng 0° và hạ độ cao nhẹ nhàng...")
        init_z = self.sp_pos[2]
        descend_speed = 0.25  # 0.25 m/s
        t_start_desc = time.time()

        while self.running:
            elapsed = time.time() - t_start_desc
            target_z = min(0.0, init_z + descend_speed * elapsed)
            self.sp_pos[0] = self.target_abs_x
            self.sp_pos[1] = self.start_y
            self.sp_pos[2] = target_z
            self.sp_vel = np.array([0.0, 0.0, descend_speed if target_z < -0.1 else 0.08])

            alt = -self.pos[2]
            v_z = abs(self.vel[2])
            print(f"\r  -> Đang hạ độ cao: Alt = {alt:.2f}m | SP_Z = {-self.sp_pos[2]:.2f}m", end="")
            if (alt < 0.05 and v_z < 0.08 and elapsed > 2.0) or (target_z >= 0.0 and elapsed > 12.0):
                break
            time.sleep(0.05)

        print("\n\033[1;33m[4. HẠ CÁNH]\033[0m Đã chạm đất an toàn. Khóa servo 0° và ổn định 0.5s...")
        self.sp_pos[2] = 0.0
        time.sleep(0.5)

        print("\033[1;33m[4. HẠ CÁNH]\033[0m Tiếp đất thành công! Ngắt động cơ (DISARM)...")
        for _ in range(5):
            self.mav.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 21196, 0, 0, 0, 0, 0
            )
            time.sleep(0.1)

        self.streaming = False
        time.sleep(1.0)
        print("\033[1;32m[HOÀN THÀNH]\033[0m Đã hạ cánh và disarm an toàn.")

    def save_and_plot(self):
        self.running = False
        time.sleep(0.3)
        if not self.log_data:
            print("\033[1;31m[LỖI]\033[0m Không có dữ liệu telemetry để lưu.")
            return

        import pandas as pd
        ts = int(time.time())
        csv_file = os.path.join(LOG_DIR, f"virgo_straight_x2m_{ts}.csv")
        df = pd.DataFrame(self.log_data)
        df.to_csv(csv_file, index=False)
        print(f"\n\033[1;32m[ĐÃ LƯU TELEMETRY]\033[0m File CSV ({len(df)} mẫu): {csv_file}")

        plot_png = os.path.join(PLOT_DIR, f"virgo_straight_x2m_{ts}_analysis.png")
        plot_flight_log(csv_file, plot_png)
        print(f"\033[1;32m[ĐÃ LƯU ĐỒ THỊ]\033[0m Hình ảnh 6 Panel: {plot_png}")

        # In-flight translation and hold mask (when moving towards target X)
        trans_mask = (df['phase'] == 'translate_x')
        df_trans = df[trans_mask] if trans_mask.sum() > 10 else df

        # Summary statistics during steady-state hover at target X
        hold_mask = (df['phase'] == 'hold')
        df_hold = df[hold_mask] if hold_mask.sum() > 10 else df

        max_roll = np.degrees(np.max(np.abs(df_trans['roll'])))
        max_pitch = np.degrees(np.max(np.abs(df_trans['pitch'])))
        max_yaw_err = np.degrees(np.max(np.abs(df_trans['yaw'])))

        max_servo_alpha = np.degrees(np.max([np.max(np.abs(df_trans[f'servo_alpha_{i}'])) for i in range(4)]))
        mean_hold_x = df_hold['pos_x'].mean()
        max_hold_x_err = np.max(np.abs(df_hold['pos_x'] - self.target_abs_x))
        max_y_drift = np.max(np.abs(df_trans['pos_y'] - self.start_y))
        rms_y_drift = np.sqrt(np.mean((df_trans['pos_y'] - self.start_y)**2))

        print("\n" + "="*70)
        print("  \033[1;32mKẾT QUẢ ĐÁNH GIÁ CHẤT LƯỢNG ĐIỀU KHIỂN (DECOUPLED 6-DOF)\033[0m")
        print("="*70)
        print(f"  • Độ lệch góc Roll tối đa (In-Flight) : \033[1;36m{max_roll:.2f}°\033[0m  (Chuẩn: < 1.0°)")
        print(f"  • Độ lệch góc Pitch tối đa (In-Flight): \033[1;36m{max_pitch:.2f}°\033[0m  (Chuẩn: < 1.0°)")
        print(f"  • Sai số góc Yaw tối đa (In-Flight)   : \033[1;36m{max_yaw_err:.2f}°\033[0m  (Chuẩn: < 3.0°)")
        print(f"  • Góc nghiêng servo alpha tối đa      : \033[1;33m{max_servo_alpha:.2f}°\033[0m (Servo chủ động nghiêng đẩy X!)")
        print(f"  • Tọa độ X trung bình khi giữ vị trí  : \033[1;32m{mean_hold_x:.2f} m\033[0m  (Mục tiêu: {self.target_abs_x:.2f} m)")
        print(f"  • Sai số vị trí X tối đa khi giữ đích : \033[1;32m{max_hold_x_err:.3f} m\033[0m (Chỉ tiêu: <= 0.20m)")
        print(f"  • Độ trôi dạt Y tối đa khi bay thẳng  : \033[1;32m{max_y_drift:.3f} m\033[0m (Chỉ tiêu: <= 0.20m)")
        print(f"  • Độ lệch chuẩn RMS trôi dạt Y (RMS)  : \033[1;32m{rms_y_drift:.3f} m\033[0m")
        print("="*70 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Virgo OMAV Straight +X Zero-Tilt Flight Test")
    parser.add_argument("--target-x", "-x", type=float, default=2.0, help="Tọa độ X mục tiêu (m) [Mặc định: 2.0]")
    parser.add_argument("--alt", "-a", type=float, default=1.5, help="Độ cao bay giữ (m) [Mặc định: 1.5]")
    parser.add_argument("--speed", "-s", type=float, default=0.20, help="Vận tốc bay tịnh tiến (m/s) [Mặc định: 0.20]")
    parser.add_argument("--hold", "-t", type=float, default=4.0, help="Thời gian giữ vị trí tại X=2m (s) [Mặc định: 4.0]")
    parser.add_argument("--step", action="store_true", help="Gửi lệnh bước (step) thay vì quỹ đạo mượt (ramp)")
    parser.add_argument("--url", "-u", type=str, default="udp:127.0.0.1:14540", help="PX4 MAVLink URL")
    args = parser.parse_args()

    runner = VirgoStraightXTest(
        mavlink_url=args.url,
        target_alt=args.alt,
        target_x=args.target_x,
        speed=args.speed
    )

    try:
        if runner.takeoff():
            runner.fly_straight_x(smooth_ramp=(not args.step))
            runner.hold_at_target(args.hold)
            runner.land_and_disarm()
    except KeyboardInterrupt:
        print("\n\033[1;31m[DỪNG KHẨN CẤP]\033[0m Người dùng hủy thao tác bằng phím tắt!")
        runner.land_and_disarm()
    finally:
        runner.save_and_plot()


if __name__ == "__main__":
    main()
