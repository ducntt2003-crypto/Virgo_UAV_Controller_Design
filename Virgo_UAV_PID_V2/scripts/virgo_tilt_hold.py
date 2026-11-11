#!/usr/bin/env python3
"""
==============================================================================
 Virgo OMAV - 6-DOF Decoupled In-Place Tilt & Hover Controller (H-Frame)
 Paper: "MorphEUS: Morphable OmnidirEctional Unmanned System" (arXiv:2505.18270v1)
 Adapted for: Virgo H-Frame Omnidirectional UAV (Lx=0.28m, Ly=0.22m)
==============================================================================
Chương trình cho UAV Virgo cất cánh lên độ cao 1.5m, đứng yên tại chỗ
(x=0, y=0, z=1.5m) và nghiêng thân (Roll, Pitch) theo góc nhập từ người dùng.

Tích hợp:
  - Slew-rate limiter (Bộ tạo quỹ đạo mượt chống shock giật servo).
  - Tách rời hoàn toàn vị trí và góc nghiêng (Decoupled 6-DOF).
  - Chế độ dòng lệnh hoặc tương tác terminal trực tiếp.
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


class VirgoTiltHold:
    def __init__(self, mavlink_url="udp:127.0.0.1:14540", target_alt=1.5):
        self.url = mavlink_url
        self.target_alt = target_alt
        print(f"\033[1;36m[Virgo Tilt]\033[0m Kết nối PX4 SITL qua {self.url}...")
        self.mav = mavutil.mavlink_connection(self.url)
        self.mav.wait_heartbeat()
        self.target_sys = self.mav.target_system
        self.target_comp = self.mav.target_component
        print(f"\033[1;32m[Virgo Tilt]\033[0m Đã kết nối PX4! SysID={self.target_sys}, CompID={self.target_comp}")

        # State estimates
        self.pos = np.zeros(3)  # Local NED: [x, y, z]
        self.vel = np.zeros(3)
        self.att = np.zeros(3)  # [roll, pitch, yaw] (rad)
        self.motors = np.zeros(8)
        self.servos = np.zeros(8)

        # Setpoints
        self.sp_pos = np.array([0.0, 0.0, -self.target_alt])
        self.sp_att = np.zeros(3)       # Current filtered setpoint [roll, pitch, yaw]
        self.sp_att_cmd = np.zeros(3)   # Desired raw user target

        # Slew-rate limiter configuration (rad/s)
        self.max_tilt_rate = math.radians(40.0)  # 40 deg/s for smooth motion

        self.has_att = False
        self.has_pos = False

        self.running = True
        self.streaming = False
        self.in_tilt_mode = False
        self.log_data = []

        # Background threads
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
            elif msg_type == 'SERVO_OUTPUT_RAW':
                port = getattr(msg, 'port', 0)
                raw = [getattr(msg, f'servo{i}_raw') for i in range(1, 9)]
                if port == 0:
                    self.motors = np.array([
                        (v - 100.0) / 1100.0 if (v >= 100 and v <= 1200)
                        else (v / 1000.0 if v <= 1.0 else (v - 1000.0) / 1000.0)
                        for v in raw
                    ])
                elif port == 1:
                    def _decode_sv(v):
                        if 0 <= v <= 1000:
                            return (v - 500.0) / 500.0 * (4.0 * math.pi)
                        elif 1000 < v <= 2000:
                            return (v - 1500.0) / 500.0 * (4.0 * math.pi)
                        elif abs(v) <= 1.05:
                            return v * (4.0 * math.pi)
                        return 0.0
                    self.servos = np.array([_decode_sv(v) for v in raw])

            t_now = time.time()
            row = {
                'time': t_now,
                'pos_x': self.pos[0], 'pos_y': self.pos[1], 'pos_z': self.pos[2],
                'vel_x': self.vel[0], 'vel_y': self.vel[1], 'vel_z': self.vel[2],
                'roll': self.att[0], 'pitch': self.att[1], 'yaw': self.att[2],
                'sp_x': self.sp_pos[0], 'sp_y': self.sp_pos[1], 'sp_z': self.sp_pos[2],
                'sp_roll': self.sp_att[0], 'sp_pitch': self.sp_att[1], 'sp_yaw': self.sp_att[2]
            }
            for i in range(8):
                row[f'motor_{i}'] = self.motors[i]
            for i in range(4):
                row[f'servo_alpha_{i}'] = self.servos[2*i]
                row[f'servo_beta_{i}'] = self.servos[2*i+1]
            self.log_data.append(row)

    def _control_stream_loop(self):
        """Continuously streams position and slew-rate limited tilt setpoints at 25Hz."""
        # Bitmask: use pos_x, pos_y, pos_z and yaw. Ignore vel, acc, yaw_rate.
        # Bit 10 (1024) is 0 -> USE YAW setpoint!
        # Bit 11 (2048) is 1 -> Ignore yaw rate.
        type_mask = 0b0000101111111000  # 2040: Pos + Yaw enabled
        start_time = time.time()
        dt = 0.04  # 25 Hz

        while self.running:
            if self.streaming:
                # Slew-rate filtering for Roll and Pitch setpoints
                for ax in range(2):
                    err = self.sp_att_cmd[ax] - self.sp_att[ax]
                    max_step = self.max_tilt_rate * dt
                    step = max(min(err, max_step), -max_step)
                    self.sp_att[ax] += step

                self.sp_att[2] = self.sp_att_cmd[2]  # Yaw direct

                ms = int((time.time() - start_time) * 1000) & 0xFFFFFFFF
                # Hold position fixed at (x=0, y=0, z=-target_alt) and hold active yaw
                self.mav.mav.set_position_target_local_ned_send(
                    ms, self.target_sys, self.target_comp,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask,
                    float(self.sp_pos[0]), float(self.sp_pos[1]), float(self.sp_pos[2]),
                    0, 0, 0, 0, 0, 0, float(self.sp_att[2]), 0.0
                )

                # Send 6-DOF decoupled tilt angles via debug_vect only during active tilt testing
                if self.in_tilt_mode:
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
        # Wait until telemetry has arrived and EKF2 heading has stabilized
        print("\033[1;36m[Takeoff]\033[0m Chờ đồng bộ dữ liệu cảm biến và góc Yaw từ PX4 EKF2...")
        t_wait = time.time()
        while (not self.has_att or not self.has_pos) and (time.time() - t_wait < 10.0) and self.running:
            time.sleep(0.1)
        time.sleep(1.0)
        initial_yaw = float(self.att[2])
        self.home_x = float(self.pos[0]) if self.has_pos else 0.0
        self.home_y = float(self.pos[1]) if self.has_pos else 0.0
        print(f"\n\033[1;33m[Takeoff]\033[0m Cất cánh thẳng đứng tại chỗ (X={self.home_x:.2f}, Y={self.home_y:.2f}), giữ hướng mặt đất ({math.degrees(initial_yaw):.1f}°) lên {self.target_alt}m...")
        self.sp_pos = np.array([self.home_x, self.home_y, -self.target_alt])
        self.sp_att = np.array([0.0, 0.0, initial_yaw])
        self.sp_att_cmd = np.array([0.0, 0.0, initial_yaw])
        self.streaming = True

        time.sleep(1.0)

        print("\033[1;33m[Takeoff]\033[0m Chuyển sang chế độ OFFBOARD...")
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)

        print("\033[1;33m[Takeoff]\033[0m Kích hoạt động cơ (ARM)...")
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0
        )

        t0 = time.time()
        while time.time() - t0 < 12.0 and self.running:
            alt_err = abs(-self.pos[2] - self.target_alt)
            vel_mag = np.linalg.norm(self.vel)
            if alt_err < 0.12 and vel_mag < 0.15 and (time.time() - t0 > 4.0):
                break
            time.sleep(0.1)

        print(f"\033[1;32m[HOVER ĐẠT CHUẨN]\033[0m Đã đạt độ cao {-self.pos[2]:.2f}m. Đang xoay mượt hướng Yaw về 0.0° ở trên không...")
        t_yaw_align = time.time()
        align_dur = 2.0
        while time.time() - t_yaw_align < align_dur and self.running:
            fraction = (time.time() - t_yaw_align) / align_dur
            self.sp_att_cmd[2] = initial_yaw * (1.0 - fraction)
            time.sleep(0.05)
        self.sp_att_cmd[2] = 0.0
        self.sp_pos = np.array([0.0, 0.0, -self.target_alt])
        t_settle = time.time()
        while time.time() - t_settle < 4.0 and self.running:
            d_hor = np.hypot(self.pos[0], self.pos[1])
            vel_hor = np.hypot(self.vel[0], self.vel[1])
            if d_hor < 0.20 and vel_hor < 0.15:
                break
            time.sleep(0.1)

        print(f"\033[1;32m[HOVER ĐỨNG YÊN]\033[0m UAV Virgo đã ổn định tại {-self.pos[2]:.2f}m! (X={self.pos[0]:.2f}, Y={self.pos[1]:.2f})")

    def set_body_tilt(self, roll_deg, pitch_deg):
        self.in_tilt_mode = True
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        self.sp_att_cmd[0] = roll_rad
        self.sp_att_cmd[1] = pitch_rad

    def land_and_disarm(self):
        print("\n\033[1;33m[Landing]\033[0m Trả thân về góc phẳng 0° và hạ cánh an toàn...")
        self.set_body_tilt(0.0, 0.0)
        time.sleep(1.0)
        self.in_tilt_mode = False

        init_z = self.sp_pos[2]
        descend_speed = 0.25
        t_land = time.time()
        while time.time() - t_land < 12.0 and self.running:
            elapsed = time.time() - t_land
            target_z = min(0.0, init_z + descend_speed * elapsed)
            self.sp_pos = np.array([self.home_x, self.home_y, target_z])
            alt = -self.pos[2]
            v_z = abs(self.vel[2])
            if (alt < 0.05 and v_z < 0.08 and elapsed > 2.0) or (target_z >= 0.0 and elapsed > 10.0):
                break
            time.sleep(0.05)

        print("\033[1;33m[Landing]\033[0m Đã chạm đất an toàn. Khóa vị trí 0.5s...")
        self.sp_pos[2] = 0.0
        time.sleep(0.5)

        print("\033[1;33m[Landing]\033[0m Ngắt động cơ (DISARM)...")
        for _ in range(5):
            self.mav.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 21196, 0, 0, 0, 0, 0
            )
            time.sleep(0.1)
        self.streaming = False
        time.sleep(1.0)
        print("\033[1;32m[Landing]\033[0m Đã hạ cánh và disarm thành công.")

    def save_results(self, tag="custom_tilt"):
        self.running = False
        time.sleep(0.3)
        if not self.log_data:
            return

        import pandas as pd
        ts = int(time.time())
        csv_file = os.path.join(LOG_DIR, f"virgo_tilt_{tag}_{ts}.csv")
        df = pd.DataFrame(self.log_data)
        df.to_csv(csv_file, index=False)
        print(f"\033[1;32m[ĐÃ LƯU LOG]\033[0m Telemetry ({len(df)} mẫu): {csv_file}")

        plot_png = os.path.join(PLOT_DIR, f"virgo_tilt_{tag}_{ts}_analysis.png")
        plot_flight_log(csv_file, plot_png)
        print(f"\033[1;32m[ĐÃ LƯU ĐỒ THỊ]\033[0m Biểu đồ 6 đồ thị: {plot_png}")


def run_interactive_mode(runner):
    print("\n" + "="*78)
    print("  \033[1;35mVirgo OMAV - Khung Chữ H: Chế Độ Nghiêng Thân Tương Tác Trực Tiếp\033[0m")
    print("="*78)
    print("  Hình học: Khung chữ H (Lx=0.28m, Ly=0.22m) - 8 Motor Coaxial + 8 Servo 2-DOF")
    print("  Vị trí: \033[1;32mĐỨNG YÊN TẠI CHỖ (X=0.0m, Y=0.0m, Z=1.5m)\033[0m")
    print("  Bộ lọc làm mượt: Slew-rate 40°/s (Triệt tiêu trôi dạt vị trí)")
    print("-"*78)
    print("  Cú pháp nhập:")
    print("    • Nhập 2 số cách nhau bởi dấu cách:  <roll_deg> <pitch_deg>")
    print("      Ví dụ:  '20 0'      -> Nghiêng Roll +20°, Pitch 0°")
    print("              '0 -20'     -> Nghiêng Pitch -20°, Roll 0°")
    print("              '15 25'     -> Nghiêng kết hợp Roll +15°, Pitch +25°")
    print("              '0 0'       -> Trở về cân bằng phẳng 0°")
    print("    • Nhập 'status' hoặc 's' để xem telemetry chi tiết hiện tại.")
    print("    • Nhập 'land' hoặc 'q' để hạ cánh an toàn.")
    print("="*78 + "\n")

    while runner.running:
        try:
            cmd = input("\033[1;36m[Virgo Tilt Input] >> \033[0m").strip().lower()
            if not cmd:
                continue

            if cmd in ['land', 'q', 'exit', 'quit']:
                break

            if cmd in ['status', 's']:
                print(f"  -> Trạng thái thực tế: Roll={math.degrees(runner.att[0]):+.1f}°, Pitch={math.degrees(runner.att[1]):+.1f}°")
                print(f"  -> Tọa độ thực tế: X={runner.pos[0]:+.2f}m, Y={runner.pos[1]:+.2f}m, Z={-runner.pos[2]:.2f}m")
                continue

            parts = cmd.split()
            if len(parts) >= 2:
                r_val = float(parts[0])
                p_val = float(parts[1])
                r_val = max(min(r_val, 45.0), -45.0)
                p_val = max(min(p_val, 45.0), -45.0)

                runner.set_body_tilt(r_val, p_val)
                print(f"\033[1;32m[ÁP DỤNG]\033[0m Đã phát lệnh: \033[1;33mRoll = {r_val:+.1f}°\033[0m, \033[1;33mPitch = {p_val:+.1f}°\033[0m")

                for _ in range(6):
                    time.sleep(0.4)
                    r_deg = math.degrees(runner.att[0])
                    p_deg = math.degrees(runner.att[1])
                    print(f"       \033[2mTracking -> Roll thực tế={r_deg:+.1f}° | Pitch thực tế={p_deg:+.1f}° | Pos: ({runner.pos[0]:+.2f}, {runner.pos[1]:+.2f}, {-runner.pos[2]:.2f})m\033[0m")
            else:
                print("\033[1;31m[LỖI]\033[0m Vui lòng nhập 2 số: <roll> <pitch> (ví dụ: '20 -15') hoặc 'q' để hạ cánh.")

        except ValueError:
            print("\033[1;31m[LỖI]\033[0m Giá trị không hợp lệ. Vui lòng nhập số.")
        except (KeyboardInterrupt, EOFError):
            break


def main():
    parser = argparse.ArgumentParser(description="Virgo OMAV H-Frame Tilt-Hold Controller")
    parser.add_argument("--roll", "-r", type=float, default=None, help="Góc nghiêng Roll đặt trước (độ)")
    parser.add_argument("--pitch", "-p", type=float, default=None, help="Góc nghiêng Pitch đặt trước (độ)")
    parser.add_argument("--alt", "-a", type=float, default=1.5, help="Độ cao giữ (m) [Mặc định: 1.5]")
    parser.add_argument("--hold", "-t", type=float, default=0.0, help="Thời gian giữ góc nghiêng (giây). Nếu = 0: Vào chế độ tương tác terminal.")
    parser.add_argument("--url", "-u", type=str, default="udp:127.0.0.1:14540", help="PX4 MAVLink URL")
    args = parser.parse_args()

    runner = VirgoTiltHold(args.url, args.alt)
    tag = "interactive"

    try:
        runner.takeoff()

        if args.roll is not None and args.pitch is not None and args.hold > 0:
            tag = f"r{int(args.roll)}_p{int(args.pitch)}"
            print(f"\n\033[1;35m>>> Đang nghiêng thân: Roll={args.roll:+.1f}°, Pitch={args.pitch:+.1f}° trong {args.hold} giây...\033[0m")
            runner.set_body_tilt(args.roll, args.pitch)
            t_end = time.time() + args.hold
            while time.time() < t_end and runner.running:
                r_deg = math.degrees(runner.att[0])
                p_deg = math.degrees(runner.att[1])
                dx = runner.pos[0] - runner.sp_pos[0]
                dy = runner.pos[1] - runner.sp_pos[1]
                print(f"\r[TILT-HOLD] SP: (Roll={args.roll:+.1f}°, Pitch={args.pitch:+.1f}°) | Thực tế: (Roll={r_deg:+.1f}°, Pitch={p_deg:+.1f}°) | Drift: (dX={dx:+.2f}, dY={dy:+.2f}, Z={-runner.pos[2]:.2f})m", end="")
                time.sleep(0.2)
            print("\n[Virgo] Hoàn thành thời gian giữ góc.")
        elif args.roll is not None and args.pitch is not None and args.hold == 0:
            runner.set_body_tilt(args.roll, args.pitch)
            run_interactive_mode(runner)
        else:
            run_interactive_mode(runner)

        runner.land_and_disarm()

    except KeyboardInterrupt:
        runner.land_and_disarm()
    finally:
        runner.save_results(tag)


if __name__ == "__main__":
    main()
