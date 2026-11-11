#!/usr/bin/env python3
"""
==============================================================================
  Virgo OMAV - 360-Degree Roll Rotation Experiment with Gazebo Screen Recording
  - Captures real-time 3D simulation video directly from Gazebo Sim window.
  - Commands full 360-degree body roll rotation while maintaining 2.5m hover.
  - Leverages omnidirectional thrust vectoring (+/- 180 deg servo rotation).
  - Exports high-quality animated GIF and flight telemetry analysis.
==============================================================================
"""

import argparse
import math
import os
import re
import subprocess
import sys
import threading
import time
import numpy as np
import pandas as pd
from PIL import Image
import mss
from pymavlink import mavutil

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(SCRIPT_DIR, "../docs/flight_logs")
PLOT_DIR = os.path.join(SCRIPT_DIR, "../docs/flight_plots")
os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(PLOT_DIR, exist_ok=True)


class GazeboRecorder:
    def __init__(self, target_fps=15, output_size=(800, 500)):
        self.target_fps = target_fps
        self.output_size = output_size
        self.frames = []
        self.running = False
        self.thread = None
        self.bounds = self._setup_gazebo_window()

    def _setup_gazebo_window(self):
        """Finds Gazebo Sim window, repositions, raises, and sets camera tracking."""
        os.environ['DISPLAY'] = ':0'
        try:
            from Xlib import display, X, protocol
            d = display.Display()
            root = d.screen().root

            def find_win(win, name):
                try:
                    wname = win.get_wm_name()
                    if wname and name.lower() in wname.lower():
                        return win
                except Exception:
                    pass
                for ch in win.query_tree().children:
                    res = find_win(ch, name)
                    if res:
                        return res
                return None

            w = find_win(root, 'Gazebo Sim')
            if w:
                print(f"\033[1;36m[Gazebo Recorder]\033[0m Tìm thấy cửa sổ Gazebo: {hex(w.id)}")
                # Configure window to 1600x1000 at 100, 100
                w.configure(x=100, y=100, width=1600, height=1000)
                
                # Activate window via _NET_ACTIVE_WINDOW
                net_active = d.intern_atom('_NET_ACTIVE_WINDOW')
                ev = protocol.event.ClientMessage(
                    window=w,
                    client_type=net_active,
                    data=(32, [1, X.CurrentTime, 0, 0, 0])
                )
                root.send_event(ev, event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask)
                w.configure(stack_mode=X.Above)
                d.sync()
                time.sleep(0.5)

            # Focus camera on virgo_0 with 3D isometric follow
            subprocess.run(
                "gz topic -t /gui/track -m gz.msgs.CameraTrack -p "
                "'track_mode: 4, follow_target: {name: \"virgo_0\", type: 2}, "
                "track_target: {name: \"virgo_0\", type: 2}, "
                "follow_offset: {x: -2.2, y: -2.2, z: 1.2}, follow_pgain: 0.1, track_pgain: 0.1'",
                shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
        except Exception as e:
            print(f"[Gazebo Recorder] Cảnh báo cấu hình X11/gz: {e}")

        # Capture bounds inside Gazebo Sim window
        # Window is at x=100, y=128 with size 1600x1000
        # We capture the main 3D viewport region (x: 100..1600, y: 128..1050)
        return {'top': 128, 'left': 100, 'width': 1600, 'height': 970}

    def start(self):
        self.running = True
        self.frames = []
        self.thread = threading.Thread(target=self._record_loop, daemon=True)
        self.thread.start()
        print("\033[1;32m[Gazebo Recorder]\033[0m Bắt đầu ghi hình Gazebo Simulation (15 FPS)...")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        print(f"\033[1;32m[Gazebo Recorder]\033[0m Đã dừng ghi hình. Tổng số khung hình: {len(self.frames)}")

    def _record_loop(self):
        interval = 1.0 / self.target_fps
        with mss.MSS() as sct:
            while self.running:
                t0 = time.time()
                try:
                    shot = sct.grab(self.bounds)
                    img = Image.frombytes('RGB', shot.size, shot.bgra, 'raw', 'BGRX')
                    if self.output_size:
                        img = img.resize(self.output_size, Image.Resampling.LANCZOS)
                    self.frames.append(img)
                except Exception as e:
                    print(f"[Gazebo Recorder Error] {e}")
                
                dt = time.time() - t0
                sleep_time = max(0.001, interval - dt)
                time.sleep(sleep_time)

    def save_gif(self, output_path, fps=15):
        if not self.frames:
            print("[Gazebo Recorder] Không có khung hình nào để tạo GIF.")
            return None

        print(f"\033[1;36m[Gazebo Recorder]\033[0m Đang nén và xuất file GIF ({len(self.frames)} frames)...")
        duration_ms = int(1000.0 / fps)

        # Convert to adaptive palette with dithering for compact high-quality GIF
        processed_frames = []
        for i, frame in enumerate(self.frames):
            p_frame = frame.convert('P', palette=Image.Palette.ADAPTIVE, colors=128)
            processed_frames.append(p_frame)

        processed_frames[0].save(
            output_path,
            save_all=True,
            append_images=processed_frames[1:],
            optimize=True,
            duration=duration_ms,
            loop=0
        )
        file_size_mb = os.path.getsize(output_path) / (1024 * 1024)
        print(f"\033[1;32m[Gazebo Recorder]\033[0m Xuất GIF thành công: {output_path} ({file_size_mb:.2f} MB)")
        return output_path


class VirgoRoll360Experiment:
    def __init__(self, mavlink_url="udp:127.0.0.1:14540", target_alt=2.5, roll_rate_dps=22.5, max_angle=360.0):
        self.url = mavlink_url
        self.target_alt = float(target_alt)
        self.roll_rate = math.radians(float(roll_rate_dps))  # rad/s
        self.max_angle = float(max_angle)

        print(f"\033[1;36m[Virgo 360 Roll]\033[0m Đang kết nối PX4 SITL qua {self.url}...")
        self.mav = mavutil.mavlink_connection(self.url)
        self.mav.wait_heartbeat(timeout=15)
        self.target_sys = self.mav.target_system
        self.target_comp = self.mav.target_component
        print(f"\033[1;32m[Virgo 360 Roll]\033[0m Kết nối thành công! SysID={self.target_sys}, CompID={self.target_comp}")

        # Set parameter limits for full tilt
        self._set_param("MC_DES_ROLL_MAX", 180.0)
        self._set_param("MC_DES_ROLL_MIN", -180.0)
        self._set_param("MC_DES_PITCH_MAX", 180.0)
        self._set_param("MC_DES_PITCH_MIN", -180.0)
        self._set_param("MC_MAX_FXY", 1.0)
        self._set_param("FD_FAIL_R", 0.0)
        self._set_param("FD_FAIL_P", 0.0)

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
        self.sp_att = np.zeros(3)  # [roll, pitch, yaw] (rad)

        self.running = True
        self.streaming = False
        self.log_data = []

        # Threads
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.telemetry_thread.start()

        self.control_thread = threading.Thread(target=self._control_stream_loop, daemon=True)
        self.control_thread.start()

    def _set_param(self, name, val):
        try:
            self.mav.mav.param_set_send(
                self.target_sys, self.target_comp,
                name.encode('utf-8'), float(val),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
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
                    def _decode_sv(v):
                        if 0 <= v <= 1000:
                            return (v - 500.0) / 500.0 * math.pi
                        elif 1000 < v <= 2000:
                            return (v - 1500.0) / 500.0 * math.pi
                        elif abs(v) <= 1.05:
                            return v * math.pi
                        return 0.0
                    self.servos = np.array([_decode_sv(v) for v in raw])

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
        print("\033[1;36m[1. CẤT CÁNH]\033[0m Chờ đồng bộ EKF2...")
        t_wait = time.time()
        while (not self.has_att or not self.has_pos) and (time.time() - t_wait < 10.0) and self.running:
            time.sleep(0.1)
        time.sleep(1.0)
        initial_x = float(self.pos[0])
        initial_y = float(self.pos[1])
        initial_yaw = float(self.att[2])

        print(f"\033[1;33m[1. CẤT CÁNH]\033[0m Giữ tọa độ ({initial_x:.2f}, {initial_y:.2f})m, cất cánh lên {self.target_alt}m...")
        self.sp_pos = np.array([initial_x, initial_y, -self.target_alt])
        self.sp_vel = np.zeros(3)
        self.sp_att = np.array([0.0, 0.0, initial_yaw])
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
            print(f"\r  -> Alt = {-self.pos[2]:.2f}/{self.target_alt:.1f}m | Roll={r_deg:+.1f}° | Pitch={p_deg:+.1f}° | Yaw={y_deg:+.1f}°", end="")
            if alt_err < 0.15 and vel_mag < 0.2 and (time.time() - t0 > 3.0):
                break
            time.sleep(0.1)

        print(f"\n\033[1;32m[HOVER ỔN ĐỊNH]\033[0m Đạt độ cao {-self.pos[2]:.2f}m. Giữ thăng bằng 2.5s trước khi xoay Roll...")
        time.sleep(2.5)
        return True

    def execute_roll_360(self):
        print("\n" + "="*80)
        print("          BẮT ĐẦU THÍ NGHIỆM QUAY GÓC ROLL 360 ĐỘ (VIRGO OMAV)")
        print("="*80)
        print(f"  • Vận tốc góc Roll : {math.degrees(self.roll_rate):.1f} °/s")
        print(f"  • Góc quay mục tiêu: {self.max_angle:.1f}° (1 vòng tròn hoàn chỉnh)")
        print(f"  • Độ cao thử nghiệm: {self.target_alt:.1f} m")
        print("="*80 + "\n")

        total_angle_rad = math.radians(self.max_angle)
        duration = total_angle_rad / self.roll_rate
        t_start = time.time()

        initial_yaw = float(self.sp_att[2])

        while (time.time() - t_start < duration + 0.5) and self.running:
            elapsed = time.time() - t_start
            current_roll_cum = min(elapsed * self.roll_rate, total_angle_rad)
            deg_cum = math.degrees(current_roll_cum)

            # Map cumulative angle to [-pi, +pi] for Euler command
            roll_cmd = math.atan2(math.sin(current_roll_cum), math.cos(current_roll_cum))
            self.sp_att[0] = roll_cmd
            self.sp_att[1] = 0.0
            self.sp_att[2] = initial_yaw

            # Live telemetry display
            actual_roll_deg = math.degrees(self.att[0])
            actual_alt = -self.pos[2]
            sv_beta_deg = math.degrees(self.servos[1])  # Roll servo of pod 0
            m0_pwr = self.motors[0]

            print(f"\r[Roll 360 Progress: {deg_cum:5.1f}° / {self.max_angle:3.0f}°] "
                  f"Cmd Roll: {math.degrees(roll_cmd):+6.1f}° | Act Roll: {actual_roll_deg:+6.1f}° | "
                  f"Alt: {actual_alt:4.2f}m | Servo Beta: {sv_beta_deg:+6.1f}° | Motor: {m0_pwr:.2f}", end="")

            if actual_alt < 0.5 and (time.time() - t_start > 3.0):
                print(f"\n\033[1;31m[CẢNH BÁO AN TOÀN]\033[0m Độ cao xuống thấp ({actual_alt:.2f}m)! Phục hồi 0°!")
                break

            time.sleep(0.04)

        print(f"\n\033[1;32m[KẾT THÚC VÒNG ROLL]\033[0m Đã hoàn tất lệnh xoay Roll {deg_cum:.1f}°.")
        print("\033[1;36m[PHỤC HỒI]\033[0m Khóa góc phẳng (Roll=0°, Pitch=0°) và ổn định vị trí hover 3s...")
        self.sp_att[0] = 0.0
        self.sp_att[1] = 0.0
        self.sp_pos[0] = 0.0
        self.sp_pos[1] = 0.0
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
                self.sp_pos[2] += 0.25 * 0.1
            else:
                self.sp_pos[2] = 0.1

            if current_alt < 0.15:
                break
            time.sleep(0.1)

        print("\n\033[1;32m[TIẾP ĐẤT]\033[0m Chạm đất an toàn! DISARM động cơ...")
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
    parser = argparse.ArgumentParser(description="Virgo OMAV 360-Degree Roll Rotation Experiment with Gazebo Screen Recording")
    parser.add_argument("--url", default="udp:127.0.0.1:14540", help="PX4 MAVLink URL")
    parser.add_argument("--alt", type=float, default=2.5, help="Độ cao thử nghiệm (m)")
    parser.add_argument("--rate", type=float, default=24.0, help="Vận tốc góc Roll (°/s)")
    parser.add_argument("--angle", type=float, default=360.0, help="Góc quay tối đa (°)")
    parser.add_argument("--fps", type=int, default=15, help="FPS ghi hình Gazebo")
    args = parser.parse_args()

    # 1. Initialize Gazebo Screen Recorder
    recorder = GazeboRecorder(target_fps=args.fps, output_size=(800, 485))
    recorder.start()

    # 2. Initialize and run flight experiment
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

            # 3. Stop recording and save GIF
            recorder.stop()
            gif_filename = f"virgo_roll_360_gazebo_{int(time.time())}.gif"
            gif_path = os.path.join(PLOT_DIR, gif_filename)
            recorder.save_gif(gif_path, fps=args.fps)

            # Create symlink or latest copy
            latest_gif = os.path.join(PLOT_DIR, "virgo_roll_360_gazebo.gif")
            subprocess.run(f"cp {gif_path} {latest_gif}", shell=True)

            # Copy to artifact directory
            artifact_dir = "/home/duc/.gemini/antigravity/brain/063c79dc-5a8e-480b-8076-b8daeeb23834"
            subprocess.run(f"cp {gif_path} {os.path.join(artifact_dir, 'virgo_roll_360_gazebo.gif')}", shell=True)
            print(f"\033[1;32m[ARTIFACT COPIED]\033[0m Đã sao chép GIF vào artifact directory!")

    except KeyboardInterrupt:
        print("\n\033[1;31m[NGẮT BỞI NGƯỜI DÙNG]\033[0m Khẩn cấp hạ cánh...")
        exp.sp_att = np.zeros(3)
        exp.land_and_disarm()
        exp.save_log()
        recorder.stop()
    finally:
        exp.running = False


if __name__ == '__main__':
    main()
