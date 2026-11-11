#!/usr/bin/env python3
"""
==============================================================================
 ROS 2 Tilt-Hold Node for Virgo OMAV (H-Frame Omnidirectional UAV)
 Framework: ROS 2 Humble (rclpy)
==============================================================================
Node điều khiển UAV Virgo H-Frame cất cánh lên 1.5m, đứng yên tại chỗ và nghiêng thân
(Roll, Pitch) theo thông số người dùng nhập:
  1. Qua tham số ROS 2 (Parameters):
     ros2 run virgo_missions tilt_node --ros-args -p roll:=15.0 -p pitch:=-20.0 -p hold_sec:=10.0

  2. Qua ROS 2 Topic (Runtime dynamic control):
     ros2 topic pub /virgo/tilt_cmd geometry_msgs/msg/Vector3 "{x: 20.0, y: -15.0, z: 0.0}" --once

  3. Qua chế độ tương tác Terminal trực tiếp (Interactive CLI):
     ros2 run virgo_missions tilt_node --ros-args -p interactive:=true
==============================================================================
"""

import math
import os
import sys
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from pymavlink import mavutil

SCRIPT_DIR = "/home/duc/Omini_UAV/Virgo_UAV_PID_V2/scripts"
sys.path.append(SCRIPT_DIR)
try:
    from plot_flight_data import plot_flight_log
except ImportError:
    plot_flight_log = None

LOG_DIR = "/home/duc/Omini_UAV/Virgo_UAV_PID_V2/docs/flight_logs"
PLOT_DIR = "/home/duc/Omini_UAV/Virgo_UAV_PID_V2/docs/flight_plots"
os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(PLOT_DIR, exist_ok=True)


class VirgoTiltNode(Node):
    def __init__(self):
        super().__init__('virgo_tilt_node')

        # Declare ROS 2 parameters
        self.declare_parameter('roll', 0.0)         # degrees
        self.declare_parameter('pitch', 0.0)        # degrees
        self.declare_parameter('target_alt', 1.5)   # meters
        self.declare_parameter('hold_sec', 0.0)     # seconds
        self.declare_parameter('interactive', False)
        self.declare_parameter('mavlink_url', 'udp:127.0.0.1:14540')

        self.param_roll = self.get_parameter('roll').get_parameter_value().double_value
        self.param_pitch = self.get_parameter('pitch').get_parameter_value().double_value
        self.target_alt = self.get_parameter('target_alt').get_parameter_value().double_value
        self.hold_sec = self.get_parameter('hold_sec').get_parameter_value().double_value
        self.is_interactive = self.get_parameter('interactive').get_parameter_value().bool_value
        self.mav_url = self.get_parameter('mavlink_url').get_parameter_value().string_value

        self.get_logger().info(f"[Virgo ROS 2 Tilt] Kết nối PX4 SITL qua {self.mav_url}...")
        self.mav = mavutil.mavlink_connection(self.mav_url)
        self.mav.wait_heartbeat()
        self.target_sys = self.mav.target_system
        self.target_comp = self.mav.target_component
        self.get_logger().info(f"[Virgo ROS 2 Tilt] Đã kết nối! SysID={self.target_sys}, CompID={self.target_comp}")

        # State variables
        self.pos = np.zeros(3)  # Local NED
        self.vel = np.zeros(3)
        self.att = np.zeros(3)  # [roll, pitch, yaw] in rad
        self.motors = np.zeros(8)
        self.servos = np.zeros(8)

        # Setpoints
        self.sp_pos = np.array([0.0, 0.0, -self.target_alt])
        self.sp_att = np.zeros(3)
        self.sp_att_cmd = np.array([math.radians(self.param_roll), math.radians(self.param_pitch), 0.0])

        # Slew-rate configuration (40 deg/s)
        self.max_tilt_rate = math.radians(40.0)

        self.running = True
        self.streaming = False
        self.log_data = []

        # ROS 2 Subscriber for dynamic tilt commands
        self.tilt_sub = self.create_subscription(
            Vector3,
            '/virgo/tilt_cmd',
            self.tilt_cmd_callback,
            10
        )
        self.get_logger().info("[Virgo ROS 2 Tilt] Lắng nghe topic: /virgo/tilt_cmd (x=Roll°, y=Pitch°)")

        # Background telemetry thread
        self.telem_thread = threading.Thread(target=self._telemetry_worker, daemon=True)
        self.telem_thread.start()

        # 25Hz Control timer
        self.control_timer = self.create_timer(0.04, self._control_stream_callback)

        # Mission lifecycle thread
        self.mission_thread = threading.Thread(target=self._mission_lifecycle, daemon=True)
        self.mission_thread.start()

    def tilt_cmd_callback(self, msg: Vector3):
        r = max(min(msg.x, 45.0), -45.0)
        p = max(min(msg.y, 45.0), -45.0)
        self.sp_att_cmd[0] = math.radians(r)
        self.sp_att_cmd[1] = math.radians(p)
        self.get_logger().info(f"[ROS 2 CMD] Nhận góc mới: Roll={r:+.1f}°, Pitch={p:+.1f}°")

    def _telemetry_worker(self):
        while self.running:
            msg = self.mav.recv_match(blocking=True, timeout=0.1)
            if not msg:
                continue
            mtype = msg.get_type()
            if mtype == 'LOCAL_POSITION_NED':
                self.pos = np.array([msg.x, msg.y, msg.z])
                self.vel = np.array([msg.vx, msg.vy, msg.vz])
            elif mtype == 'ATTITUDE':
                self.att = np.array([msg.roll, msg.pitch, msg.yaw])
            elif mtype == 'SERVO_OUTPUT_RAW':
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
                            return (v - 500.0) / 500.0 * (math.pi / 2.0)
                        elif 1000 < v <= 2000:
                            return (v - 1500.0) / 500.0 * (math.pi / 2.0)
                        elif abs(v) <= 1.05:
                            return v * (math.pi / 2.0)
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

    def _control_stream_callback(self):
        if not self.streaming:
            return

        dt = 0.04
        for ax in range(2):
            err = self.sp_att_cmd[ax] - self.sp_att[ax]
            max_step = self.max_tilt_rate * dt
            step = max(min(err, max_step), -max_step)
            self.sp_att[ax] += step

        self.sp_att[2] = self.sp_att_cmd[2]

        type_mask = 0b0000111111111000
        ms = int(time.time() * 1000) & 0xFFFFFFFF
        self.mav.mav.set_position_target_local_ned_send(
            ms, self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            float(self.sp_pos[0]), float(self.sp_pos[1]), float(self.sp_pos[2]),
            0, 0, 0, 0, 0, 0, 0.0, 0.0
        )

        t_us = int(time.time() * 1e6) & 0xFFFFFFFFFFFFFFFF
        self.mav.mav.debug_vect_send(
            b'tilt_angle',
            t_us,
            float(self.sp_att[0]),
            float(self.sp_att[1]),
            float(self.sp_att[2])
        )

    def _mission_lifecycle(self):
        # Wait until telemetry has arrived and EKF2 heading has stabilized
        t_wait = time.time()
        while len(self.log_data) < 15 and (time.time() - t_wait < 6.0) and self.running:
            time.sleep(0.1)
        initial_yaw = float(self.att[2])
        self.get_logger().info(f"[Takeoff] Khởi động giữ hướng mặt đất ({math.degrees(initial_yaw):.1f}°), servo thẳng đứng, độ cao {self.target_alt}m...")
        self.sp_pos = np.array([0.0, 0.0, -self.target_alt])
        self.sp_att = np.array([0.0, 0.0, initial_yaw])
        self.sp_att_cmd = np.array([0.0, 0.0, initial_yaw])
        self.streaming = True
        time.sleep(1.0)

        self.get_logger().info("[Takeoff] Chuyển OFFBOARD...")
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)

        self.get_logger().info("[Takeoff] Kích hoạt động cơ (ARM)...")
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0
        )

        t0 = time.time()
        while time.time() - t0 < 8.0 and self.running:
            if abs(-self.pos[2] - self.target_alt) < 0.15 and (time.time() - t0 > 3.5):
                break
            time.sleep(0.1)

        self.get_logger().info(f"[HOVER ĐẠT CHUẨN] UAV Virgo đạt độ cao {-self.pos[2]:.2f}m. Đang xoay mượt hướng Yaw về 0.0° ở trên không...")
        t_yaw_align = time.time()
        align_dur = 2.0
        while time.time() - t_yaw_align < align_dur and self.running:
            fraction = (time.time() - t_yaw_align) / align_dur
            self.sp_att_cmd[2] = initial_yaw * (1.0 - fraction)
            time.sleep(0.05)
        self.sp_att_cmd[2] = 0.0
        time.sleep(0.5)

        self.get_logger().info(f"[HOVER ĐỨNG YÊN] UAV Virgo ổn định tại {-self.pos[2]:.2f}m. Giữ (X=0m, Y=0m)!")

        # Apply user command
        self.sp_att_cmd[0] = math.radians(self.param_roll)
        self.sp_att_cmd[1] = math.radians(self.param_pitch)
        self.get_logger().info(f"[TILT ACTIVE] Roll mong muốn={self.param_roll:+.1f}°, Pitch mong muốn={self.param_pitch:+.1f}°")

        if self.hold_sec > 0:
            t_end = time.time() + self.hold_sec
            while time.time() < t_end and self.running:
                time.sleep(0.5)
                r_deg = math.degrees(self.att[0])
                p_deg = math.degrees(self.att[1])
                self.get_logger().info(f"Tracking: Roll={r_deg:+.1f}°, Pitch={p_deg:+.1f}° | Pos: ({self.pos[0]:+.2f}, {self.pos[1]:+.2f}, {-self.pos[2]:.2f})m")
        elif self.is_interactive:
            self._run_interactive_cli()
        else:
            self.get_logger().info("Đang giữ góc nghiêng liên tục. Gửi lệnh qua /virgo/tilt_cmd hoặc Ctrl+C để hạ cánh.")
            while self.running:
                time.sleep(1.0)

        self._land_and_shutdown()

    def _run_interactive_cli(self):
        print("\n" + "="*76)
        print("  \033[1;35mROS 2 Virgo H-Frame Interactive Tilt CLI\033[0m")
        print("  Nhập: '<roll_deg> <pitch_deg>' (ví dụ: '20 -15') | '0 0' để phẳng | 'q' để hạ cánh")
        print("="*76)
        while self.running:
            try:
                cmd = input("\033[1;36m[Virgo ROS 2 Input] >> \033[0m").strip().lower()
                if not cmd:
                    continue
                if cmd in ['q', 'land', 'exit']:
                    break
                parts = cmd.split()
                if len(parts) >= 2:
                    r = max(min(float(parts[0]), 45.0), -45.0)
                    p = max(min(float(parts[1]), 45.0), -45.0)
                    self.sp_att_cmd[0] = math.radians(r)
                    self.sp_att_cmd[1] = math.radians(p)
                    print(f"\033[1;32m[Áp dụng]\033[0m Roll={r:+.1f}°, Pitch={p:+.1f}° (Vị trí giữ nguyên)")
                else:
                    print("Vui lòng nhập 2 số: <roll> <pitch> (ví dụ: '15 -10') hoặc 'q' để hạ cánh.")
            except (KeyboardInterrupt, EOFError):
                break

    def _land_and_shutdown(self):
        self.get_logger().info("[Landing] Trả thân về phẳng 0° và hạ cánh...")
        self.sp_att_cmd = np.zeros(3)
        time.sleep(1.5)

        self.sp_pos = np.array([0.0, 0.0, -0.1])
        t_land = time.time()
        while time.time() - t_land < 6.0 and self.running:
            if -self.pos[2] < 0.2:
                break
            time.sleep(0.2)

        self.get_logger().info("[Landing] Ngắt động cơ (DISARM)...")
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0
        )
        self.streaming = False
        time.sleep(1.0)
        self.get_logger().info("[Landing] Hạ cánh an toàn.")

        self._save_logs()
        rclpy.shutdown()

    def _save_logs(self):
        self.running = False
        time.sleep(0.3)
        if not self.log_data:
            return

        import pandas as pd
        ts = int(time.time())
        csv_file = os.path.join(LOG_DIR, f"virgo_ros2_tilt_{ts}.csv")
        df = pd.DataFrame(self.log_data)
        df.to_csv(csv_file, index=False)
        self.get_logger().info(f"[SAVED] Telemetry ({len(df)} mẫu): {csv_file}")

        if plot_flight_log:
            plot_png = os.path.join(PLOT_DIR, f"virgo_ros2_tilt_{ts}_analysis.png")
            plot_flight_log(csv_file, plot_png)
            self.get_logger().info(f"[SAVED] Biểu đồ: {plot_png}")


def main(args=None):
    rclpy.init(args=args)
    node = VirgoTiltNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.running = False


if __name__ == "__main__":
    main()
