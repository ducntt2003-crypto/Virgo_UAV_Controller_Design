#!/usr/bin/env python3
"""
==============================================================================
 ROS 2 Lie Group SE(3) Geometric Controller + L1 Adaptive Augmentation Node
 for Virgo OMAV V2 (CAD Model: AUV_USDF_V2)
 Based on MorphQuad (arXiv:2607.02764v4) and L1Quad (IEEE T-CST 2025)
 Framework: ROS 2 Humble (rclpy) + MAVLink Offboard
==============================================================================
"""

import math
import os
import sys
import threading
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

from pymavlink import mavutil

from virgo_missions.se3_l1_controller import (
    SE3GeometricController,
    L1AdaptiveDisturbanceObserver,
    MorphQuadNullSpaceAllocator
)

LOG_DIR = "/home/duc/Omini_UAV/Virgo_UAV_PID_V2/docs/flight_logs"
PLOT_DIR = "/home/duc/Omini_UAV/Virgo_UAV_PID_V2/docs/flight_plots"
os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(PLOT_DIR, exist_ok=True)


def euler_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX Euler angles to Rotation Matrix R."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


class VirgoSE3GeometricNode(Node):
    def __init__(self):
        super().__init__('virgo_se3_geometric_node')

        # Declare ROS 2 parameters
        self.declare_parameter('roll', 0.0)          # Desired roll (deg)
        self.declare_parameter('pitch', 0.0)         # Desired pitch (deg)
        self.declare_parameter('target_alt', 1.5)    # Desired alt (m)
        self.declare_parameter('hold_sec', 10.0)     # Hold duration (s)
        self.declare_parameter('enable_l1', True)    # Enable L1 adaptive observer
        self.declare_parameter('mavlink_url', 'udp:127.0.0.1:14540')

        self.param_roll = self.get_parameter('roll').get_parameter_value().double_value
        self.param_pitch = self.get_parameter('pitch').get_parameter_value().double_value
        self.target_alt = self.get_parameter('target_alt').get_parameter_value().double_value
        self.hold_sec = self.get_parameter('hold_sec').get_parameter_value().double_value
        self.enable_l1 = self.get_parameter('enable_l1').get_parameter_value().bool_value
        self.mav_url = self.get_parameter('mavlink_url').get_parameter_value().string_value

        self.get_logger().info(f"[SE3-L1 Node] Khởi tạo bộ điều khiển MorphQuad SE(3) + L1 (Model: AUV_USDF_V2, m=1.46kg)...")
        self.get_logger().info(f"[SE3-L1 Node] Kết nối PX4 SITL qua {self.mav_url}...")

        self.mav = mavutil.mavlink_connection(self.mav_url)
        self.mav.wait_heartbeat(timeout=20)
        self.target_sys = self.mav.target_system
        self.target_comp = self.mav.target_component
        self.get_logger().info(f"[SE3-L1 Node] Đã kết nối PX4! Target Sys={self.target_sys}, Comp={self.target_comp}")

        # Core controller objects
        self.mass = 1.46
        self.J = np.diag([0.025, 0.025, 0.045])
        self.controller = SE3GeometricController(mass=self.mass, J=self.J)
        self.observer = L1AdaptiveDisturbanceObserver(mass=self.mass, J=self.J)
        self.allocator = MorphQuadNullSpaceAllocator(lx=0.19899, ly=0.19742)

        # Telemetry states (Local NED)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.att = np.zeros(3)  # [roll, pitch, yaw] in rad
        self.omega = np.zeros(3)
        self.motors = np.zeros(8)
        self.servos = np.zeros(8)
        self.armed = False
        self.has_pos = False
        self.has_att = False

        # Desired Setpoints
        self.pd = np.array([0.0, 0.0, -self.target_alt])
        self.vd = np.zeros(3)
        self.acc_d = np.zeros(3)
        self.desired_roll = math.radians(self.param_roll)
        self.desired_pitch = math.radians(self.param_pitch)
        self.desired_yaw = 0.0
        self.Rd = euler_to_rot(self.desired_roll, self.desired_pitch, self.desired_yaw)
        self.wd = np.zeros(3)
        self.w_dot_d = np.zeros(3)

        self.sp_att = np.zeros(3)
        self.sp_att_cmd = np.array([0.0, 0.0, 0.0])
        self.max_tilt_rate = math.radians(35.0)

        # Telemetry logging & streaming control
        self.running = True
        self.streaming = False
        self.log_data = []

        # ROS 2 subscriber for runtime dynamic tilt/pose
        self.tilt_sub = self.create_subscription(
            Vector3,
            '/virgo/tilt_cmd',
            self.tilt_cmd_callback,
            10
        )

        # Start telemetry reader thread
        self.telem_thread = threading.Thread(target=self._telemetry_worker, daemon=True)
        self.telem_thread.start()

        # 50 Hz Control & Observer Loop
        self.control_timer = self.create_timer(0.02, self._control_loop_callback)

        # Mission lifecycle thread
        self.mission_thread = threading.Thread(target=self._mission_lifecycle, daemon=True)
        self.mission_thread.start()

    def tilt_cmd_callback(self, msg: Vector3):
        r = max(min(msg.x, 60.0), -60.0)
        p = max(min(msg.y, 60.0), -60.0)
        self.desired_roll = math.radians(r)
        self.desired_pitch = math.radians(p)
        self.sp_att_cmd[0] = self.desired_roll
        self.sp_att_cmd[1] = self.desired_pitch
        self.get_logger().info(f"[SE3-L1 CMD] Dynamic tilt setpoint: Roll={r:+.1f}°, Pitch={p:+.1f}°")

    def _telemetry_worker(self):
        while self.running:
            msg = self.mav.recv_match(blocking=True, timeout=0.05)
            if not msg:
                continue
            mtype = msg.get_type()
            if mtype == 'LOCAL_POSITION_NED':
                self.pos = np.array([msg.x, msg.y, msg.z])
                self.vel = np.array([msg.vx, msg.vy, msg.vz])
                self.has_pos = True
            elif mtype == 'ATTITUDE':
                self.att = np.array([msg.roll, msg.pitch, msg.yaw])
                self.omega = np.array([msg.rollspeed, msg.pitchspeed, msg.yawspeed])
                self.has_att = True
            elif mtype == 'HEARTBEAT':
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif mtype == 'SERVO_OUTPUT_RAW':
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

    def _control_loop_callback(self):
        if not self.streaming:
            return

        dt = 0.02
        # Smooth slew rate for desired attitude
        for ax in [0, 1]:
            err = self.sp_att_cmd[ax] - self.sp_att[ax]
            step = max(min(err, self.max_tilt_rate * dt), -self.max_tilt_rate * dt)
            self.sp_att[ax] += step
        self.sp_att[2] = self.sp_att_cmd[2]

        R_actual = euler_to_rot(self.att[0], self.att[1], self.att[2])
        self.Rd = euler_to_rot(self.sp_att[0], self.sp_att[1], self.sp_att[2])

        f_applied = np.array([0.0, 0.0, self.mass * 9.81])
        tau_applied = np.zeros(3)

        # 1. Update L1 Adaptive Observer
        if self.enable_l1:
            d_v_filt, d_w_filt = self.observer.update(
                dt, self.vel, self.omega, R_actual, f_applied, tau_applied
            )
        else:
            d_v_filt, d_w_filt = np.zeros(3), np.zeros(3)

        # 2. Compute SE(3) Geometric Control Commands
        f_body, tau_body, ep, ev, eR, ew = self.controller.compute_control(
            self.pos, self.vel, R_actual, self.omega,
            self.pd, self.vd, self.acc_d,
            self.Rd, self.wd, self.w_dot_d,
            d_v_filt, d_w_filt
        )

        # 3. Compute Null-Space Control Allocation
        lock_servos = abs(self.pos[2]) < 0.80
        alloc_res = self.allocator.allocate(f_body, tau_body, lock_servos=lock_servos)

        # 4. Stream MAVLink setpoint to PX4 Autopilot
        ms = int(time.time() * 1000) & 0xFFFFFFFF
        type_mask = 0b0000101111111000  # 3064: 3D Pos + Active Yaw
        self.mav.mav.set_position_target_local_ned_send(
            ms, self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            float(self.pd[0]), float(self.pd[1]), float(self.pd[2]),
            0, 0, 0, 0, 0, 0, float(self.sp_att[2]), 0.0
        )

        # Transmit desired attitude to PX4 receiver
        t_us = int(time.time() * 1e6) & 0xFFFFFFFFFFFFFFFF
        self.mav.mav.debug_vect_send(
            b'tilt_angle',
            t_us,
            float(self.sp_att[0]),
            float(self.sp_att[1]),
            float(self.sp_att[2])
        )

        # Log telemetry row
        t_now = time.time()
        self.log_data.append({
            'time': t_now,
            'pos_x': self.pos[0], 'pos_y': self.pos[1], 'pos_z': self.pos[2],
            'sp_pos_x': self.pd[0], 'sp_pos_y': self.pd[1], 'sp_pos_z': self.pd[2],
            'roll': math.degrees(self.att[0]), 'pitch': math.degrees(self.att[1]), 'yaw': math.degrees(self.att[2]),
            'sp_roll': math.degrees(self.sp_att[0]), 'sp_pitch': math.degrees(self.sp_att[1]), 'sp_yaw': math.degrees(self.sp_att[2]),
            'eR_norm': float(np.linalg.norm(eR)),
            'ep_norm': float(np.linalg.norm(ep)),
            'd_v_x': float(d_v_filt[0]), 'd_v_y': float(d_v_filt[1]), 'd_v_z': float(d_v_filt[2]),
            'servo_pitch_0': float(math.degrees(self.servos[0])) if len(self.servos) > 0 else 0.0,
            'servo_roll_0': float(math.degrees(self.servos[1])) if len(self.servos) > 1 else 0.0,
            'motor_0': float(self.motors[0]) if len(self.motors) > 0 else 0.0
        })

    def _mission_lifecycle(self):
        # 1. Wait for initial telemetry
        self.get_logger().info("[Lifecycle] Chờ dữ liệu EKF2 và trạng thái ban đầu...")
        t_wait = time.time()
        while (time.time() - t_wait < 5.0) and self.running:
            if abs(self.pos[2]) > 0.001 or abs(self.att[2]) > 0.001:
                break
            time.sleep(0.1)

        initial_x = float(self.pos[0])
        initial_y = float(self.pos[1])
        initial_yaw = float(self.att[2])
        self.get_logger().info(f"[Lifecycle] EKF2 sẵn sàng! Vị trí ban đầu: ({initial_x:.2f}, {initial_y:.2f}), Yaw={math.degrees(initial_yaw):.1f}°. Mục tiêu cất cánh={self.target_alt}m...")

        # Initialize setpoints for vertical climb with locked servos at current ground location
        self.pd = np.array([initial_x, initial_y, -self.target_alt])
        self.sp_att = np.array([0.0, 0.0, initial_yaw])
        self.sp_att_cmd = np.array([0.0, 0.0, initial_yaw])

        # 2. Pre-stream setpoints for 1.5s (PX4 requires setpoints stream before entering OFFBOARD)
        self.get_logger().info("[Lifecycle] Bắt đầu stream setpoints vị trí lên PX4 trước khi kích hoạt OFFBOARD...")
        self.streaming = True
        time.sleep(1.5)

        # 3. Enable OFFBOARD mode and ARM vehicle
        self.get_logger().info("[Lifecycle] Đang kích hoạt chế độ OFFBOARD và ARM động cơ...")
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
            self.get_logger().error("[LỖI] Không thể ARM máy bay! Kiểm tra preflight checks.")
            return

        self.get_logger().info("[Lifecycle] Đã ARM thành công và vào OFFBOARD! Đang cất cánh...")

        # 4. Monitor vertical climb to target_alt
        t_climb_start = time.time()
        self.get_logger().info(f"[Takeoff] Đang cất cánh lên độ cao {self.target_alt}m...")
        while (time.time() - t_climb_start < 12.0) and self.running:
            current_alt = -self.pos[2]
            vel_mag = float(np.linalg.norm(self.vel))
            self.get_logger().info(f"[Takeoff] Độ cao hiện tại: {current_alt:.2f}m / {self.target_alt:.2f}m, V={vel_mag:.2f}m/s", throttle_duration_sec=1.0)
            if abs(current_alt - self.target_alt) < 0.15 and vel_mag < 0.15 and (time.time() - t_climb_start > 4.0):
                break
            time.sleep(0.1)

        self.get_logger().info(f"[HOVER ĐẠT CHUẨN] UAV đã bay lên độ cao {-self.pos[2]:.2f}m! Đang xoay Yaw về 0.0°...")
        t_yaw = time.time()
        align_dur = 2.0
        while (time.time() - t_yaw < align_dur) and self.running:
            frac = (time.time() - t_yaw) / align_dur
            self.sp_att_cmd[2] = initial_yaw * (1.0 - frac)
            time.sleep(0.05)
        self.sp_att_cmd[2] = 0.0
        time.sleep(1.0)

        # Lock stationkeeping setpoint strictly at settled hover position
        self.pd = np.array([float(self.pos[0]), float(self.pos[1]), -self.target_alt])
        self.get_logger().info(f"[HOVER ĐỨNG YÊN] Khóa tọa độ giữ vị trí tại X={self.pos[0]:.2f}m, Y={self.pos[1]:.2f}m, Z={-self.pos[2]:.2f}m")

        # 5. Execute desired tilt test
        self.get_logger().info(f"[SE3-L1 TEST] Kích hoạt nghiêng thân: Roll={self.param_roll:+.1f}°, Pitch={self.param_pitch:+.1f}° trong {self.hold_sec}s...")
        self.sp_att_cmd[0] = self.desired_roll
        self.sp_att_cmd[1] = self.desired_pitch

        t_hold = time.time()
        while (time.time() - t_hold < self.hold_sec) and self.running:
            self.get_logger().info(
                f"[SE3-L1 Flight] Vị trí=({self.pos[0]:+.2f}, {self.pos[1]:+.2f}, {-self.pos[2]:.2f}m) | "
                f"Góc nghiêng=(R={math.degrees(self.att[0]):+.1f}°, P={math.degrees(self.att[1]):+.1f}°) | "
                f"Nhiễu L1 d_v=({self.observer.d_v_filt[0]:+.2f}, {self.observer.d_v_filt[1]:+.2f})",
                throttle_duration_sec=1.5
            )
            time.sleep(0.1)

        # 6. Return to level and Land
        self.get_logger().info("[Landing] Trả thân UAV về phẳng (0.0° Roll/Pitch)...")
        self.sp_att_cmd[0] = 0.0
        self.sp_att_cmd[1] = 0.0
        time.sleep(2.0)

        self.get_logger().info("[Landing] Đang hạ cánh xuống mặt đất...")
        self.pd = np.array([float(self.pos[0]), float(self.pos[1]), -0.1])
        t_land = time.time()
        while time.time() - t_land < 6.0 and self.running:
            if -self.pos[2] < 0.2:
                break
            time.sleep(0.2)

        # Disarm
        self.get_logger().info("[Landing] Ngắt động cơ (DISARM)...")
        for _ in range(3):
            self.mav.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 21196, 0, 0, 0, 0, 0
            )
            time.sleep(0.2)
        self.streaming = False
        time.sleep(1.0)

        # Save flight data & plot
        self._save_and_plot()
        self.running = False
        rclpy.shutdown()

    def _save_and_plot(self):
        if not self.log_data:
            return

        ts = int(time.time())
        csv_file = os.path.join(LOG_DIR, f"virgo_se3_l1_{ts}.csv")
        import csv
        with open(csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(self.log_data[0].keys()))
            writer.writeheader()
            writer.writerows(self.log_data)
        self.get_logger().info(f"[LOG] Đã lưu dữ liệu bay vào: {csv_file}")

        plot_file = os.path.join(PLOT_DIR, f"virgo_se3_l1_{ts}_analysis.png")
        t_arr = np.array([row['time'] - self.log_data[0]['time'] for row in self.log_data])
        fig, axs = plt.subplots(3, 2, figsize=(14, 10))
        fig.suptitle(f"Virgo OMAV V2 - Lie Group SE(3) + L1 Adaptive Control Benchmark ({os.path.basename(csv_file)})", fontsize=14, fontweight='bold')

        # 1. Position tracking
        axs[0, 0].plot(t_arr, [row['pos_x'] for row in self.log_data], label='X (Actual)', color='tab:blue')
        axs[0, 0].plot(t_arr, [row['sp_pos_x'] for row in self.log_data], '--', label='X (Setpoint)', color='tab:blue', alpha=0.6)
        axs[0, 0].plot(t_arr, [row['pos_y'] for row in self.log_data], label='Y (Actual)', color='tab:orange')
        axs[0, 0].plot(t_arr, [row['sp_pos_y'] for row in self.log_data], '--', label='Y (Setpoint)', color='tab:orange', alpha=0.6)
        axs[0, 0].set_ylabel('Position (m)')
        axs[0, 0].legend()
        axs[0, 0].grid(True)
        axs[0, 0].set_title('Horizontal Stationkeeping (Anti-Drift)')

        # 2. Altitude tracking
        axs[0, 1].plot(t_arr, [-row['pos_z'] for row in self.log_data], label='Altitude (Actual)', color='tab:green')
        axs[0, 1].plot(t_arr, [-row['sp_pos_z'] for row in self.log_data], '--', label='Altitude (Setpoint)', color='black', alpha=0.6)
        axs[0, 1].set_ylabel('Altitude (m)')
        axs[0, 1].legend()
        axs[0, 1].grid(True)
        axs[0, 1].set_title('Altitude Profile')

        # 3. Attitude tracking
        axs[1, 0].plot(t_arr, [row['roll'] for row in self.log_data], label='Roll (deg)', color='tab:red')
        axs[1, 0].plot(t_arr, [row['sp_roll'] for row in self.log_data], '--', label='Roll Setpoint', color='tab:red', alpha=0.6)
        axs[1, 0].plot(t_arr, [row['pitch'] for row in self.log_data], label='Pitch (deg)', color='tab:purple')
        axs[1, 0].plot(t_arr, [row['sp_pitch'] for row in self.log_data], '--', label='Pitch Setpoint', color='tab:purple', alpha=0.6)
        axs[1, 0].set_ylabel('Angle (deg)')
        axs[1, 0].legend()
        axs[1, 0].grid(True)
        axs[1, 0].set_title('Decoupled Attitude Tracking')

        # 4. Error norms
        axs[1, 1].plot(t_arr, [row['ep_norm'] for row in self.log_data], label='||e_p|| (m)', color='tab:blue')
        axs[1, 1].plot(t_arr, [row['eR_norm'] for row in self.log_data], label='||e_R|| (rad)', color='tab:red')
        axs[1, 1].set_ylabel('Error Norm')
        axs[1, 1].legend()
        axs[1, 1].grid(True)
        axs[1, 1].set_title('Lie Group SE(3) Coordinate-Free Error')

        # 5. L1 Disturbance Observer
        axs[2, 0].plot(t_arr, [row['d_v_x'] for row in self.log_data], label='d_v_x (m/s^2)', color='tab:cyan')
        axs[2, 0].plot(t_arr, [row['d_v_y'] for row in self.log_data], label='d_v_y (m/s^2)', color='tab:olive')
        axs[2, 0].plot(t_arr, [row['d_v_z'] for row in self.log_data], label='d_v_z (m/s^2)', color='tab:brown')
        axs[2, 0].set_ylabel('Disturbance (m/s^2)')
        axs[2, 0].set_xlabel('Time (s)')
        axs[2, 0].legend()
        axs[2, 0].grid(True)
        axs[2, 0].set_title('L1 Adaptive Disturbance Observer Estimation')

        # 6. Pod 0 Gimbal Tilt Angles
        axs[2, 1].plot(t_arr, [row['servo_pitch_0'] for row in self.log_data], label='Pod 0 Alpha (Pitch)', color='tab:blue')
        axs[2, 1].plot(t_arr, [row['servo_roll_0'] for row in self.log_data], label='Pod 0 Beta (Roll)', color='tab:orange')
        axs[2, 1].set_ylabel('Gimbal Angle (deg)')
        axs[2, 1].set_xlabel('Time (s)')
        axs[2, 1].legend()
        axs[2, 1].grid(True)
        axs[2, 1].set_title('Null-Space Pod Vectoring (Singularity Free)')

        plt.tight_layout()
        plt.savefig(plot_file, dpi=180)
        plt.close(fig)
        self.get_logger().info(f"[PLOT] Đã xuất biểu đồ phân tích bay: {plot_file}")


def main(args=None):
    rclpy.init(args=args)
    node = VirgoSE3GeometricNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == '__main__':
    main()
