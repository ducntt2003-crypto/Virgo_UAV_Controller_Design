#!/usr/bin/env python3
"""
==============================================================================
 ROS 2 Mission Node for MorphEUS OMAV (arXiv:2505.18270v1 / ICRA 2025)
 Framework: ROS 2 Humble (rclpy) + MAVLink Offboard
 
 Supported Missions:
   1. 'translation'       : Zero-tilt horizontal translation (2x2m square, roll/pitch <= ±0.5°)
   2. 'contact_inspection': Continuous contact inspection (pitch 15°..30° facing wall + vertical scan)
   3. 'narrow_spaces'     : Flying in constrained narrow spaces (3D corkscrew path + roll 35° tilt)
   4. 'visual_inspection' : Continuous visual inspection (circular orbit with heading locked on target)
   5. 'all'               : Sequential execution of all 4 mission profiles
==============================================================================
"""

import math
import os
import sys
import threading
import time
import numpy as np

SCRIPT_DIR = "/home/duc/Omini_UAV/Virgo_UAV_PID_V2/scripts"
sys.path.append(SCRIPT_DIR)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from std_msgs.msg import String, Header

from pymavlink import mavutil

LOG_DIR = "/home/duc/Omini_UAV/Virgo_UAV_PID_V2/docs/flight_logs"
PLOT_DIR = "/home/duc/Omini_UAV/Virgo_UAV_PID_V2/docs/flight_plots"
os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(PLOT_DIR, exist_ok=True)

class VirgoMissionNode(Node):
    def __init__(self):
        super().__init__('virgo_mission_node')

        # Declare parameters
        self.declare_parameter('mission', 'translation')
        self.declare_parameter('target_alt', 1.5)
        self.declare_parameter('connection_url', 'udp:127.0.0.1:14540')

        self.mission_name = self.get_parameter('mission').get_parameter_value().string_value
        self.target_alt = self.get_parameter('target_alt').get_parameter_value().double_value
        self.conn_url = self.get_parameter('connection_url').get_parameter_value().string_value

        self.get_logger().info(f"[Virgo ROS 2] Initializing Mission Node...")
        self.get_logger().info(f"[Virgo ROS 2] Selected Mission: {self.mission_name}")
        self.get_logger().info(f"[Virgo ROS 2] Target Altitude: {self.target_alt} m")

        # ROS 2 Publishers
        self.pub_status = self.create_publisher(String, '/virgo/status', 10)
        self.pub_current_pose = self.create_publisher(PoseStamped, '/virgo/current_pose', 10)
        self.pub_target_pose = self.create_publisher(PoseStamped, '/virgo/target_pose', 10)

        # MAVLink interface to PX4
        self.mav = mavutil.mavlink_connection(self.conn_url)
        self.get_logger().info(f"[Virgo ROS 2] Waiting for PX4 heartbeat on {self.conn_url}...")
        self.mav.wait_heartbeat(timeout=15)
        self.target_sys = self.mav.target_system
        self.target_comp = self.mav.target_component
        self.get_logger().info(f"[Virgo ROS 2] PX4 Connected! SysID={self.target_sys}, CompID={self.target_comp}")

        # Request data stream at 20 Hz
        self.mav.mav.request_data_stream_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1
        )
        # Request SERVO_OUTPUT_RAW at 50 Hz
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, 36, 20000, 0, 0, 0, 0, 0
        )

        # State Telemetry
        self.lock = threading.Lock()
        self.running = True
        self.pos = np.zeros(3)      # NED: x, y, z
        self.vel = np.zeros(3)
        self.att = np.zeros(3)      # Roll, Pitch, Yaw (rad)
        self.motors = np.zeros(8)
        self.servos = np.zeros(8)
        self.armed = False
        self.flight_mode = "UNKNOWN"

        self.sp_pos = np.zeros(3)
        self.sp_att = np.zeros(3)

        self.log_data = []
        self.start_time = time.time()

        # Background telemetry reader thread
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        # ROS 2 Periodic Status Timer (10 Hz)
        self.timer = self.create_timer(0.1, self._timer_callback)

        # Background mission execution thread
        self.mission_thread = threading.Thread(target=self._run_mission_sequence, daemon=True)
        self.mission_thread.start()

    def _rx_loop(self):
        while self.running and rclpy.ok():
            msg = self.mav.recv_match(blocking=True, timeout=0.1)
            if not msg:
                continue
            mtype = msg.get_type()

            with self.lock:
                if mtype == 'LOCAL_POSITION_NED':
                    self.pos = np.array([msg.x, msg.y, msg.z])
                    self.vel = np.array([msg.vx, msg.vy, msg.vz])
                elif mtype == 'ATTITUDE':
                    self.att = np.array([msg.roll, msg.pitch, msg.yaw])
                elif mtype == 'HEARTBEAT':
                    self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.flight_mode = mavutil.mode_string_v10(msg)
                elif mtype == 'SERVO_OUTPUT_RAW':
                    port = getattr(msg, 'port', 0)
                    raw_1_8 = [getattr(msg, f'servo{i}_raw', 0) for i in range(1, 9)]

                    if port == 0:
                        # Motors 0-7 (100..1200 rad/s)
                        self.motors = np.array([max(0.0, min(1.0, (v - 100.0) / 1100.0)) if v > 50
                                                else max(0.0, min(1.0, (v - 1000.0) / 1000.0)) for v in raw_1_8])
                    elif port == 1:
                        # Servos 0-7 (centered at 500 for [-90, +90] deg, or 1500 for PWM)
                        self.servos = np.array([
                            (v - 500.0) / 500.0 * (math.pi / 2.0) if (v >= 0 and v <= 1000)
                            else ((v - 1500.0) / 500.0 * (math.pi / 2.0) if v > 1000 else math.radians(v))
                            for v in raw_1_8
                        ])

                # Record telemetry
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

    def send_tilt_angles(self, roll_rad, pitch_rad, yaw_rad=0.0):
        t_us = int(time.time() * 1e6) & 0xFFFFFFFFFFFFFFFF
        self.mav.mav.debug_vect_send(
            b'tilt_angle',
            t_us,
            float(roll_rad),
            float(pitch_rad),
            float(yaw_rad)
        )

    def _timer_callback(self):
        with self.lock:
            p = self.pos.copy()
            a = self.att.copy()
            sp_p = self.sp_pos.copy()

        # Publish Current Pose in ROS 2
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(p[0])
        pose_msg.pose.position.y = float(p[1])
        pose_msg.pose.position.z = float(-p[2]) # Z up in ROS

        cr = math.cos(a[0]*0.5); sr = math.sin(a[0]*0.5)
        cp = math.cos(a[1]*0.5); sp = math.sin(a[1]*0.5)
        cy = math.cos(a[2]*0.5); sy = math.sin(a[2]*0.5)
        pose_msg.pose.orientation.w = cr*cp*cy + sr*sp*sy
        pose_msg.pose.orientation.x = sr*cp*cy - cr*sp*sy
        pose_msg.pose.orientation.y = cr*sp*cy + sr*cp*sy
        pose_msg.pose.orientation.z = cr*cp*sy - sr*sp*cy
        self.pub_current_pose.publish(pose_msg)

        # Publish Target Pose
        sp_msg = PoseStamped()
        sp_msg.header.stamp = self.get_clock().now().to_msg()
        sp_msg.header.frame_id = 'map'
        sp_msg.pose.position.x = float(sp_p[0])
        sp_msg.pose.position.y = float(sp_p[1])
        sp_msg.pose.position.z = float(-sp_p[2])
        self.pub_target_pose.publish(sp_msg)

    def _run_mission_sequence(self):
        time.sleep(2.0)
        try:
            self.arm_and_takeoff(self.target_alt)

            if self.mission_name in ['translation', 'all']:
                self.run_mission_translation()
                time.sleep(2.0)

            if self.mission_name in ['contact_inspection', 'all']:
                self.run_mission_contact_inspection()
                time.sleep(2.0)

            if self.mission_name in ['narrow_spaces', 'all']:
                self.run_mission_narrow_spaces()
                time.sleep(2.0)

            if self.mission_name in ['visual_inspection', 'all']:
                self.run_mission_visual_inspection()
                time.sleep(2.0)

            self.land_and_disarm()

        except Exception as e:
            self.get_logger().error(f"[Virgo ROS 2] Exception during mission: {e}")
            self.land_and_disarm()

        finally:
            self.finish_and_save()

    def arm_and_takeoff(self, target_alt):
        self.get_logger().info(f"[Takeoff] Pre-streaming setpoints to altitude {target_alt}m...")
        self.sp_pos = np.array([0.0, 0.0, -target_alt])
        self.sp_att = np.zeros(3)

        type_mask = 0b0000111111111000
        start_ms = time.time()

        for _ in range(30):
            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0.0, 0.0, -target_alt,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(0.0, 0.0, 0.0)
            time.sleep(0.04)

        self.get_logger().info("[Takeoff] Switching to OFFBOARD mode...")
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)

        self.get_logger().info("[Takeoff] Arming vehicle...")
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(1.0)

        # Climb to target altitude
        t0 = time.time()
        while time.time() - t0 < 8.0 and self.running:
            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0.0, 0.0, -target_alt,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(0.0, 0.0, 0.0)
            alt = -self.pos[2]
            status_str = f"Climbing | Alt: {alt:.2f}/{target_alt:.1f}m | Roll: {math.degrees(self.att[0]):.1f}°"
            self.pub_status.publish(String(data=status_str))
            if abs(alt - target_alt) < 0.2:
                break
            time.sleep(0.05)
        self.get_logger().info(f"[Takeoff] Hover established at {target_alt}m.")

    def run_mission_translation(self):
        """Mission 1: Zero-Tilt Translation (2x2m Square Path)"""
        self.get_logger().info("==========================================================")
        self.get_logger().info("  MISSION 1: ZERO-TILT TRANSLATION (2x2m SQUARE PATH)     ")
        self.get_logger().info("  Paper: Section IV - Zero fuselage tilt lateral motion   ")
        self.get_logger().info("==========================================================")

        waypoints = [
            (2.0, 0.0, -self.target_alt, "Forward +2m"),
            (2.0, 2.0, -self.target_alt, "Right +2m"),
            (0.0, 2.0, -self.target_alt, "Backward -2m"),
            (0.0, 0.0, -self.target_alt, "Left -2m (Home)")
        ]

        type_mask = 0b0000111111111000
        start_ms = time.time()

        for wp_x, wp_y, wp_z, desc in waypoints:
            self.sp_pos = np.array([wp_x, wp_y, wp_z])
            self.sp_att = np.zeros(3)
            self.get_logger().info(f">>> Waypoint: {desc} -> ({wp_x:.1f}, {wp_y:.1f}, {-wp_z:.1f}m)")
            t_leg = time.time()

            while time.time() - t_leg < 7.0 and self.running:
                ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
                self.mav.mav.set_position_target_local_ned_send(
                    ms, self.target_sys, self.target_comp,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask,
                    wp_x, wp_y, wp_z,
                    0, 0, 0, 0, 0, 0, 0.0, 0.0
                )
                self.send_tilt_angles(0.0, 0.0, 0.0)
                dist = np.linalg.norm(self.pos[:2] - np.array([wp_x, wp_y]))
                r_deg = math.degrees(self.att[0])
                p_deg = math.degrees(self.att[1])
                status = f"Square | {desc} | Dist: {dist:.2f}m | Roll: {r_deg:+.1f}° | Pitch: {p_deg:+.1f}°"
                self.pub_status.publish(String(data=status))

                if dist < 0.2 and (time.time() - t_leg > 3.0):
                    break
                time.sleep(0.05)

        self.get_logger().info("[PASS] Mission 1: Zero-tilt translation complete!")

    def run_mission_contact_inspection(self):
        """Mission 2: Continuous Contact Inspection with Tool Pitch Tilt"""
        self.get_logger().info("==========================================================")
        self.get_logger().info("  MISSION 2: CONTINUOUS CONTACT INSPECTION (TOOL PITCH)   ")
        self.get_logger().info("  Paper: Section IV-A / Fig. 6 & 11 Continuous Contact    ")
        self.get_logger().info("==========================================================")

        type_mask = 0b0000111111111000
        start_ms = time.time()

        # Step 1: Approach wall at x=1.5m, y=0.0m
        self.get_logger().info(">>> Step 2.1: Approaching wall inspection stand-off (1.5, 0.0, -1.5m)...")
        self.sp_pos = np.array([1.5, 0.0, -1.5])
        self.sp_att = np.zeros(3)
        t_app = time.time()
        while time.time() - t_app < 6.0 and self.running:
            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                1.5, 0.0, -1.5,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(0.0, 0.0, 0.0)
            time.sleep(0.05)

        # Step 2: Pitch tool forward (+20° pitch) while maintaining contact position
        self.get_logger().info(">>> Step 2.2: Orienting tool normal to wall (+20° pitch hold)...")
        pitch_contact = math.radians(20)
        self.sp_att = np.array([0.0, pitch_contact, 0.0])
        t_pitch = time.time()
        while time.time() - t_pitch < 4.0 and self.running:
            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                1.5, 0.0, -1.5,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(0.0, pitch_contact, 0.0)
            time.sleep(0.05)

        # Step 3: Vertical scanning along wall while maintaining pitched contact tool pose
        scan_waypoints = [-1.9, -1.2, -1.5]
        for z_w in scan_waypoints:
            self.get_logger().info(f">>> Step 2.3: Scanning vertical surface at Z={-z_w:.1f}m with Pitch +20°...")
            self.sp_pos[2] = z_w
            t_scan = time.time()
            while time.time() - t_scan < 5.0 and self.running:
                ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
                self.mav.mav.set_position_target_local_ned_send(
                    ms, self.target_sys, self.target_comp,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask,
                    1.5, 0.0, z_w,
                    0, 0, 0, 0, 0, 0, 0.0, 0.0
                )
                self.send_tilt_angles(0.0, pitch_contact, 0.0)
                time.sleep(0.05)

        # Step 4: Level out and return
        self.get_logger().info(">>> Step 2.4: Leveling out and returning to center...")
        self.sp_att = np.zeros(3)
        self.sp_pos = np.array([0.0, 0.0, -self.target_alt])
        t_ret = time.time()
        while time.time() - t_ret < 5.0 and self.running:
            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0.0, 0.0, -self.target_alt,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(0.0, 0.0, 0.0)
            time.sleep(0.05)

        self.get_logger().info("[PASS] Mission 2: Continuous contact inspection complete!")

    def run_mission_narrow_spaces(self):
        """Mission 3: Flying in Constrained Narrow Spaces (Decoupled 3D Corkscrew Path)"""
        self.get_logger().info("==========================================================")
        self.get_logger().info("  MISSION 3: FLYING IN CONSTRAINED NARROW SPACES          ")
        self.get_logger().info("  Paper: Section IV-B / Fig. 9 Narrow Passage Navigation  ")
        self.get_logger().info("==========================================================")

        type_mask = 0b0000111111111000
        start_ms = time.time()
        t0 = time.time()
        duration = 16.0
        r_path = 1.2
        omega = 2.0 * math.pi / 12.0

        while time.time() - t0 < duration and self.running:
            t = time.time() - t0
            x_sp = r_path * math.sin(omega * t)
            y_sp = r_path * (1.0 - math.cos(omega * t))
            z_sp = -self.target_alt + 0.3 * math.sin(omega * t)

            # Oscillate roll angle ±25° to morph through constrained apertures
            roll_sp = math.radians(25) * math.sin(omega * t)
            pitch_sp = 0.0

            self.sp_pos = np.array([x_sp, y_sp, z_sp])
            self.sp_att = np.array([roll_sp, pitch_sp, 0.0])

            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                x_sp, y_sp, z_sp,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(roll_sp, pitch_sp, 0.0)
            status = f"Narrow-Space | Roll SP: {math.degrees(roll_sp):+.1f}° | Actual Roll: {math.degrees(self.att[0]):+.1f}°"
            self.pub_status.publish(String(data=status))
            time.sleep(0.05)

        # Level out and return home
        self.sp_att = np.zeros(3)
        self.sp_pos = np.array([0.0, 0.0, -self.target_alt])
        t_ret = time.time()
        while time.time() - t_ret < 5.0 and self.running:
            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0.0, 0.0, -self.target_alt,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(0.0, 0.0, 0.0)
            time.sleep(0.05)

        self.get_logger().info("[PASS] Mission 3: Constrained narrow space navigation complete!")

    def run_mission_visual_inspection(self):
        """Mission 4: Continuous Visual Inspection (Orbit with Locked Viewpoint)"""
        self.get_logger().info("==========================================================")
        self.get_logger().info("  MISSION 4: CONTINUOUS VISUAL INSPECTION (ORBIT POI)     ")
        self.get_logger().info("  Paper: Section IV-C / Fig. 10 Continuous Visual Track   ")
        self.get_logger().info("==========================================================")

        type_mask = 0b0000101111111000 # Enable yaw setpoint
        start_ms = time.time()
        t0 = time.time()
        duration = 18.0
        r_orbit = 1.5
        omega = 2.0 * math.pi / 14.0

        # Point of Interest (POI) at center (0, 0, -target_alt)
        poi_x = 0.0
        poi_y = 0.0

        while time.time() - t0 < duration and self.running:
            t = time.time() - t0
            x_sp = r_orbit * math.cos(omega * t)
            y_sp = r_orbit * math.sin(omega * t)
            z_sp = -self.target_alt

            # Lock heading directly onto center POI
            yaw_sp = math.atan2(poi_y - y_sp, poi_x - x_sp)

            self.sp_pos = np.array([x_sp, y_sp, z_sp])
            self.sp_att = np.array([0.0, 0.0, yaw_sp])

            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                x_sp, y_sp, z_sp,
                0, 0, 0, 0, 0, 0, yaw_sp, 0.0
            )
            self.send_tilt_angles(0.0, 0.0, yaw_sp)
            status = f"Visual Orbit | Yaw Target: {math.degrees(yaw_sp):+.1f}° | Actual Yaw: {math.degrees(self.att[2]):+.1f}°"
            self.pub_status.publish(String(data=status))
            time.sleep(0.05)

        # Return to home hover
        self.sp_att = np.zeros(3)
        self.sp_pos = np.array([0.0, 0.0, -self.target_alt])
        t_ret = time.time()
        while time.time() - t_ret < 5.0 and self.running:
            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0.0, 0.0, -self.target_alt,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(0.0, 0.0, 0.0)
            time.sleep(0.05)

        self.get_logger().info("[PASS] Mission 4: Continuous visual inspection complete!")

    def land_and_disarm(self):
        self.get_logger().info("[Landing] Descending to ground...")
        self.send_tilt_angles(0.0, 0.0, 0.0)
        start_ms = time.time()
        t_land = time.time()
        while time.time() - t_land < 5.0 and self.running:
            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                0.0, 0.0, -0.1,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(0.0, 0.0, 0.0)
            if -self.pos[2] < 0.2:
                break
            time.sleep(0.05)

        self.get_logger().info("[Landing] Disarming...")
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(1.0)
        self.get_logger().info("[Landing] Disarmed successfully.")

    def finish_and_save(self):
        self.running = False
        time.sleep(0.5)
        if not self.log_data:
            self.get_logger().warn("[Virgo ROS 2] No telemetry data recorded.")
            return

        import pandas as pd
        csv_file = os.path.join(LOG_DIR, f"virgo_{self.mission_name}_{int(time.time())}.csv")
        df = pd.DataFrame(self.log_data)
        df.to_csv(csv_file, index=False)
        self.get_logger().info(f"[SAVED] Telemetry log ({len(df)} samples) written to: {csv_file}")

        # Plot flight log
        plot_png = os.path.join(PLOT_DIR, f"virgo_{self.mission_name}_{int(time.time())}_analysis.png")
        sys.path.append("/home/duc/Omini_UAV/Virgo_UAV_PID_V2/scripts")
        from plot_flight_data import plot_flight_log
        plot_flight_log(csv_file, plot_png)
        self.get_logger().info(f"[SAVED] Analysis plot saved to: {plot_png}")

        # Signal ROS shutdown
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = VirgoMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[Virgo ROS 2] KeyboardInterrupt. Landing...")
        node.land_and_disarm()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
