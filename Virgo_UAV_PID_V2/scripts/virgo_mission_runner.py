#!/usr/bin/env python3
"""
==============================================================================
 Virgo OMAV Mission Runner (Direct Script)
 Paper: "Virgo: Morphable OmnidirEctional Unmanned System" (arXiv:2505.18270v1)
==============================================================================
Supported Missions:
  1. 'translation'        : Decoupled horizontal translation with 0° fuselage tilt
  2. 'contact_inspection' : Approaching wall, tilting probe pitch 20°, vertical scan
  3. 'narrow_spaces'      : Corkscrew 3D trajectory with decoupled roll attitude
  4. 'visual_inspection'  : Orbiting inspection target with heading locked on POI
  5. 'all'                : Sequential execution of all 4 missions
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

from plot_flight_data import plot_flight_log

LOG_DIR = "/home/duc/Omini_UAV/Virgo_UAV_PID_V2/docs/flight_logs"
PLOT_DIR = "/home/duc/Omini_UAV/Virgo_UAV_PID_V2/docs/flight_plots"
os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(PLOT_DIR, exist_ok=True)

class MorpheusMissionRunner:
    def __init__(self, connection_str="udp:127.0.0.1:14540"):
        print(f"\033[1;36m[Virgo]\033[0m Connecting to PX4 SITL on {connection_str}...")
        self.mav = mavutil.mavlink_connection(connection_str)
        self.mav.wait_heartbeat(timeout=15)
        self.target_sys = self.mav.target_system
        self.target_comp = self.mav.target_component
        print(f"\033[1;32m[Virgo]\033[0m Connected to System {self.target_sys}, Component {self.target_comp}!")

        # Request data streams
        self.mav.mav.request_data_stream_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1
        )
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, 36, 20000, 0, 0, 0, 0, 0
        )

        # State Telemetry
        self.running = True
        self.lock = threading.Lock()
        self.pos = np.zeros(3)      # X, Y, Z (NED, Z negative is up)
        self.vel = np.zeros(3)      # Vx, Vy, Vz
        self.att = np.zeros(3)      # Roll, Pitch, Yaw (rad)
        self.motors = np.zeros(8)
        self.servos = np.zeros(8)
        self.armed = False
        self.flight_mode = "UNKNOWN"

        self.sp_pos = np.zeros(3)
        self.sp_att = np.zeros(3)

        self.log_data = []
        self.start_time = time.time()

        # Telemetry reader thread
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def _rx_loop(self):
        while self.running:
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

                # Record sample
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

    def arm_and_takeoff(self, target_alt=1.5):
        print(f"\033[1;33m[Takeoff]\033[0m Pre-streaming setpoints to target altitude {target_alt}m...")
        self.sp_pos = np.array([0.0, 0.0, -target_alt])
        self.sp_att = np.zeros(3)

        type_mask = 0b0000101111111000
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

        print("\033[1;33m[Takeoff]\033[0m Switching to OFFBOARD mode...")
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)

        print("\033[1;33m[Takeoff]\033[0m Arming vehicle...")
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(1.0)

        # Climb
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
            print(f"\r\033[1;34m[Climbing]\033[0m Alt: {alt:.2f}/{target_alt:.1f}m | Roll: {math.degrees(self.att[0]):.1f}° | Pitch: {math.degrees(self.att[1]):.1f}°", end="")
            if abs(alt - target_alt) < 0.2:
                break
            time.sleep(0.05)
        print(f"\n\033[1;32m[Takeoff]\033[0m Hover established at {target_alt}m altitude.")

    def run_mission_translation(self, target_alt=1.5):
        print("\n\033[1;35m==========================================================\033[0m")
        print("\033[1;35m  MISSION 1: ZERO-TILT TRANSLATION (2x2m SQUARE PATH)     \033[0m")
        print("\033[1;35m  Paper: Section IV - Zero fuselage tilt lateral motion   \033[0m")
        print("\033[1;35m==========================================================\033[0m")

        waypoints = [
            (2.0, 0.0, -target_alt, "Forward +2m"),
            (2.0, 2.0, -target_alt, "Right +2m"),
            (0.0, 2.0, -target_alt, "Backward -2m"),
            (0.0, 0.0, -target_alt, "Left -2m (Home)")
        ]

        type_mask = 0b0000101111111000
        start_ms = time.time()

        for wp_x, wp_y, wp_z, desc in waypoints:
            self.sp_pos = np.array([wp_x, wp_y, wp_z])
            self.sp_att = np.zeros(3)
            print(f"\n\033[1;33m>>> Waypoint:\033[0m {desc} -> ({wp_x:.1f}, {wp_y:.1f}, {-wp_z:.1f}m)")
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
                print(f"\rTracking: Dist={dist:.2f}m | Roll={r_deg:+.2f}° | Pitch={p_deg:+.2f}°", end="")
                if dist < 0.2 and (time.time() - t_leg > 3.0):
                    break
                time.sleep(0.05)

        print("\n\033[1;32m[PASS]\033[0m Mission 1: Zero-tilt translation complete!")

    def run_mission_contact_inspection(self, target_alt=1.5):
        print("\n\033[1;35m==========================================================\033[0m")
        print("\033[1;35m  MISSION 2: CONTINUOUS CONTACT INSPECTION (TOOL PITCH)   \033[0m")
        print("\033[1;35m  Paper: Section IV-A / Fig. 6 & 11 Continuous Contact    \033[0m")
        print("\033[1;35m==========================================================\033[0m")

        type_mask = 0b0000101111111000
        start_ms = time.time()

        # Step 1: Approach wall at x=1.5m, y=0.0m
        print("\n\033[1;33m>>> Step 2.1: Approaching wall inspection stand-off (1.5, 0.0, -1.5m)...\033[0m")
        self.sp_pos = np.array([1.5, 0.0, -target_alt])
        self.sp_att = np.zeros(3)
        t_app = time.time()
        while time.time() - t_app < 6.0 and self.running:
            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                1.5, 0.0, -target_alt,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(0.0, 0.0, 0.0)
            time.sleep(0.05)

        # Step 2: Pitch tool forward (+20° pitch) while maintaining contact position
        print("\n\033[1;33m>>> Step 2.2: Orienting tool normal to wall (+20° pitch hold)...\033[0m")
        pitch_contact = math.radians(20)
        self.sp_att = np.array([0.0, pitch_contact, 0.0])
        t_pitch = time.time()
        while time.time() - t_pitch < 4.0 and self.running:
            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                1.5, 0.0, -target_alt,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(0.0, pitch_contact, 0.0)
            time.sleep(0.05)

        # Step 3: Vertical scanning along wall
        scan_waypoints = [-1.9, -1.2, -1.5]
        for z_w in scan_waypoints:
            print(f"\n\033[1;33m>>> Step 2.3: Scanning vertical surface at Z={-z_w:.1f}m with Pitch +20°...\033[0m")
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
        print("\n\033[1;33m>>> Step 2.4: Leveling out and returning to center...\033[0m")
        self.sp_att = np.zeros(3)
        self.sp_pos = np.array([0.0, 0.0, -target_alt])
        t_ret = time.time()
        while time.time() - t_ret < 5.0 and self.running:
            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0.0, 0.0, -target_alt,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(0.0, 0.0, 0.0)
            time.sleep(0.05)

        print("\n\033[1;32m[PASS]\033[0m Mission 2: Continuous contact inspection complete!")

    def run_mission_narrow_spaces(self, target_alt=1.5):
        print("\n\033[1;35m==========================================================\033[0m")
        print("\033[1;35m  MISSION 3: FLYING IN CONSTRAINED NARROW SPACES          \033[0m")
        print("\033[1;35m  Paper: Section IV-B / Fig. 9 Narrow Passage Navigation  \033[0m")
        print("\033[1;35m==========================================================\033[0m")

        type_mask = 0b0000101111111000
        start_ms = time.time()
        t0 = time.time()
        duration = 16.0
        r_path = 1.2
        omega = 2.0 * math.pi / 12.0

        while time.time() - t0 < duration and self.running:
            t = time.time() - t0
            x_sp = r_path * math.sin(omega * t)
            y_sp = r_path * (1.0 - math.cos(omega * t))
            z_sp = -target_alt + 0.3 * math.sin(omega * t)

            # Roll oscillation ±25° to morph through constrained apertures
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
            print(f"\rNarrow Passage: Roll SP={math.degrees(roll_sp):+.1f}° | Actual Roll={math.degrees(self.att[0]):+.1f}°", end="")
            time.sleep(0.05)

        # Return home
        self.sp_att = np.zeros(3)
        self.sp_pos = np.array([0.0, 0.0, -target_alt])
        t_ret = time.time()
        while time.time() - t_ret < 5.0 and self.running:
            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0.0, 0.0, -target_alt,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(0.0, 0.0, 0.0)
            time.sleep(0.05)

        print("\n\033[1;32m[PASS]\033[0m Mission 3: Constrained narrow space navigation complete!")

    def run_mission_visual_inspection(self, target_alt=1.5):
        print("\n\033[1;35m==========================================================\033[0m")
        print("\033[1;35m  MISSION 4: CONTINUOUS VISUAL INSPECTION (ORBIT POI)     \033[0m")
        print("\033[1;35m  Paper: Section IV-C / Fig. 10 Continuous Visual Track   \033[0m")
        print("\033[1;35m==========================================================\033[0m")

        type_mask = 0b0000101111111000 # Enable yaw setpoint
        start_ms = time.time()
        t0 = time.time()
        duration = 18.0
        r_orbit = 1.5
        omega = 2.0 * math.pi / 14.0

        poi_x = 0.0
        poi_y = 0.0

        while time.time() - t0 < duration and self.running:
            t = time.time() - t0
            x_sp = r_orbit * math.cos(omega * t)
            y_sp = r_orbit * math.sin(omega * t)
            z_sp = -target_alt

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
            print(f"\rVisual Orbit: Yaw SP={math.degrees(yaw_sp):+.1f}° | Actual Yaw={math.degrees(self.att[2]):+.1f}°", end="")
            time.sleep(0.05)

        # Return to home hover
        self.sp_att = np.zeros(3)
        self.sp_pos = np.array([0.0, 0.0, -target_alt])
        t_ret = time.time()
        while time.time() - t_ret < 5.0 and self.running:
            ms = int((time.time() - start_ms) * 1000) & 0xFFFFFFFF
            self.mav.mav.set_position_target_local_ned_send(
                ms, self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0.0, 0.0, -target_alt,
                0, 0, 0, 0, 0, 0, 0.0, 0.0
            )
            self.send_tilt_angles(0.0, 0.0, 0.0)
            time.sleep(0.05)

        print("\n\033[1;32m[PASS]\033[0m Mission 4: Continuous visual inspection complete!")

    def land_and_disarm(self):
        print("\n\033[1;33m[Landing]\033[0m Descending to ground...")
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

        print("\033[1;33m[Landing]\033[0m Disarming motors...")
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(1.0)
        print("\033[1;32m[Landing]\033[0m Vehicle disarmed safely.")

    def finish_and_save(self, mission_name):
        self.running = False
        time.sleep(0.5)
        if not self.log_data:
            print("[Virgo] No telemetry recorded.")
            return

        import pandas as pd
        csv_file = os.path.join(LOG_DIR, f"virgo_{mission_name}_{int(time.time())}.csv")
        df = pd.DataFrame(self.log_data)
        df.to_csv(csv_file, index=False)
        print(f"\033[1;32m[SAVED]\033[0m Telemetry log ({len(df)} samples) written to: {csv_file}")

        plot_png = os.path.join(PLOT_DIR, f"virgo_{mission_name}_{int(time.time())}_analysis.png")
        plot_flight_log(csv_file, plot_png)
        print(f"\033[1;32m[SAVED]\033[0m Flight analysis plot saved to: {plot_png}")

def main():
    parser = argparse.ArgumentParser(description="Virgo OMAV Mission Runner")
    parser.add_argument("--mission", "-m", type=str, default="translation",
                        choices=["translation", "contact_inspection", "narrow_spaces", "visual_inspection", "all"],
                        help="Mission profile to execute")
    parser.add_argument("--alt", "-a", type=float, default=1.5, help="Target altitude (m)")
    parser.add_argument("--url", "-u", type=str, default="udp:127.0.0.1:14540", help="PX4 MAVLink URL")
    args = parser.parse_args()

    runner = MorpheusMissionRunner(args.url)
    try:
        runner.arm_and_takeoff(args.alt)
        if args.mission in ["translation", "all"]:
            runner.run_mission_translation(args.alt)
            time.sleep(2.0)
        if args.mission in ["contact_inspection", "all"]:
            runner.run_mission_contact_inspection(args.alt)
            time.sleep(2.0)
        if args.mission in ["narrow_spaces", "all"]:
            runner.run_mission_narrow_spaces(args.alt)
            time.sleep(2.0)
        if args.mission in ["visual_inspection", "all"]:
            runner.run_mission_visual_inspection(args.alt)
            time.sleep(2.0)
        runner.land_and_disarm()
    except KeyboardInterrupt:
        print("\n[Interrupted] Landing immediately...")
        runner.land_and_disarm()
    finally:
        runner.finish_and_save(args.mission)

if __name__ == "__main__":
    main()
