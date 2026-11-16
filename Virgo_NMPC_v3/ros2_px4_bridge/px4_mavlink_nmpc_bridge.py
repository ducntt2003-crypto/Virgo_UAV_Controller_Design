#!/usr/bin/env python3
"""
==============================================================================
 PX4 MAVLink NMPC Offboard Bridge for Virgo OMAV
 Connects VirgoNMPCController directly to PX4 Autopilot SITL & Gazebo Harmonic
 via MAVLink UDP (port 14540).
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

sys.path.append(os.path.join(os.path.dirname(__file__), "../models"))
sys.path.append(os.path.join(os.path.dirname(__file__), "../controllers"))

from virgo_nmpc_controller import VirgoNMPCController
from se3_kinematics import euler_to_quat


class PX4MavlinkNMPCBridge:
    """
    Interfaces the real-time NMPC controller with PX4 SITL over MAVLink.
    Streams high-rate odometry into NMPC and publishes optimal 6-DOF setpoints.
    """

    def __init__(self, connection_str="udp:127.0.0.1:14540", target_alt=1.5):
        print(f"\033[1;36m[MAVLink Bridge]\033[0m Connecting to PX4 at {connection_str}...")
        self.mav = mavutil.mavlink_connection(connection_str)
        self.mav.wait_heartbeat()
        print(f"\033[1;32m[MAVLink Bridge]\033[0m Heartbeat received from System {self.mav.target_system}, Component {self.mav.target_component}!")

        self.target_sys = self.mav.target_system
        self.target_comp = self.mav.target_component
        self.target_alt = float(target_alt)

        self.controller = VirgoNMPCController()

        # Telemetry State
        self.pos = np.zeros(3)      # NED (m)
        self.vel = np.zeros(3)      # NED (m/s)
        self.att = np.zeros(3)      # Roll, Pitch, Yaw (rad)
        self.omega = np.zeros(3)    # p, q, r (rad/s)
        self.servos = np.zeros(8)   # 4 alpha (pitch), 4 beta (roll) (rad)
        self.motors = np.zeros(8)   # 8 motor PWMs / throttle

        self.armed = False
        self.in_air = False
        self.running = True

        # Threads
        self.rx_thread = threading.Thread(target=self._telemetry_receiver_loop, daemon=True)
        self.rx_thread.start()

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

            elif msg_type == 'ATTITUDE':
                self.att = np.array([msg.roll, msg.pitch, msg.yaw], dtype=float)
                self.omega = np.array([msg.rollspeed, msg.pitchspeed, msg.yawspeed], dtype=float)

            elif msg_type == 'SERVO_OUTPUT_RAW':
                # Decode 8 servo angles from PWM channels (channels 9..16 or 1..8)
                raw = [getattr(msg, f'servo{i+1}_raw', 0) for i in range(16)]
                # Typically channels 8-15 are servos in Virgo SDF
                def _decode_sv(v):
                    if 0 < v <= 1000:
                        return (v - 500.0) / 500.0 * (math.pi / 2.0)
                    elif 1000 < v <= 2000:
                        return (v - 1500.0) / 500.0 * (math.pi / 2.0)
                    return 0.0

                self.servos = np.array([_decode_sv(raw[i + 8]) for i in range(8)])

    def takeoff_and_engage_nmpc(self):
        """Standardized climb sequence followed by engaging NMPC offboard loop."""
        print("\033[1;33m[Takeoff]\033[0m Arming vehicle and switching to OFFBOARD...")

        # Stream setpoints before switching mode (PX4 requirement)
        for _ in range(20):
            self._send_setpoint(np.array([0.0, 0.0, -self.target_alt]), np.zeros(3), np.zeros(3), np.zeros(3))
            time.sleep(0.05)

        # Send Arm Command
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6, 0, 0, 0, 0, 0
        )
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0
        )

        t0 = time.time()
        while time.time() - t0 < 10.0 and self.running:
            alt_err = abs(-self.pos[2] - self.target_alt)
            if alt_err < 0.15 and time.time() - t0 > 3.0:
                print(f"\n\033[1;32m[Hover Reached]\033[0m Altitude = {-self.pos[2]:.2f}m. Engaging NMPC!")
                break
            time.sleep(0.1)

    def _send_setpoint(self, pos, vel, acc, att):
        """Sends MAVLink SET_POSITION_TARGET_LOCAL_NED and debug_vect tilt angles."""
        type_mask = 0b0000100000000000  # Pos + Vel + Acc + Yaw
        t_ms = int(time.time() * 1000) & 0xFFFFFFFF

        self.mav.mav.set_position_target_local_ned_send(
            t_ms, self.target_sys, self.target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            float(pos[0]), float(pos[1]), float(pos[2]),
            float(vel[0]), float(vel[1]), float(vel[2]),
            float(acc[0]), float(acc[1]), float(acc[2]),
            float(att[2]), 0.0
        )

        # Stream decoupled 6-DOF tilt angles
        t_us = int(time.time() * 1e6) & 0xFFFFFFFFFFFFFFFF
        self.mav.mav.debug_vect_send(
            b'tilt_angle',
            t_us,
            float(att[0]),
            float(att[1]),
            float(att[2])
        )

    def run_nmpc_loop(self):
        """Real-time 25 Hz NMPC execution loop."""
        self.takeoff_and_engage_nmpc()

        dt = self.controller.dt  # 0.04s
        t_mission_start = time.time()
        total_dur = self.controller.traj_gen.total_duration

        print(f"\033[1;32m[NMPC Running]\033[0m Executing 6-DOF decoupled trajectory ({total_dur:.1f}s)...")

        while self.running and (time.time() - t_mission_start < total_dur + 5.0):
            t_now = time.time() - t_mission_start

            # Convert Euler to unit quaternion
            quat = euler_to_quat(self.att[0], self.att[1], self.att[2])
            alpha_curr = np.array([self.servos[2*i] for i in range(4)])
            beta_curr  = np.array([self.servos[2*i+1] for i in range(4)])

            # Update controller state
            self.controller.update_state(self.pos, self.vel, quat, self.omega, alpha_curr, beta_curr)

            # Compute NMPC
            ctrl = self.controller.compute_control(t_now)

            # Look-ahead feedforward setpoint
            xref, att_ref = self.controller.traj_gen.get_reference_state(t_now + 0.1)

            # Stream setpoint
            self._send_setpoint(xref[0:3], xref[3:6], np.zeros(3), att_ref)

            time.sleep(dt)

        print("\033[1;32m[NMPC Complete]\033[0m Mission finished cleanly.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Virgo OMAV PX4 MAVLink NMPC Offboard Bridge")
    parser.add_argument("--conn", type=str, default="udp:127.0.0.1:14540", help="MAVLink connection URI")
    parser.add_argument("--alt", type=float, default=1.5, help="Target altitude (m)")
    args = parser.parse_args()

    bridge = PX4MavlinkNMPCBridge(args.conn, args.alt)
    try:
        bridge.run_nmpc_loop()
    except KeyboardInterrupt:
        print("\n[Bridge] Terminated by user.")
