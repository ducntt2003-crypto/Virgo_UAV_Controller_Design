#!/usr/bin/env python3
"""
==============================================================================
 ROS 2 Humble NMPC Offboard Node for Virgo OMAV
 Interfaces with PX4 Autopilot v1.15 via Micro-XRCE-DDS Agent.
 Subscribes: /fmu/out/vehicle_odometry
 Publishes:  /fmu/in/trajectory_setpoint, /fmu/in/offboard_control_mode
==============================================================================
"""

import math
import os
import sys
import numpy as np

# Ensure controllers and models are in sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), "../models"))
sys.path.append(os.path.join(os.path.dirname(__file__), "../controllers"))

from virgo_nmpc_controller import VirgoNMPCController
from se3_kinematics import quat_to_euler

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from px4_msgs.msg import (
        VehicleOdometry,
        TrajectorySetpoint,
        OffboardControlMode,
        VehicleCommand
    )
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


if ROS2_AVAILABLE:
    class VirgoNMPCROS2Node(Node):
        def __init__(self):
            super().__init__('virgo_nmpc_node')

            # QoS profile matching PX4 Micro-XRCE-DDS topics
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )

            # Publishers to PX4
            self.offboard_mode_pub = self.create_publisher(
                OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
            self.trajectory_setpoint_pub = self.create_publisher(
                TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
            self.vehicle_command_pub = self.create_publisher(
                VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

            # Subscribers from PX4
            self.odom_sub = self.create_subscription(
                VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile)

            # Controller
            self.controller = VirgoNMPCController()
            self.dt = self.controller.dt

            # Internal state
            self.pos = np.zeros(3)
            self.vel = np.zeros(3)
            self.quat = np.array([1.0, 0.0, 0.0, 0.0])
            self.omega = np.zeros(3)
            self.servos = np.zeros(8)
            self.has_odom = False

            self.t_start = self.get_clock().now().nanoseconds * 1e-9

            # 25 Hz Control Loop Timer
            self.timer = self.create_timer(self.dt, self.control_loop)
            self.get_logger().info('Virgo OMAV SE(3) NMPC ROS 2 Node initialized at 25 Hz.')

        def odometry_callback(self, msg: VehicleOdometry):
            self.pos = np.array([msg.position[0], msg.position[1], msg.position[2]], dtype=float)
            self.vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]], dtype=float)
            # PX4 VehicleOdometry quaternion is [qw, qx, qy, qz]
            self.quat = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]], dtype=float)
            self.omega = np.array([msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]], dtype=float)
            self.has_odom = True

        def publish_offboard_mode(self):
            msg = OffboardControlMode()
            msg.position = True
            msg.velocity = True
            msg.acceleration = True
            msg.attitude = False
            msg.body_rate = False
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.offboard_mode_pub.publish(msg)

        def control_loop(self):
            self.publish_offboard_mode()
            if not self.has_odom:
                return

            t_now = (self.get_clock().now().nanoseconds * 1e-9) - self.t_start
            alpha_curr = np.array([self.servos[2*i] for i in range(4)])
            beta_curr = np.array([self.servos[2*i+1] for i in range(4)])

            # Update NMPC state
            self.controller.update_state(self.pos, self.vel, self.quat, self.omega, alpha_curr, beta_curr)

            # Compute NMPC
            ctrl = self.controller.compute_control(t_now)

            # Get lookahead feedforward setpoint
            xref, att_ref = self.controller.traj_gen.get_reference_state(t_now + 0.1)

            # Publish TrajectorySetpoint to PX4
            sp = TrajectorySetpoint()
            sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            sp.position = [float(xref[0]), float(xref[1]), float(xref[2])]
            sp.velocity = [float(xref[3]), float(xref[4]), float(xref[5])]
            sp.acceleration = [0.0, 0.0, 0.0]
            sp.yaw = float(att_ref[2])
            self.trajectory_setpoint_pub.publish(sp)


def main(args=None):
    if not ROS2_AVAILABLE:
        print("[ROS 2] ROS 2 or px4_msgs not installed in current environment.")
        print("  Use the direct MAVLink bridge (px4_mavlink_nmpc_bridge.py) instead.")
        return

    rclpy.init(args=args)
    node = VirgoNMPCROS2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('NMPC node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
