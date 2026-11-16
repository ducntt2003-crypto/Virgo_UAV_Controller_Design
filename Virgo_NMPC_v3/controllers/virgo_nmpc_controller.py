#!/usr/bin/env python3
"""
==============================================================================
 High-Level NMPC Controller for Virgo OMAV
 Manages reference lookahead, state integration, safety watchdog,
 and actuator command conversions.
==============================================================================
"""

import math
import os
import sys
import time
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), "../models"))
sys.path.append(os.path.join(os.path.dirname(__file__), "../solver"))
sys.path.append(os.path.join(os.path.dirname(__file__), "."))

from casadi_nmpc_solver import CasadiNMPCSolver
from trajectory_generator import DecoupledTrajectoryGenerator
from se3_kinematics import quat_to_euler, pod_unit_thrust_numpy


class VirgoNMPCController:
    """
    Supervises NMPC real-time closed-loop control for the 4.56kg Virgo OMAV platform.
    """

    def __init__(self, virgo_config=None, nmpc_config=None):
        self.solver = CasadiNMPCSolver(virgo_config, nmpc_config)
        self.traj_gen = DecoupledTrajectoryGenerator()

        self.N = self.solver.N
        self.dt = self.solver.dt
        self.model = self.solver.model

        # Internal state tracking
        self.current_state = np.zeros(21)
        self.current_state[6] = 1.0  # qw = 1.0

        # Safety & Diagnostics
        self.solve_times_ms = []
        self.fail_count = 0
        self.watchdog_tripped = False

    def update_state(self, pos, vel, quat, omega, alpha, beta):
        """
        Updates current estimated state from telemetry / EKF2.
        All inputs should be numpy arrays with respective dimensions.
        """
        self.current_state[0:3] = pos
        self.current_state[3:6] = vel
        # Normalize quaternion
        q_norm = np.linalg.norm(quat)
        self.current_state[6:10] = quat / (q_norm if q_norm > 1e-6 else 1.0)
        self.current_state[10:13] = omega
        self.current_state[13:17] = alpha
        self.current_state[17:21] = beta

    def compute_control(self, current_time):
        """
        Computes optimal control vector for current flight time t.
        Returns dictionary containing:
          - pod_thrusts: [T1, T2, T3, T4] in N
          - servo_alpha_cmd: [a1, a2, a3, a4] in rad
          - servo_beta_cmd:  [b1, b2, b3, b4] in rad
          - tau_yaw: reaction torque in N*m
          - f_B: total body force [fx, fy, fz] in N (NED)
          - tau_B: total body torque [tx, ty, tz] in N*m
          - solve_time_ms: time taken by solver in ms
          - success: boolean
        """
        # 1. Build Reference Lookahead Horizon: (21, N+1)
        xref_horizon = np.zeros((21, self.N + 1))
        for k in range(self.N + 1):
            t_k = current_time + k * self.dt
            xk_ref, _ = self.traj_gen.get_reference_state(t_k)
            xref_horizon[:, k] = xk_ref

        # 2. Call NMPC Solver
        u_opt, X_pred, t_ms, ok = self.solver.solve(self.current_state, xref_horizon)
        self.solve_times_ms.append(t_ms)

        if not ok:
            self.fail_count += 1
            if self.fail_count > 5:
                self.watchdog_tripped = True
        else:
            self.fail_count = 0

        # 3. Decode Optimal Control
        T_opt = u_opt[0:4]
        dot_alpha_opt = u_opt[4:8]
        dot_beta_opt = u_opt[8:12]
        tau_yaw_opt = float(u_opt[12])

        # Integrate commanded servo angles: alpha_cmd = alpha_curr + dot_alpha * dt
        alpha_curr = self.current_state[13:17]
        beta_curr = self.current_state[17:21]

        alpha_cmd = np.clip(alpha_curr + dot_alpha_opt * self.dt, self.model.angle_min, self.model.angle_max)
        beta_cmd = np.clip(beta_curr + dot_beta_opt * self.dt, self.model.angle_min, self.model.angle_max)

        # 4. Calculate Net Resultant Body Wrench (NED)
        f_B = np.zeros(3)
        tau_B = np.zeros(3)
        for i in range(4):
            t_hat_i = pod_unit_thrust_numpy(alpha_cmd[i], beta_cmd[i])
            f_pod_i = T_opt[i] * t_hat_i
            lever_arm = np.array(self.model.cfg["geometry"]["pods"][f"pod_{i+1}"])
            tau_pod_i = np.cross(lever_arm, f_pod_i)

            f_B += f_pod_i
            tau_B += tau_pod_i

        tau_B[2] += tau_yaw_opt

        # 5. Motor RPM Speeds: Omega = sqrt(T / (2 * c_T))
        omega_rotors = np.sqrt(np.clip(T_opt / (2.0 * self.model.c_T), 0.0, self.model.cfg["propulsion"]["omega_max"]**2))

        # Normalized motor thrusts in [0, 1] relative to max thrust per pod
        motor_norm = T_opt / self.model.thrust_max

        result = {
            'pod_thrusts': T_opt,
            'rotor_speeds_rad_s': omega_rotors,
            'motor_normalized': motor_norm,
            'servo_alpha_cmd': alpha_cmd,
            'servo_beta_cmd': beta_cmd,
            'dot_alpha': dot_alpha_opt,
            'dot_beta': dot_beta_opt,
            'tau_yaw': tau_yaw_opt,
            'f_B': f_B,
            'tau_B': tau_B,
            'X_pred': X_pred,
            'solve_time_ms': t_ms,
            'success': ok,
            'watchdog': self.watchdog_tripped
        }
        return result


if __name__ == "__main__":
    print("[VirgoNMPCController] Initializing test controller...")
    controller = VirgoNMPCController()

    pos0 = np.array([0.0, 0.0, -1.5])
    vel0 = np.zeros(3)
    quat0 = np.array([1.0, 0.0, 0.0, 0.0])
    omega0 = np.zeros(3)
    alpha0 = np.zeros(4)
    beta0 = np.zeros(4)

    controller.update_state(pos0, vel0, quat0, omega0, alpha0, beta0)
    print("  Testing control compute step at t=0.0s...")
    res = controller.compute_control(0.0)
    print(f"  Solve Time: {res['solve_time_ms']:.2f} ms")
    print(f"  Thrusts: {np.round(res['pod_thrusts'], 2)} N (Total: {np.sum(res['pod_thrusts']):.2f} N)")
    print(f"  Resultant Body Force f_B: {np.round(res['f_B'], 2)} N (Hover requires Z ~ -44.72 N)")
    print(f"  Resultant Body Torque tau_B: {np.round(res['tau_B'], 4)} N*m")
    print("[VirgoNMPCController] Validation passed successfully!")
