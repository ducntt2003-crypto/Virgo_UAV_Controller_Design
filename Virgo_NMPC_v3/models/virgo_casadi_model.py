#!/usr/bin/env python3
"""
==============================================================================
 Virgo OMAV SE(3) CasADi Dynamic Model
 Formulates the continuous-time and discretized ODE for the 4.56kg 6-DOF platform.
==============================================================================
"""

import os
import yaml
import casadi as cs
import numpy as np
from se3_kinematics import (
    quat_to_rot_casadi,
    quat_derivative_casadi,
    pod_unit_thrust_casadi
)


class VirgoCasadiModel:
    """
    Symbolic 6-DOF dynamic model for Virgo OMAV on Lie Group SE(3).
    Includes physical lever arms, actuator tilt kinematics, and coaxial rotor torque.
    """

    def __init__(self, config_path=None):
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), "../config/virgo_params.yaml")

        with open(config_path, "r") as f:
            self.cfg = yaml.safe_load(f)

        # Vehicle Constants
        self.mass = float(self.cfg["vehicle"]["mass"])
        self.g = np.array(self.cfg["vehicle"]["gravity"], dtype=float)

        self.J_xx = float(self.cfg["inertia"]["J_xx"])
        self.J_yy = float(self.cfg["inertia"]["J_yy"])
        self.J_zz = float(self.cfg["inertia"]["J_zz"])
        self.J_B = cs.diag(cs.vertcat(self.J_xx, self.J_yy, self.J_zz))
        self.J_B_inv = cs.diag(cs.vertcat(1.0 / self.J_xx, 1.0 / self.J_yy, 1.0 / self.J_zz))

        # Lever arms relative to CoM in Body Frame B:
        pods = self.cfg["geometry"]["pods"]
        self.l1 = cs.vertcat(*pods["pod_1"])  # Front-Right
        self.l2 = cs.vertcat(*pods["pod_2"])  # Front-Left
        self.l3 = cs.vertcat(*pods["pod_3"])  # Rear-Left
        self.l4 = cs.vertcat(*pods["pod_4"])  # Rear-Right
        self.lever_arms = [self.l1, self.l2, self.l3, self.l4]

        # Actuator parameters
        self.c_T = float(self.cfg["propulsion"]["c_T"])
        self.c_Q = float(self.cfg["propulsion"]["c_Q"])
        self.thrust_min = float(self.cfg["propulsion"]["thrust_min_per_pod"])
        self.thrust_max = float(self.cfg["propulsion"]["thrust_max_per_pod"])
        self.angle_min = float(self.cfg["servos"]["angle_min"])
        self.angle_max = float(self.cfg["servos"]["angle_max"])
        self.rate_limit = float(self.cfg["servos"]["rate_limit"])
        self.rate_limit_alpha = float(self.cfg["servos"].get("rate_limit_alpha", 10.0))
        self.rate_limit_beta = float(self.cfg["servos"].get("rate_limit_beta", 6.4))

        # State dimensions
        # x = [p(3), v(3), q(4), omega(3), alpha(4), beta(4)] -> n_x = 21
        self.n_x = 21
        # u = [T(4), dot_alpha(4), dot_beta(4), tau_yaw(1)]   -> n_u = 13
        self.n_u = 13

        # Build symbolic model
        self._build_symbolic_dynamics()

    def _build_symbolic_dynamics(self):
        # ---------------------------------------------------------------------
        # 1. State Variables: x in R^21
        # ---------------------------------------------------------------------
        p = cs.SX.sym("p", 3)          # World Position [x, y, z] (NED)
        v = cs.SX.sym("v", 3)          # World Velocity [vx, vy, vz] (NED)
        q = cs.SX.sym("q", 4)          # Unit Quaternion [qw, qx, qy, qz]
        omega = cs.SX.sym("omega", 3)  # Body Angular Rates [p, q, r]
        alpha = cs.SX.sym("alpha", 4)  # Pitch angles of 4 pods [alpha_1..4]
        beta = cs.SX.sym("beta", 4)    # Roll angles of 4 pods [beta_1..4]

        x = cs.vertcat(p, v, q, omega, alpha, beta)

        # ---------------------------------------------------------------------
        # 2. Control Inputs: u in R^13
        # ---------------------------------------------------------------------
        T = cs.SX.sym("T", 4)                  # Pod Thrusts [T_1, T_2, T_3, T_4] (N)
        dot_alpha = cs.SX.sym("dot_alpha", 4)  # Pitch Servo Slew Rates (rad/s)
        dot_beta = cs.SX.sym("dot_beta", 4)    # Roll Servo Slew Rates (rad/s)
        tau_yaw = cs.SX.sym("tau_yaw", 1)      # Coaxial Rotor Reaction Torque (N*m)

        u = cs.vertcat(T, dot_alpha, dot_beta, tau_yaw)

        # ---------------------------------------------------------------------
        # 3. Pod Thrust Vectors & Moments in Body Frame B
        # ---------------------------------------------------------------------
        f_B_total = cs.vertcat(0.0, 0.0, 0.0)
        tau_B_total = cs.vertcat(0.0, 0.0, 0.0)

        for i in range(4):
            t_hat_i = pod_unit_thrust_casadi(alpha[i], beta[i])
            f_pod_i = T[i] * t_hat_i
            tau_pod_i = cs.cross(self.lever_arms[i], f_pod_i)

            f_B_total += f_pod_i
            tau_B_total += tau_pod_i

        # Add pure coaxial differential reaction yaw torque
        tau_B_total += cs.vertcat(0.0, 0.0, tau_yaw)

        # ---------------------------------------------------------------------
        # 4. Continuous-Time Equations of Motion
        # ---------------------------------------------------------------------
        # Translation in World Frame: m * dot{v} = m * g + R_WB * f_B
        R_WB = quat_to_rot_casadi(q)
        f_W = cs.mtimes(R_WB, f_B_total)
        g_W = cs.vertcat(self.g[0], self.g[1], self.g[2])

        p_dot = v
        v_dot = g_W + f_W / self.mass

        # Rotation: \dot{q} = 0.5 * q \otimes [0, \omega]
        q_dot = quat_derivative_casadi(q, omega)

        # Angular Acceleration: J_B * dot{omega} = -omega x (J_B * omega) + tau_B
        gyroscopic = cs.cross(omega, cs.mtimes(self.J_B, omega))
        omega_dot = cs.mtimes(self.J_B_inv, tau_B_total - gyroscopic)

        # Servo Actuator Kinematics: \dot{alpha} = u_{dot_alpha}, \dot{beta} = u_{dot_beta}
        alpha_dot = dot_alpha
        beta_dot = dot_beta

        # Full state derivative
        x_dot = cs.vertcat(p_dot, v_dot, q_dot, omega_dot, alpha_dot, beta_dot)

        # Store CasADi functions
        self.f_expl = x_dot
        self.x = x
        self.u = u
        self.f_B_total = f_B_total
        self.tau_B_total = tau_B_total

        self.dynamics_fun = cs.Function("dynamics", [x, u], [x_dot], ["x", "u"], ["x_dot"])
        self.wrench_fun = cs.Function("wrench", [x, u], [f_B_total, tau_B_total], ["x", "u"], ["f_B", "tau_B"])

    def create_rk4_integrator(self, dt):
        """Creates an Explicit Runge-Kutta 4th order integrator function: x_{k+1} = f_rk4(x_k, u_k)."""
        x = self.x
        u = self.u
        k1 = self.dynamics_fun(x, u)
        k2 = self.dynamics_fun(x + 0.5 * dt * k1, u)
        k3 = self.dynamics_fun(x + 0.5 * dt * k2, u)
        k4 = self.dynamics_fun(x + dt * k3, u)

        x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        # Re-normalize quaternion
        q_raw = x_next[6:10]
        q_norm = cs.norm_2(q_raw)
        q_normalized = q_raw / cs.if_else(q_norm > 1e-6, q_norm, 1.0)

        x_next_normalized = cs.vertcat(
            x_next[0:6],
            q_normalized,
            x_next[10:]
        )

        return cs.Function("rk4_step", [x, u], [x_next_normalized], ["x", "u"], ["x_next"])


if __name__ == "__main__":
    print("[VirgoCasadiModel] Compiling and verifying symbolic equations...")
    model = VirgoCasadiModel()
    print(f"  State dim n_x: {model.n_x}, Control dim n_u: {model.n_u}")

    # Test numerical evaluation
    x0 = np.zeros(21)
    x0[6] = 1.0  # qw = 1.0 (identity quaternion)
    u0 = np.zeros(13)
    u0[0:4] = (model.mass * 9.80665) / 4.0  # Hover thrust per pod

    x_dot_val = model.dynamics_fun(x0, u0).full().flatten()
    print(f"  Hover acceleration check: a_z = {x_dot_val[5]:.4f} m/s^2 (expect 0.0000 m/s^2)")
    assert abs(x_dot_val[5]) < 1e-4, "Hover dynamic equilibrium verification failed!"

    rk4 = model.create_rk4_integrator(0.04)
    x1 = rk4(x0, u0).full().flatten()
    print(f"  RK4 step test successful: norm(q) = {np.linalg.norm(x1[6:10]):.6f}")
    print("[VirgoCasadiModel] Validation passed successfully!")
