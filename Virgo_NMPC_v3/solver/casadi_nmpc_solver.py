#!/usr/bin/env python3
"""
==============================================================================
 Real-Time Nonlinear Model Predictive Control (NMPC) Solver
 Implementation: Direct Multiple Shooting on SE(3) using CasADi & IPOPT
==============================================================================
"""

import os
import sys
import time
import yaml
import numpy as np
import casadi as cs

sys.path.append(os.path.join(os.path.dirname(__file__), "../models"))
from virgo_casadi_model import VirgoCasadiModel
from se3_kinematics import quat_error_so3_casadi


class CasadiNMPCSolver:
    """
    Multiple Shooting NMPC Solver for the 21-state 13-input Virgo OMAV platform.
    Features quaternion-based SE(3) error metrics, servo rate limits, and warm-starting.
    """

    def __init__(self, virgo_config=None, nmpc_config=None):
        if virgo_config is None:
            virgo_config = os.path.join(os.path.dirname(__file__), "../config/virgo_params.yaml")
        if nmpc_config is None:
            nmpc_config = os.path.join(os.path.dirname(__file__), "../config/nmpc_config.yaml")

        with open(nmpc_config, "r") as f:
            self.nmpc_cfg = yaml.safe_load(f)

        self.model = VirgoCasadiModel(virgo_config)
        self.N = int(self.nmpc_cfg["horizon"]["N"])
        self.dt = float(self.nmpc_cfg["horizon"]["dt"])

        # Load weights
        w = self.nmpc_cfg["weights"]
        self.Q_pos = np.array(w["Q_pos"], dtype=float)
        self.Q_vel = np.array(w["Q_vel"], dtype=float)
        self.Q_att = np.array(w["Q_att"], dtype=float)
        self.Q_omega = np.array(w["Q_omega"], dtype=float)
        self.Q_alpha = np.array(w["Q_servo_alpha"], dtype=float)
        self.Q_beta = np.array(w["Q_servo_beta"], dtype=float)

        self.R_thrust = np.array(w["R_thrust"], dtype=float)
        self.R_dot_alpha = np.array(w["R_dot_alpha"], dtype=float)
        self.R_dot_beta = np.array(w["R_dot_beta"], dtype=float)
        self.R_tau_yaw = 0.1
        self.terminal_mult = float(w["terminal_multiplier"])

        self.T_hover = (self.model.mass * self.model.g[2]) / 4.0

        # State and control dimensions
        self.n_x = self.model.n_x  # 21
        self.n_u = self.model.n_u  # 13

        # Warm start buffers
        self.X_guess = np.zeros((self.n_x, self.N + 1))
        self.U_guess = np.zeros((self.n_u, self.N))
        for k in range(self.N):
            self.U_guess[0:4, k] = self.T_hover

        # Build NLP
        self._build_nlp()

    def _build_nlp(self):
        """Constructs CasADi Opti optimization problem."""
        self.opti = cs.Opti()

        # Decision Variables
        # X: States over horizon [x_0, x_1, ..., x_N]
        # U: Controls over horizon [u_0, u_1, ..., u_{N-1}]
        self.X = self.opti.variable(self.n_x, self.N + 1)
        self.U = self.opti.variable(self.n_u, self.N)

        # Parameters
        self.P_x0 = self.opti.parameter(self.n_x, 1)          # Initial measured state
        self.P_xref = self.opti.parameter(self.n_x, self.N + 1)  # Reference trajectory

        # Integrator
        f_rk4 = self.model.create_rk4_integrator(self.dt)

        cost = 0.0

        # Initial condition constraint
        self.opti.subject_to(self.X[:, 0] == self.P_x0)

        # Build Stage Costs and Multiple Shooting Constraints
        for k in range(self.N):
            xk = self.X[:, k]
            uk = self.U[:, k]
            xk_ref = self.P_xref[:, k]

            # 1. State Tracking Errors
            p_err = xk[0:3] - xk_ref[0:3]
            v_err = xk[3:6] - xk_ref[3:6]
            att_err = quat_error_so3_casadi(xk[6:10], xk_ref[6:10])
            omega_err = xk[10:13] - xk_ref[10:13]
            alpha_err = xk[13:17] - xk_ref[13:17]
            beta_err = xk[17:21] - xk_ref[17:21]

            cost += cs.mtimes([p_err.T, cs.diag(self.Q_pos), p_err])
            cost += cs.mtimes([v_err.T, cs.diag(self.Q_vel), v_err])
            cost += cs.mtimes([att_err.T, cs.diag(self.Q_att), att_err])
            cost += cs.mtimes([omega_err.T, cs.diag(self.Q_omega), omega_err])
            cost += cs.mtimes([alpha_err.T, cs.diag(self.Q_alpha), alpha_err])
            cost += cs.mtimes([beta_err.T, cs.diag(self.Q_beta), beta_err])

            # 2. Control Effort Penalties
            T_dev = uk[0:4] - self.T_hover
            dot_a = uk[4:8]
            dot_b = uk[8:12]
            tau_y = uk[12]

            cost += cs.mtimes([T_dev.T, cs.diag(self.R_thrust), T_dev])
            cost += cs.mtimes([dot_a.T, cs.diag(self.R_dot_alpha), dot_a])
            cost += cs.mtimes([dot_b.T, cs.diag(self.R_dot_beta), dot_b])
            cost += self.R_tau_yaw * (tau_y ** 2)

            # 3. Dynamic continuity constraint (Multiple Shooting)
            x_next = f_rk4(xk, uk)
            self.opti.subject_to(self.X[:, k + 1] == x_next)

            # 4. Control Bounds
            # Pod thrust: 0.5 <= T <= 18.5 N
            self.opti.subject_to(self.opti.bounded(self.model.thrust_min, uk[0:4], self.model.thrust_max))
            # Slew rate: -10 <= dot_alpha, dot_beta <= +10 rad/s
            self.opti.subject_to(self.opti.bounded(-self.model.rate_limit, uk[4:8], self.model.rate_limit))
            self.opti.subject_to(self.opti.bounded(-self.model.rate_limit, uk[8:12], self.model.rate_limit))
            # Coaxial yaw torque: -0.5 <= tau_yaw <= 0.5 N*m
            self.opti.subject_to(self.opti.bounded(-0.5, uk[12], 0.5))

            # 5. State Bounds: Gimbal limits: -pi/2 <= alpha, beta <= pi/2
            self.opti.subject_to(self.opti.bounded(self.model.angle_min, xk[13:17], self.model.angle_max))
            self.opti.subject_to(self.opti.bounded(self.model.angle_min, xk[17:21], self.model.angle_max))

        # Terminal Cost
        xN = self.X[:, self.N]
        xN_ref = self.P_xref[:, self.N]
        p_err_N = xN[0:3] - xN_ref[0:3]
        v_err_N = xN[3:6] - xN_ref[3:6]
        att_err_N = quat_error_so3_casadi(xN[6:10], xN_ref[6:10])
        omega_err_N = xN[10:13] - xN_ref[10:13]

        cost += self.terminal_mult * (
            cs.mtimes([p_err_N.T, cs.diag(self.Q_pos), p_err_N]) +
            cs.mtimes([v_err_N.T, cs.diag(self.Q_vel), v_err_N]) +
            cs.mtimes([att_err_N.T, cs.diag(self.Q_att), att_err_N]) +
            cs.mtimes([omega_err_N.T, cs.diag(self.Q_omega), omega_err_N])
        )

        self.opti.minimize(cost)

        # Fast IPOPT solver options
        p_opts = {
            "expand": True,
            "print_time": False
        }
        s_opts = {
            "max_iter": 35,
            "tol": 1.0e-3,
            "acceptable_tol": 5.0e-3,
            "acceptable_iter": 5,
            "print_level": 0,
            "linear_solver": "mumps",
            "warm_start_init_point": "yes"
        }
        self.opti.solver("ipopt", p_opts, s_opts)

    def solve(self, x0_val, xref_horizon):
        """
        Solves one NMPC iteration.
        Inputs:
          x0_val: current state (21,)
          xref_horizon: reference trajectory across horizon (21, N+1)
        Returns:
          u_opt: optimal first control (13,)
          X_pred: predicted state trajectory (21, N+1)
          solve_time: calculation duration in ms
          success: boolean flag
        """
        t_start = time.perf_counter()

        self.opti.set_value(self.P_x0, x0_val)
        self.opti.set_value(self.P_xref, xref_horizon)

        # Set warm start initial values
        self.opti.set_initial(self.X, self.X_guess)
        self.opti.set_initial(self.U, self.U_guess)

        success = True
        try:
            sol = self.opti.solve()
            X_sol = sol.value(self.X)
            U_sol = sol.value(self.U)
        except Exception:
            # Retrieve partial/debug values if max_iter hit or mild failure
            success = False
            X_sol = self.opti.debug.value(self.X)
            U_sol = self.opti.debug.value(self.U)

        solve_time_ms = (time.perf_counter() - t_start) * 1000.0

        # Update warm-start buffers by shifting one step forward
        self.X_guess[:, :-1] = X_sol[:, 1:]
        self.X_guess[:, -1] = X_sol[:, -1]
        self.U_guess[:, :-1] = U_sol[:, 1:]
        self.U_guess[:, -1] = U_sol[:, -1]

        u_opt = U_sol[:, 0]
        return u_opt, X_sol, solve_time_ms, success


if __name__ == "__main__":
    print("[CasadiNMPCSolver] Initializing and compiling solver graph...")
    solver = CasadiNMPCSolver()
    print(f"  Initialized horizon N={solver.N}, dt={solver.dt}s")

    # Initial state: hover at Z = -1.5m, slightly disturbed
    x0 = np.zeros(21)
    x0[2] = -1.5   # Z = -1.5m (1.5m AGL)
    x0[6] = 1.0    # qw = 1.0

    # Constant reference at origin hover
    xref = np.zeros((21, solver.N + 1))
    for k in range(solver.N + 1):
        xref[2, k] = -1.5
        xref[6, k] = 1.0

    print("  Solving benchmark test iteration...")
    u_opt, X_pred, t_ms, ok = solver.solve(x0, xref)
    print(f"  Result: Success={ok}, Solve Time={t_ms:.2f} ms")
    print(f"  Optimal thrusts T_1..4: {np.round(u_opt[0:4], 2)} N (sum={np.sum(u_opt[0:4]):.2f} N, hover={solver.model.mass*9.80665:.2f} N)")
    print(f"  Optimal slew rates: alpha={np.round(u_opt[4:8], 3)}, beta={np.round(u_opt[8:12], 3)}")
    assert ok, "NMPC benchmark solve failed!"
    print("[CasadiNMPCSolver] Validation completed successfully!")
