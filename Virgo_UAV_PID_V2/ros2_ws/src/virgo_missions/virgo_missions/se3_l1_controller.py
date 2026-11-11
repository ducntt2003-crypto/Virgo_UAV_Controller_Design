"""
==============================================================================
 SE(3) Geometric Controller + L1 Adaptive Disturbance Observer + Null-Space Allocation
 Based on MorphQuad (arXiv:2607.02764v4) and L1Quad (IEEE T-CST 2025)
 Model: AUV_USDF_V2 (m=1.46kg, Lx=0.19899m, Ly=0.19742m, r=0.28030m)
==============================================================================
"""

import math
import numpy as np


def skew(v: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix [v]x from 3D vector v."""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0]
    ])


def vee(mat: np.ndarray) -> np.ndarray:
    """Vee map: extract 3D vector from skew-symmetric matrix."""
    return np.array([mat[2, 1], mat[0, 2], mat[1, 0]])


class L1AdaptiveDisturbanceObserver:
    """
    L1 Adaptive Disturbance Observer for Full 6-DOF OMAV Dynamics.
    Estimates unmodeled external forces (wind gusts, ground effect, tool contact)
    and external torques, filtering high-frequency noise while guaranteeing
    bounded tracking error.
    """
    def __init__(self, mass: float = 1.46, J: np.ndarray = None):
        self.mass = mass
        self.J = J if J is not None else np.diag([0.025, 0.025, 0.045])
        self.J_inv = np.linalg.inv(self.J)

        # Translational predictor states
        self.v_hat = np.zeros(3)
        self.d_v_hat = np.zeros(3)
        self.d_v_filt = np.zeros(3)

        # Rotational predictor states
        self.w_hat = np.zeros(3)
        self.d_w_hat = np.zeros(3)
        self.d_w_filt = np.zeros(3)

        # Tuning parameters (L1Quad / MorphQuad Section IV)
        self.As_v = 15.0       # Predictor Hurwitz pole for velocity
        self.Gamma_v = 120.0   # Adaptation gain for translational disturbances
        self.w_co_v = 18.0     # Low-pass filter cutoff (rad/s)
        self.d_v_max = 20.0    # Projection bound (m/s^2)

        self.As_w = 20.0       # Predictor Hurwitz pole for angular rate
        self.Gamma_w = 150.0   # Adaptation gain for rotational disturbances
        self.w_co_w = 25.0     # Low-pass filter cutoff (rad/s)
        self.d_w_max = 50.0    # Projection bound (rad/s^2)

        self.g = 9.81
        self.initialized = False

    def reset(self, v_init: np.ndarray, w_init: np.ndarray):
        self.v_hat = v_init.copy()
        self.w_hat = w_init.copy()
        self.d_v_hat = np.zeros(3)
        self.d_v_filt = np.zeros(3)
        self.d_w_hat = np.zeros(3)
        self.d_w_filt = np.zeros(3)
        self.initialized = True

    def update(self, dt: float, v_meas: np.ndarray, w_meas: np.ndarray,
               R_meas: np.ndarray, f_body: np.ndarray, tau_body: np.ndarray):
        """
        Update observer states given measured velocity, angular velocity,
        orientation matrix R, and applied body forces and torques.
        """
        if not self.initialized:
            self.reset(v_meas, w_meas)
            return self.d_v_filt, self.d_w_filt

        dt = max(min(dt, 0.05), 0.001)

        # ---------------------------------------------------------------------
        # 1. Translational Observer (Velocity Predictor + Adaptation Law)
        # ---------------------------------------------------------------------
        # v_tilde = v_hat - v_meas
        v_tilde = self.v_hat - v_meas

        # v_dot_hat = -g*e3 + (1/m)*R*f_body + d_v_hat - As_v * v_tilde
        e3 = np.array([0.0, 0.0, 1.0])
        acc_applied = (1.0 / self.mass) * (R_meas @ f_body)
        v_dot_hat = -self.g * e3 + acc_applied + self.d_v_hat - self.As_v * v_tilde

        # Euler step for predictor
        self.v_hat += dt * v_dot_hat

        # Adaptation with smooth projection
        d_v_hat_dot = -self.Gamma_v * v_tilde
        self.d_v_hat += dt * d_v_hat_dot
        norm_dv = np.linalg.norm(self.d_v_hat)
        if norm_dv > self.d_v_max:
            self.d_v_hat = (self.d_v_max / norm_dv) * self.d_v_hat

        # Low-pass filter: C(s) = w_co / (s + w_co)
        self.d_v_filt += dt * self.w_co_v * (self.d_v_hat - self.d_v_filt)

        # ---------------------------------------------------------------------
        # 2. Rotational Observer (Angular Rate Predictor + Adaptation Law)
        # ---------------------------------------------------------------------
        w_tilde = self.w_hat - w_meas

        # w_dot_hat = J_inv * (tau_body - w x J*w) + d_w_hat - As_w * w_tilde
        gyroscopic = np.cross(w_meas, self.J @ w_meas)
        w_dot_hat = self.J_inv @ (tau_body - gyroscopic) + self.d_w_hat - self.As_w * w_tilde
        self.w_hat += dt * w_dot_hat

        # Adaptation
        d_w_hat_dot = -self.Gamma_w * w_tilde
        self.d_w_hat += dt * d_w_hat_dot
        norm_dw = np.linalg.norm(self.d_w_hat)
        if norm_dw > self.d_w_max:
            self.d_w_hat = (self.d_w_max / norm_dw) * self.d_w_hat

        self.d_w_filt += dt * self.w_co_w * (self.d_w_hat - self.d_w_filt)

        return self.d_v_filt, self.d_w_filt


class SE3GeometricController:
    """
    Lie Group SE(3) Singularity-Free Geometric Tracking Controller.
    Computes body forces and torques directly on R^3 x SO(3),
    incorporating feedforward acceleration and L1 adaptive disturbance rejection.
    """
    def __init__(self, mass: float = 1.46, J: np.ndarray = None):
        self.mass = mass
        self.J = J if J is not None else np.diag([0.025, 0.025, 0.045])
        self.g = 9.81

        # Position gains
        self.kp = np.array([4.5, 4.5, 6.0])
        self.kv = np.array([3.2, 3.2, 4.0])

        # Attitude gains
        self.kR = np.array([8.5, 8.5, 4.0])
        self.kw = np.array([0.75, 0.75, 0.45])

    def compute_control(self, p: np.ndarray, v: np.ndarray, R: np.ndarray, w: np.ndarray,
                        pd: np.ndarray, vd: np.ndarray, acc_d: np.ndarray,
                        Rd: np.ndarray, wd: np.ndarray, w_dot_d: np.ndarray,
                        d_v_filt: np.ndarray = None, d_w_filt: np.ndarray = None) -> tuple:
        """
        Compute desired Body Force f_body and Body Torque tau_body.
        MorphQuad Eq. 11 augmented with L1 disturbance cancellation.
        """
        # Tracking errors on SE(3)
        ep = p - pd
        ev = v - vd

        # Rotation error on SO(3): eR = 1/2 * (Rd^T * R - R^T * Rd)^vee
        eR_mat = 0.5 * (Rd.T @ R - R.T @ Rd)
        eR = vee(eR_mat)

        # Angular rate error: ew = w - R^T * Rd * wd
        R_rel = R.T @ Rd
        ew = w - R_rel @ wd

        # Disturbance compensation
        d_v = d_v_filt if d_v_filt is not None else np.zeros(3)
        d_w = d_w_filt if d_w_filt is not None else np.zeros(3)

        # ---------------------------------------------------------------------
        # 1. Desired Body Force (Eq. 11)
        # f_Geo = R^T * (-kp*ep - kv*ev - m*g*e3 + m*acc_d - m*d_v)
        # In Body FLU frame: f_body = [fx, fy, fz]^T
        # ---------------------------------------------------------------------
        e3 = np.array([0.0, 0.0, 1.0])
        f_inertial = -self.kp * ep - self.kv * ev + self.mass * (self.g * e3 + acc_d) - self.mass * d_v
        f_body = R.T @ f_inertial

        # ---------------------------------------------------------------------
        # 2. Desired Body Torque (Eq. 11)
        # tau_Geo = -kR*eR - kw*ew + w x (J*w) - J*(w_hat * R^T*Rd*wd - R^T*Rd*w_dot_d) - J*d_w
        # ---------------------------------------------------------------------
        gyroscopic = np.cross(w, self.J @ w)
        w_cross_term = np.cross(w, R_rel @ wd) - R_rel @ w_dot_d
        tau_body = -self.kR * eR - self.kw * ew + gyroscopic - self.J @ w_cross_term - self.J @ d_w

        return f_body, tau_body, ep, ev, eR, ew


class MorphQuadNullSpaceAllocator:
    """
    Closed-Form Analytical Minimum-Norm Allocator with Null-Space Singularity
    Avoidance (Gimbal Lock) and Downwash Mitigation (MorphQuad arXiv:2607.02764v4).
    Adapted to CAD model AUV_USDF_V2.
    """
    def __init__(self, lx: float = 0.19899, ly: float = 0.19742):
        self.lx = lx
        self.ly = ly
        self.r = math.sqrt(lx * lx + ly * ly)

        # Pod positions in body FLU frame:
        # Pod 0: Front-Left  (+lx, +ly)
        # Pod 1: Rear-Left   (-lx, +ly)
        # Pod 2: Rear-Right  (-lx, -ly)
        # Pod 3: Front-Right (+lx, -ly)
        self.p = [
            np.array([ lx,  ly, 0.0]),
            np.array([-lx,  ly, 0.0]),
            np.array([-lx, -ly, 0.0]),
            np.array([ lx, -ly, 0.0])
        ]

        # Wrench map matrix M (6 x 12)
        I3 = np.eye(3)
        self.M = np.block([
            [I3, I3, I3, I3],
            [skew(self.p[0]), skew(self.p[1]), skew(self.p[2]), skew(self.p[3])]
        ])

        # SVD Null-Space basis NM (12 x 6)
        _, _, Vt = np.linalg.svd(self.M)
        self.NM = Vt[6:].T

        # Unwrapping memory for continuous multi-turn rotations
        self.alpha_unwrapped = np.zeros(4)
        self.beta_unwrapped = np.zeros(4)
        self.alpha_prev_raw = np.zeros(4)
        self.beta_prev_raw = np.zeros(4)

    def allocate(self, f_body: np.ndarray, tau_body: np.ndarray, lock_servos: bool = False) -> dict:
        """
        Allocate 6-DOF wrench to 4 pod thrust vectors, gimbal angles, and motor speeds.
        """
        f = f_body.copy()
        tau = tau_body.copy()
        f_norm = np.linalg.norm(f)

        # 1. Closed-form minimum-norm solution t0
        f_quarter = f * 0.25
        kB = np.array([0.0, 0.0, 1.0])

        e_psi0 = np.array([-self.ly / self.r,  self.lx / self.r, 0.0])
        e_psi1 = np.array([ self.ly / self.r,  self.lx / self.r, 0.0])
        e_psi2 = np.array([ self.ly / self.r, -self.lx / self.r, 0.0])
        e_psi3 = np.array([-self.ly / self.r, -self.lx / self.r, 0.0])

        t0 = np.zeros((4, 3))
        t0[0] = f_quarter - kB * (tau[0] / (4.0 * self.ly)) - kB * (tau[1] / (4.0 * self.lx)) + e_psi0 * (tau[2] / (4.0 * self.r))
        t0[1] = f_quarter - kB * (tau[0] / (4.0 * self.ly)) + kB * (tau[1] / (4.0 * self.lx)) + e_psi1 * (tau[2] / (4.0 * self.r))
        t0[2] = f_quarter + kB * (tau[0] / (4.0 * self.ly)) + kB * (tau[1] / (4.0 * self.lx)) + e_psi2 * (tau[2] / (4.0 * self.r))
        t0[3] = f_quarter + kB * (tau[0] / (4.0 * self.ly)) - kB * (tau[1] / (4.0 * self.lx)) + e_psi3 * (tau[2] / (4.0 * self.r))

        # 2. Null-space injection
        cN = np.zeros(6)
        if f_norm > 0.05 and not lock_servos:
            f_hat = f / f_norm

            # Condition (ii): Gimbal-Lock Repulsion (Inner servo beta repulsion from +/- 90 deg)
            k_sing = 0.5 * f_norm
            cN[0] = k_sing * f_hat[1]
            cN[2] = k_sing * f_hat[1]
            cN[4] = k_sing * f_hat[1]

            # Condition (i): Downwash Avoidance
            k_wash = 0.4 * f_norm
            if f_hat[0] > 0.0:
                cN[1] = -k_wash * f_hat[0]
                cN[3] =  k_wash * f_hat[0]
            else:
                cN[3] = -k_wash * f_hat[0]
                cN[5] = -k_wash * f_hat[0]

        delta_null = (self.NM @ cN).reshape(4, 3)
        t_effective = t0 + delta_null

        # 3. Resolve per-pod gimbal angles and thrust magnitudes
        alpha = np.zeros(4)
        beta = np.zeros(4)
        thrust_mag = np.zeros(4)

        for i in range(4):
            ti = t_effective[i]
            t_mag = np.linalg.norm(ti)
            thrust_mag[i] = t_mag

            if lock_servos or t_mag < 0.02:
                # Lock upright
                alpha[i] = round(self.alpha_unwrapped[i] / (2.0 * math.pi)) * (2.0 * math.pi)
                beta[i]  = round(self.beta_unwrapped[i]  / (2.0 * math.pi)) * (2.0 * math.pi)
                self.alpha_unwrapped[i] = alpha[i]
                self.beta_unwrapped[i]  = beta[i]
                self.alpha_prev_raw[i]  = 0.0
                self.beta_prev_raw[i]   = 0.0
            else:
                that = ti / t_mag
                raw_beta  = math.asin(max(min(-that[1], 0.995), -0.995))
                raw_alpha = math.atan2(that[0], that[2])

                # Multi-turn continuous phase unwrapping
                diff_a = raw_alpha - self.alpha_prev_raw[i]
                while diff_a > math.pi:  diff_a -= 2.0 * math.pi
                while diff_a < -math.pi: diff_a += 2.0 * math.pi
                self.alpha_unwrapped[i] += diff_a
                self.alpha_prev_raw[i] = raw_alpha

                diff_b = raw_beta - self.beta_prev_raw[i]
                while diff_b > math.pi:  diff_b -= 2.0 * math.pi
                while diff_b < -math.pi: diff_b += 2.0 * math.pi
                self.beta_unwrapped[i] += diff_b
                self.beta_prev_raw[i] = raw_beta

                alpha[i] = self.alpha_unwrapped[i]
                beta[i]  = self.beta_unwrapped[i]

        return {
            'thrust_vectors': t_effective,
            'thrust_magnitudes': thrust_mag,
            'alpha_pitch': alpha,
            'beta_roll': beta,
            'cN': cN,
            'wrench_delivered': self.M @ t_effective.flatten()
        }
