#!/usr/bin/env python3
"""
==============================================================================
 6-DOF Decoupled Trajectory Generator for Virgo OMAV NMPC
 Generates 25m x 25m square benchmark with stationary corner attitude transitions
 and constant 30° decoupled attitude holding along edges.
==============================================================================
"""

import math
import numpy as np
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), "../models"))
from se3_kinematics import euler_to_quat


class DecoupledTrajectoryGenerator:
    """
    Constructs a smooth, continuous C^2 reference trajectory for the 25m square
    decoupled 6-DOF flight mission.
    """

    def __init__(self, edge_length=25.0, altitude=1.5, cruise_speed=1.5, corner_duration=4.0):
        self.L = float(edge_length)
        self.target_alt = float(altitude)  # In NED, z = -target_alt
        self.cruise_speed = float(cruise_speed)
        self.corner_dur = float(corner_duration)

        self.deg30 = math.radians(30.0)

        # Build trajectory schedule
        self._build_schedule()

    def _build_schedule(self):
        """Constructs timing table for all flight phases."""
        edge_dur = self.L / self.cruise_speed  # ~16.67 s per 25m edge

        self.phases = []
        t = 0.0

        # Phase 0: Initial hover at origin (0, 0)
        self.phases.append({
            'name': 'Origin Hover',
            't_start': t, 't_end': t + 2.0,
            'type': 'hover',
            'pos': np.array([0.0, 0.0, -self.target_alt]),
            'att': np.array([0.0, 0.0, 0.0])
        })
        t += 2.0

        # Phase 1: Attitude tilt at origin to Roll = +30°
        self.phases.append({
            'name': 'Origin Tilt to Roll +30°',
            't_start': t, 't_end': t + self.corner_dur,
            'type': 'transition',
            'pos': np.array([0.0, 0.0, -self.target_alt]),
            'att_start': np.array([0.0, 0.0, 0.0]),
            'att_end': np.array([self.deg30, 0.0, 0.0])
        })
        t += self.corner_dur

        # Phase 2: Edge 1 (0, 0) -> (+L, 0) holding Roll = +30°
        self.phases.append({
            'name': 'Edge 1 (+X, Roll=+30°)',
            't_start': t, 't_end': t + edge_dur,
            'type': 'cruise',
            'p_start': np.array([0.0, 0.0, -self.target_alt]),
            'p_end': np.array([self.L, 0.0, -self.target_alt]),
            'att': np.array([self.deg30, 0.0, 0.0])
        })
        t += edge_dur

        # Phase 3: Corner 1 (+L, 0) transition Roll +30° -> Pitch +30°
        self.phases.append({
            'name': 'Corner 1 (Roll 30° -> Pitch 30°)',
            't_start': t, 't_end': t + self.corner_dur,
            'type': 'transition',
            'pos': np.array([self.L, 0.0, -self.target_alt]),
            'att_start': np.array([self.deg30, 0.0, 0.0]),
            'att_end': np.array([0.0, self.deg30, 0.0])
        })
        t += self.corner_dur

        # Phase 4: Edge 2 (+L, 0) -> (+L, +L) holding Pitch = +30°
        self.phases.append({
            'name': 'Edge 2 (+Y, Pitch=+30°)',
            't_start': t, 't_end': t + edge_dur,
            'type': 'cruise',
            'p_start': np.array([self.L, 0.0, -self.target_alt]),
            'p_end': np.array([self.L, self.L, -self.target_alt]),
            'att': np.array([0.0, self.deg30, 0.0])
        })
        t += edge_dur

        # Phase 5: Corner 2 (+L, +L) transition Pitch +30° -> Yaw +30°
        self.phases.append({
            'name': 'Corner 2 (Pitch 30° -> Yaw 30°)',
            't_start': t, 't_end': t + self.corner_dur,
            'type': 'transition',
            'pos': np.array([self.L, self.L, -self.target_alt]),
            'att_start': np.array([0.0, self.deg30, 0.0]),
            'att_end': np.array([0.0, 0.0, self.deg30])
        })
        t += self.corner_dur

        # Phase 6: Edge 3 (+L, +L) -> (0, +L) holding Yaw = +30°
        self.phases.append({
            'name': 'Edge 3 (-X, Yaw=+30°)',
            't_start': t, 't_end': t + edge_dur,
            'type': 'cruise',
            'p_start': np.array([self.L, self.L, -self.target_alt]),
            'p_end': np.array([0.0, self.L, -self.target_alt]),
            'att': np.array([0.0, 0.0, self.deg30])
        })
        t += edge_dur

        # Phase 7: Corner 3 (0, +L) transition Yaw +30° -> Flat 0°
        self.phases.append({
            'name': 'Corner 3 (Yaw 30° -> Flat 0°)',
            't_start': t, 't_end': t + self.corner_dur,
            'type': 'transition',
            'pos': np.array([0.0, self.L, -self.target_alt]),
            'att_start': np.array([0.0, 0.0, self.deg30]),
            'att_end': np.array([0.0, 0.0, 0.0])
        })
        t += self.corner_dur

        # Phase 8: Edge 4 (0, +L) -> (0, 0) Flat normal flight
        self.phases.append({
            'name': 'Edge 4 (-Y, Flat 0°)',
            't_start': t, 't_end': t + edge_dur,
            'type': 'cruise',
            'p_start': np.array([0.0, self.L, -self.target_alt]),
            'p_end': np.array([0.0, 0.0, -self.target_alt]),
            'att': np.array([0.0, 0.0, 0.0])
        })
        t += edge_dur

        # Phase 9: Final settle at (0, 0)
        self.phases.append({
            'name': 'Final Hover & Settle',
            't_start': t, 't_end': t + 3.0,
            'type': 'hover',
            'pos': np.array([0.0, 0.0, -self.target_alt]),
            'att': np.array([0.0, 0.0, 0.0])
        })
        t += 3.0

        self.total_duration = t

    def get_reference_state(self, t):
        """
        Returns full reference state x_ref = [p, v, q, omega, alpha, beta] (21 dims)
        at time t.
        """
        t = np.clip(t, 0.0, self.total_duration)

        # Locate phase
        curr_p = self.phases[-1]
        for p in self.phases:
            if p['t_start'] <= t <= p['t_end']:
                curr_p = p
                break

        p_ref = np.zeros(3)
        v_ref = np.zeros(3)
        att_ref = np.zeros(3)
        omega_ref = np.zeros(3)

        if curr_p['type'] == 'hover':
            p_ref = curr_p['pos'].copy()
            att_ref = curr_p['att'].copy()

        elif curr_p['type'] == 'transition':
            p_ref = curr_p['pos'].copy()
            tau = (t - curr_p['t_start']) / (curr_p['t_end'] - curr_p['t_start'])
            # Quintic polynomial: s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5
            s = 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)
            s_dot = (30.0 * (tau**2) - 60.0 * (tau**3) + 30.0 * (tau**4)) / (curr_p['t_end'] - curr_p['t_start'])

            delta_att = curr_p['att_end'] - curr_p['att_start']
            att_ref = curr_p['att_start'] + delta_att * s
            omega_ref = delta_att * s_dot

        elif curr_p['type'] == 'cruise':
            att_ref = curr_p['att'].copy()
            tau = (t - curr_p['t_start']) / (curr_p['t_end'] - curr_p['t_start'])
            # Quintic trajectory for smooth velocity and acceleration
            s = 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)
            s_dot = (30.0 * (tau**2) - 60.0 * (tau**3) + 30.0 * (tau**4)) / (curr_p['t_end'] - curr_p['t_start'])

            delta_p = curr_p['p_end'] - curr_p['p_start']
            p_ref = curr_p['p_start'] + delta_p * s
            v_ref = delta_p * s_dot

        q_ref = euler_to_quat(att_ref[0], att_ref[1], att_ref[2])

        # Nominal servos neutral (0.0 rad)
        alpha_ref = np.zeros(4)
        beta_ref = np.zeros(4)

        x_ref = np.concatenate([p_ref, v_ref, q_ref, omega_ref, alpha_ref, beta_ref])
        return x_ref, att_ref


if __name__ == "__main__":
    traj = DecoupledTrajectoryGenerator()
    print(f"[TrajectoryGenerator] Total trajectory duration: {traj.total_duration:.1f} s")
    x0, att0 = traj.get_reference_state(0.0)
    print(f"  t=0.0s: Pos={x0[0:3]}, Att={att0}")
    x_mid, att_mid = traj.get_reference_state(10.0)
    print(f"  t=10.0s: Pos={x_mid[0:3]}, Att={att_mid}")
