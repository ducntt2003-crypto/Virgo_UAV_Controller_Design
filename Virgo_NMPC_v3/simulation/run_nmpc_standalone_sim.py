#!/usr/bin/env python3
"""
==============================================================================
 High-Fidelity Closed-Loop NMPC Simulation for Virgo OMAV
 Benchmark: 25m x 25m Decoupled 6-DOF Square Trajectory with Corner Transitions
==============================================================================
"""

import gzip
import math
import os
import sys
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.append(os.path.join(os.path.dirname(__file__), "../models"))
sys.path.append(os.path.join(os.path.dirname(__file__), "../controllers"))

from virgo_casadi_model import VirgoCasadiModel
from virgo_nmpc_controller import VirgoNMPCController
from se3_kinematics import quat_to_euler, euler_to_quat


def run_nmpc_benchmark_simulation(edge_length=25.0, max_time=None):
    """
    Executes closed-loop NMPC numerical simulation of the 25m square decoupled mission.
    """
    print("=" * 78)
    print(" Virgo OMAV - 6-DOF Decoupled NMPC Benchmark Simulation (4.56kg)")
    print(f" Trajectory: {edge_length}m x {edge_length}m Square with 30° Corner Attitude Transitions")
    print("=" * 78)

    controller = VirgoNMPCController()
    plant_model = VirgoCasadiModel()
    dt = controller.dt
    f_rk4_plant = plant_model.create_rk4_integrator(dt)

    total_duration = controller.traj_gen.total_duration
    if max_time is not None:
        total_duration = min(total_duration, max_time)

    num_steps = int(total_duration / dt)
    print(f"Total mission duration: {total_duration:.1f} s ({num_steps} steps at {1.0/dt:.0f} Hz)\n")

    # Initial state: Hover at origin (0, 0, -1.5m NED)
    x_curr = np.zeros(21)
    x_curr[2] = -1.5  # Altitude 1.5m AGL
    x_curr[6] = 1.0   # qw = 1.0 (identity orientation)

    telemetry = []
    t_sim_start = time.time()

    # Progress reporting
    report_interval = max(1, int(2.0 / dt))

    for step in range(num_steps):
        t_curr = step * dt

        # Update controller with plant state
        pos = x_curr[0:3]
        vel = x_curr[3:6]
        quat = x_curr[6:10]
        omega = x_curr[10:13]
        alpha = x_curr[13:17]
        beta = x_curr[17:21]

        controller.update_state(pos, vel, quat, omega, alpha, beta)

        # Get reference state for logging
        xref_now, att_ref_now = controller.traj_gen.get_reference_state(t_curr)

        # Solve NMPC
        ctrl_res = controller.compute_control(t_curr)
        u_opt = np.concatenate([
            ctrl_res['pod_thrusts'],
            ctrl_res['dot_alpha'],
            ctrl_res['dot_beta'],
            [ctrl_res['tau_yaw']]
        ])

        # Step plant forward in time using RK4
        x_next = f_rk4_plant(x_curr, u_opt).full().flatten()
        # Add slight realistic sensor noise
        x_curr = x_next.copy()

        # Extract Euler angles
        r_deg, p_deg, y_deg = np.degrees(quat_to_euler(quat))
        r_ref_deg, p_ref_deg, y_ref_deg = np.degrees(att_ref_now)

        # Telemetry row
        row = {
            'time': t_curr,
            'pos_x': pos[0], 'pos_y': pos[1], 'pos_z': pos[2],
            'vel_x': vel[0], 'vel_y': vel[1], 'vel_z': vel[2],
            'roll': math.radians(r_deg), 'pitch': math.radians(p_deg), 'yaw': math.radians(y_deg),
            'sp_x': xref_now[0], 'sp_y': xref_now[1], 'sp_z': xref_now[2],
            'sp_vx': xref_now[3], 'sp_vy': xref_now[4], 'sp_vz': xref_now[5],
            'sp_roll': att_ref_now[0], 'sp_pitch': att_ref_now[1], 'sp_yaw': att_ref_now[2],
            'solve_time_ms': ctrl_res['solve_time_ms']
        }
        for i in range(4):
            row[f'thrust_pod_{i}'] = ctrl_res['pod_thrusts'][i]
            row[f'servo_alpha_{i}'] = alpha[i]
            row[f'servo_beta_{i}'] = beta[i]

        telemetry.append(row)

        if step % report_interval == 0:
            print(f"\r  [Sim t={t_curr:5.1f}s / {total_duration:.1f}s] "
                  f"Pos: ({pos[0]:+5.1f}, {pos[1]:+5.1f}, {-pos[2]:4.2f})m | "
                  f"Att: R={r_deg:+5.1f}°, P={p_deg:+5.1f}°, Y={y_deg:+5.1f}° | "
                  f"NMPC: {ctrl_res['solve_time_ms']:4.1f}ms", end="", flush=True)

    print("\n\n\033[1;32m[SIMULATION COMPLETED]\033[0m Processing telemetry and generating figures...")

    df = pd.DataFrame(telemetry)

    # Save compressed CSV
    csv_dir = os.path.join(os.path.dirname(__file__), "../docs")
    os.makedirs(csv_dir, exist_ok=True)
    csv_path = os.path.join(csv_dir, "nmpc_square_decoupled_telemetry.csv.gz")
    with gzip.open(csv_path, "wt", encoding="utf-8") as f:
        df.to_csv(f, index=False)
    print(f"  Telemetry log saved to: {csv_path}")

    # Generate Analysis Plot
    plot_path = os.path.join(csv_dir, "nmpc_square_decoupled_analysis.png")
    plot_nmpc_telemetry(df, plot_path)
    print(f"  Analysis figure saved to: {plot_path}")

    return df


def plot_nmpc_telemetry(df, output_path):
    """Generates publication-quality 6-panel analysis for NMPC flight data."""
    fig, axs = plt.subplots(3, 2, figsize=(16, 12))
    plt.subplots_adjust(hspace=0.32, wspace=0.22)

    # 1. 2D Trajectory X-Y
    ax = axs[0, 0]
    ax.plot(df['sp_y'], df['sp_x'], 'k--', label='Reference Trajectory', alpha=0.7)
    ax.plot(df['pos_y'], df['pos_x'], 'b-', linewidth=2, label='NMPC Actual Path')
    ax.set_title('2D Position Trajectory (North vs East)', fontsize=12, fontweight='bold')
    ax.set_xlabel('East Y (m)')
    ax.set_ylabel('North X (m)')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='lower right')
    ax.axis('equal')

    # 2. Position Tracking & Altitude vs Time
    ax = axs[0, 1]
    ax.plot(df['time'], df['sp_x'], 'r--', label='X ref', alpha=0.6)
    ax.plot(df['time'], df['pos_x'], 'r-', label='X act')
    ax.plot(df['time'], df['sp_y'], 'g--', label='Y ref', alpha=0.6)
    ax.plot(df['time'], df['pos_y'], 'g-', label='Y act')
    ax.plot(df['time'], -df['pos_z'], 'b-', linewidth=1.5, label='Altitude Z (m)')
    ax.set_title('Position Tracking vs Time', fontsize=12, fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', ncol=3)

    # 3. Attitude Decoupling (Roll, Pitch, Yaw)
    ax = axs[1, 0]
    ax.plot(df['time'], np.degrees(df['sp_roll']), 'r--', label='Roll Ref (30° Edge 1)', alpha=0.7)
    ax.plot(df['time'], np.degrees(df['roll']), 'r-', label='Roll Actual')
    ax.plot(df['time'], np.degrees(df['sp_pitch']), 'g--', label='Pitch Ref (30° Edge 2)', alpha=0.7)
    ax.plot(df['time'], np.degrees(df['pitch']), 'g-', label='Pitch Actual')
    ax.plot(df['time'], np.degrees(df['sp_yaw']), 'b--', label='Yaw Ref (30° Edge 3)', alpha=0.7)
    ax.plot(df['time'], np.degrees(df['yaw']), 'b-', label='Yaw Actual')
    ax.set_title('6-DOF Decoupled Attitude Tracking', fontsize=12, fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Euler Angle (deg)')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', ncol=3)

    # 4. Tracking Errors
    ax = axs[1, 1]
    pos_err = np.sqrt((df['pos_x'] - df['sp_x'])**2 + (df['pos_y'] - df['sp_y'])**2)
    roll_err = np.abs(np.degrees(df['roll'] - df['sp_roll']))
    pitch_err = np.abs(np.degrees(df['pitch'] - df['sp_pitch']))
    yaw_err = np.abs(np.degrees((df['yaw'] - df['sp_yaw'] + np.pi) % (2*np.pi) - np.pi))

    ax.plot(df['time'], pos_err, 'm-', label=f'XY Pos Error (Mean={pos_err.mean():.2f}m)')
    ax.plot(df['time'], roll_err, 'r:', label=f'Roll Error (Mean={roll_err.mean():.2f}°)')
    ax.plot(df['time'], pitch_err, 'g:', label=f'Pitch Error (Mean={pitch_err.mean():.2f}°)')
    ax.plot(df['time'], yaw_err, 'b:', label=f'Yaw Error (Mean={yaw_err.mean():.2f}°)')
    ax.set_title('Tracking Errors vs Time', fontsize=12, fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Tracking Error')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right')

    # 5. Pod Thrust Actuation
    ax = axs[2, 0]
    has_thrust = any(f'thrust_pod_{i}' in df for i in range(4))
    for i in range(4):
        if has_thrust:
            ax.plot(df['time'], df[f'thrust_pod_{i}'], label=f'Pod {i+1} Thrust')
        elif f'motor_{i}' in df:
            ax.plot(df['time'], df[f'motor_{i}'], label=f'Motor {i} PWM')
    ax.axhline(11.18, color='k', linestyle=':', label='Hover Thrust (11.18N)')
    ax.set_title('Actuator Allocation: Pod Thrusts / Motors', fontsize=12, fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Thrust / PWM')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', ncol=2)

    # 6. Servo Gimbal Angles & Slew Smoothness
    ax = axs[2, 1]
    for i in range(4):
        ax.plot(df['time'], np.degrees(df[f'servo_alpha_{i}']), label=f'Pod {i+1} Pitch α')
        ax.plot(df['time'], np.degrees(df[f'servo_beta_{i}']), ':', label=f'Pod {i+1} Roll β')
    ax.set_title('Gimbal Tilt Angles (Outer Pitch α, Inner Roll β)', fontsize=12, fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (deg)')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', ncol=4, fontsize=8)

    plt.suptitle('Virgo OMAV SE(3) NMPC 25m Square Decoupled Flight Benchmark', fontsize=15, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path, dpi=250)
    plt.close()


if __name__ == "__main__":
    run_nmpc_benchmark_simulation()
