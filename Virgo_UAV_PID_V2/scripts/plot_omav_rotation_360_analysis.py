#!/usr/bin/env python3
"""
==============================================================================
  Virgo OMAV - 360-Degree Continuous Rotation Flight Data Analysis & Plotter
  Generates publication-quality 6-panel evaluation plot for Roll, Pitch, or Both.
==============================================================================
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def generate_omav_rotation_plot(csv_path, output_png=None):
    if not os.path.exists(csv_path):
        print(f"[ERROR] Không tìm thấy file CSV: {csv_path}")
        return None

    df = pd.read_csv(csv_path)
    t = (df['time'] - df['time'].iloc[0]).to_numpy()

    if output_png is None:
        plot_dir = os.path.join(os.path.dirname(csv_path), "../flight_plots")
        os.makedirs(plot_dir, exist_ok=True)
        base = os.path.splitext(os.path.basename(csv_path))[0]
        output_png = os.path.join(plot_dir, f"{base}_analysis.png")

    fig = plt.figure(figsize=(18, 13))

    # Detect dominant axis
    max_sp_roll = np.degrees(df['sp_roll'].abs().max())
    max_sp_pitch = np.degrees(df['sp_pitch'].abs().max())
    if max_sp_roll > 100 and max_sp_pitch > 100:
        maneuver_title = "Roll & Pitch 360° Sequential"
    elif max_sp_pitch > 100:
        maneuver_title = "360° Decoupled Pitch Rotation"
    else:
        maneuver_title = "360° Decoupled Roll Rotation"

    plt.suptitle(f"Virgo OMAV - {maneuver_title} & Safe Touchdown Analysis (Quasi-Static PID)", 
                 fontsize=16, fontweight='bold', y=0.98)

    # 1. Position Tracking & Altitude (X, Y, Z)
    ax1 = plt.subplot(3, 2, 1)
    ax1.plot(t, df['pos_x'], 'b-', lw=1.8, label='X (Actual)')
    ax1.plot(t, df['sp_x'], 'b--', lw=1.2, alpha=0.7, label='X (Cmd)')
    ax1.plot(t, df['pos_y'], 'g-', lw=1.8, label='Y (Actual)')
    ax1.plot(t, df['sp_y'], 'g--', lw=1.2, alpha=0.7, label='Y (Cmd)')
    ax1.plot(t, -df['pos_z'], 'r-', lw=2.0, label='Altitude (-Z)')
    ax1.plot(t, -df['sp_z'], 'r--', lw=1.2, alpha=0.7, label='Alt Setpoint')
    ax1.set_title("1. Position Tracking & Altitude Hold (m)", fontweight='bold')
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position (m)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best', fontsize=9)

    # 2. Attitude Tracking (Roll, Pitch, Yaw)
    ax2 = plt.subplot(3, 2, 2)
    sp_roll_deg = np.degrees(df['sp_roll'].to_numpy())
    sp_pitch_deg = np.degrees(df['sp_pitch'].to_numpy())

    if 'roll_cont' in df.columns:
        act_roll_deg = np.degrees(df['roll_cont'].to_numpy())
    else:
        act_roll_deg = np.degrees(np.unwrap(df['roll'].to_numpy()))

    if 'pitch_cont' in df.columns:
        act_pitch_deg = np.degrees(df['pitch_cont'].to_numpy())
    else:
        act_pitch_deg = np.degrees(np.unwrap(df['pitch'].to_numpy()))

    # Clean zero-centering for unwrapped representation
    if abs(act_roll_deg[0]) > 180: act_roll_deg -= np.sign(act_roll_deg[0]) * 360
    if abs(act_pitch_deg[0]) > 180: act_pitch_deg -= np.sign(act_pitch_deg[0]) * 360

    if max_sp_roll > 100:
        ax2.plot(t, sp_roll_deg, 'r--', lw=2.0, label='Roll Setpoint')
        ax2.plot(t, act_roll_deg, 'r-', lw=1.8, label='Roll Actual')
    else:
        ax2.plot(t, act_roll_deg, 'r-', lw=1.0, alpha=0.6, label='Roll Actual')

    if max_sp_pitch > 100:
        ax2.plot(t, sp_pitch_deg, 'b--', lw=2.0, label='Pitch Setpoint')
        ax2.plot(t, act_pitch_deg, 'b-', lw=1.8, label='Pitch Actual')
    else:
        ax2.plot(t, act_pitch_deg, 'b-', lw=1.0, alpha=0.6, label='Pitch Actual')

    ax2.plot(t, np.degrees(df['yaw']), 'k-', lw=1.0, alpha=0.7, label='Yaw')
    ax2.axhline(90, color='gray', linestyle=':', alpha=0.5, label='±90° Knife-edge')
    ax2.axhline(-90, color='gray', linestyle=':', alpha=0.5)
    ax2.axhline(180, color='purple', linestyle=':', alpha=0.5, label='180° Inverted')
    ax2.axhline(360, color='green', linestyle=':', alpha=0.5, label='360° Complete')
    ax2.set_title("2. 360° Continuous Attitude Tracking (°)", fontweight='bold')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Euler Angle (°)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=8)

    # 3. 8 BLDC Motor Throttles
    ax3 = plt.subplot(3, 2, 3)
    colors = ['#1f77b4', '#aec7e8', '#ff7f0e', '#ffbb78', '#2ca02c', '#98df8a', '#d62728', '#ff9896']
    for i in range(8):
        ax3.plot(t, df[f'motor_{i}'], color=colors[i], lw=1.2, label=f'M{i}')
    ax3.axhline(0.35, color='black', linestyle='--', alpha=0.5, label='Nominal Hover (0.35)')
    ax3.set_title("3. 8 BLDC Motor Throttles (Coaxial Counter-Rotating)", fontweight='bold')
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Normalized Throttle [0, 1]")
    ax3.set_ylim(-0.05, 1.05)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right', ncol=4, fontsize=8)

    # 4. 8 Gimbal Servo Continuous Angles (Alpha & Beta)
    ax4 = plt.subplot(3, 2, 4)
    for i in range(4):
        ax4.plot(t, np.degrees(df[f'servo_beta_{i}']), lw=1.4, label=f'Beta (Roll) P{i}')
    for i in range(4):
        ax4.plot(t, np.degrees(df[f'servo_alpha_{i}']), '--', lw=1.4, alpha=0.7, label=f'Alpha (Pitch) P{i}')
    ax4.axhline(0, color='gray', linestyle=':', alpha=0.4)
    ax4.axhline(-180, color='purple', linestyle=':', alpha=0.4, label='-180° Inverted')
    ax4.axhline(-360, color='red', linestyle=':', alpha=0.4, label='-360° Vector')
    ax4.set_title("4. 8 Gimbal Servo Continuous Vectoring (Alpha & Beta)", fontweight='bold')
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Servo Angle (°)")
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc='best', ncol=2, fontsize=8)

    # 5. Tracking & Stationkeeping Errors
    ax5 = plt.subplot(3, 2, 5)
    err_x = df['pos_x'] - df['sp_x']
    err_y = df['pos_y'] - df['sp_y']
    err_z = (-df['pos_z']) - (-df['sp_z'])
    ax5.plot(t, err_x, 'b-', lw=1.5, label='Error X')
    ax5.plot(t, err_y, 'g-', lw=1.5, label='Error Y')
    ax5.plot(t, err_z, 'r-', lw=1.8, label='Error Z (Alt)')
    ax5.axhline(0.25, color='gray', linestyle='--', alpha=0.5, label='±0.25m Hover Gate')
    ax5.axhline(-0.25, color='gray', linestyle='--', alpha=0.5)
    ax5.set_title("5. Tracking & Stationkeeping Errors (m)", fontweight='bold')
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Position Error (m)")
    err_max = max(abs(err_x).max(), abs(err_y).max(), abs(err_z).max()) * 1.15
    ax5.set_ylim(-max(err_max, 0.8), max(err_max, 0.8))
    ax5.grid(True, alpha=0.3)
    ax5.legend(loc='best', fontsize=9)

    # 6. 2D Horizontal Trajectory (X vs Y)
    ax6 = plt.subplot(3, 2, 6)
    ax6.plot(df['pos_x'], df['pos_y'], 'm-', lw=2.0, label='Flight Path')
    ax6.plot(df['sp_x'].iloc[0], df['sp_y'].iloc[0], 'ro', markersize=8, label='Origin Setpoint')
    ax6.plot(df['pos_x'].iloc[0], df['pos_y'].iloc[0], 'go', markersize=6, label='Takeoff Point')
    ax6.plot(df['pos_x'].iloc[-1], df['pos_y'].iloc[-1], 'k^', markersize=8, label='Touchdown Point')
    circle_025m = plt.Circle((0, 0), 0.25, color='green', fill=False, linestyle='--', label='±0.25m Boundary')
    circle_050m = plt.Circle((0, 0), 0.50, color='orange', fill=False, linestyle=':', label='±0.50m Boundary')
    ax6.add_patch(circle_025m)
    ax6.add_patch(circle_050m)
    ax6.set_title("6. 2D Horizontal Trajectory & Touchdown (X vs Y)", fontweight='bold')
    ax6.set_xlabel("X (m)")
    ax6.set_ylabel("Y (m)")
    max_range = max(abs(df['pos_x']).max(), abs(df['pos_y']).max()) * 1.25
    ax6.set_xlim(-max(max_range, 1.0), max(max_range, 1.0))
    ax6.set_ylim(-max(max_range, 1.0), max(max_range, 1.0))
    ax6.set_aspect('equal')
    ax6.grid(True, alpha=0.3)
    ax6.legend(loc='best', fontsize=8)

    plt.tight_layout()
    plt.savefig(output_png, dpi=200)
    plt.close()
    print(f"\033[1;32m[PLOT GENERATED]\033[0m Đã xuất biểu đồ phân tích 6 panel:\n  -> {output_png}")
    return output_png


if __name__ == '__main__':
    if len(sys.argv) > 1:
        generate_omav_rotation_plot(sys.argv[1])
    else:
        print("Sử dụng: python3 plot_omav_rotation_360_analysis.py <path_to_csv>")
