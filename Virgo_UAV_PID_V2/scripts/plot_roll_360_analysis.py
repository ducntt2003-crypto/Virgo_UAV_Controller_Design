#!/usr/bin/env python3
"""
==============================================================================
  Virgo OMAV - 360-Degree Roll Flight Data Analysis & Plotter
  Generates publication-quality 6-panel evaluation plot from flight CSV.
==============================================================================
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def generate_roll_360_plot(csv_path, output_png=None):
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
    plt.suptitle("Virgo OMAV - 360-Degree Roll Flight & Hover Hold Analysis (Optimized PID)", 
                 fontsize=16, fontweight='bold', y=0.98)

    # 1. Position Tracking & Altitude (X, Y, Z)
    ax1 = plt.subplot(3, 2, 1)
    ax1.plot(t, df['pos_x'], 'b-', lw=1.8, label='X (Actual)')
    ax1.plot(t, df['sp_x'], 'b--', lw=1.2, alpha=0.7, label='X (Cmd)')
    ax1.plot(t, df['pos_y'], 'g-', lw=1.8, label='Y (Actual)')
    ax1.plot(t, df['sp_y'], 'g--', lw=1.2, alpha=0.7, label='Y (Cmd)')
    ax1.plot(t, -df['pos_z'], 'r-', lw=2.0, label='Altitude (-Z)')
    ax1.plot(t, -df['sp_z'], 'r--', lw=1.2, alpha=0.7, label='Alt (Cmd = 2.5m)')
    ax1.set_title("1. Position Tracking & Altitude Hold (m)", fontweight='bold')
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position (m)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best', fontsize=9)

    # 2. Roll Rotation Tracking (0 to 360 deg)
    ax2 = plt.subplot(3, 2, 2)
    # Unwrap roll to continuous angle for clean tracking view
    roll_raw = df['roll'].to_numpy()
    roll_unwrapped = np.unwrap(roll_raw)
    sp_roll_deg = np.degrees(df['sp_roll'].to_numpy())
    act_roll_deg = np.degrees(roll_unwrapped)
    # If starting negative, shift by 360 if appropriate
    if act_roll_deg[0] < -100:
        act_roll_deg += 360

    ax2.plot(t, sp_roll_deg, 'r--', lw=2.0, label='Roll Setpoint (0° -> 360°)')
    ax2.plot(t, act_roll_deg, 'r-', lw=1.8, label='Roll Actual (Unwrapped)')
    ax2.plot(t, np.degrees(df['pitch']), 'b-', lw=1.0, alpha=0.6, label='Pitch')
    ax2.plot(t, np.degrees(df['yaw']), 'k-', lw=1.0, alpha=0.6, label='Yaw')
    ax2.axhline(90, color='gray', linestyle=':', alpha=0.6, label='Knife-edge 90°')
    ax2.axhline(180, color='purple', linestyle=':', alpha=0.6, label='Inverted 180°')
    ax2.set_title("2. Full 360° Roll Attitude Tracking (°)", fontweight='bold')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Euler Angle (°)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=9)

    # 3. 8 BLDC Motor Throttles (4 Coaxial Counter-Rotating Pairs)
    ax3 = plt.subplot(3, 2, 3)
    colors = ['#1f77b4', '#aec7e8', '#ff7f0e', '#ffbb78', '#2ca02c', '#98df8a', '#d62728', '#ff9896']
    for i in range(8):
        ax3.plot(t, df[f'motor_{i}'], color=colors[i], lw=1.2, label=f'M{i}')
    ax3.axhline(0.35, color='black', linestyle='--', alpha=0.5, label='Hover Throttle (0.35)')
    ax3.set_title("3. 8 BLDC Motor Throttles (Coaxial Pairs)", fontweight='bold')
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Normalized Throttle [0, 1]")
    ax3.set_ylim(-0.05, 1.05)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right', ncol=4, fontsize=8)

    # 4. 8 Gimbal Servo Angles (Alpha Pitch & Beta Roll)
    ax4 = plt.subplot(3, 2, 4)
    for i in range(4):
        ax4.plot(t, np.degrees(df[f'servo_beta_{i}']), lw=1.5, label=f'Beta (Roll) Pod {i}')
    ax4.plot(t, np.degrees(df['servo_alpha_0']), 'k--', alpha=0.5, label='Alpha (Pitch) Pod 0')
    ax4.axhline(0, color='gray', linestyle=':', alpha=0.4)
    ax4.axhline(-180, color='purple', linestyle=':', alpha=0.4, label='-180° Inverted')
    ax4.axhline(-360, color='red', linestyle=':', alpha=0.4, label='-360° Complete Decoupled Roll')
    ax4.set_title("4. 8 Gimbal Servo Continuous Vectoring (0° -> -360°)", fontweight='bold')
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Servo Angle (°)")
    ax4.set_ylim(-380, 60)
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc='best', fontsize=8)

    # 5. Position & Altitude Drift Errors
    ax5 = plt.subplot(3, 2, 5)
    err_x = df['pos_x'] - df['sp_x']
    err_y = df['pos_y'] - df['sp_y']
    err_z = (-df['pos_z']) - (-df['sp_z'])
    ax5.plot(t, err_x, 'b-', lw=1.5, label='Error X')
    ax5.plot(t, err_y, 'g-', lw=1.5, label='Error Y')
    ax5.plot(t, err_z, 'r-', lw=1.8, label='Error Z (Altitude)')
    ax5.axhline(0.25, color='gray', linestyle='--', alpha=0.5, label='±0.25m Hover Gate')
    ax5.axhline(-0.25, color='gray', linestyle='--', alpha=0.5)
    ax5.set_title("5. Tracking & Stationkeeping Errors (m)", fontweight='bold')
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Position Error (m)")
    err_max = max(abs(err_x).max(), abs(err_y).max(), abs(err_z).max()) * 1.15
    ax5.set_ylim(-max(err_max, 1.0), max(err_max, 1.0))
    ax5.grid(True, alpha=0.3)
    ax5.legend(loc='best', fontsize=9)

    # 6. 2D Horizontal Trajectory (X vs Y)
    ax6 = plt.subplot(3, 2, 6)
    ax6.plot(df['pos_x'], df['pos_y'], 'm-', lw=2.0, label='Flight Path')
    ax6.plot(df['sp_x'].iloc[0], df['sp_y'].iloc[0], 'ro', markersize=8, label='Setpoint (0, 0)')
    ax6.plot(df['pos_x'].iloc[0], df['pos_y'].iloc[0], 'go', markersize=6, label='Takeoff Point')
    circle_02m = plt.Circle((0, 0), 0.25, color='green', fill=False, linestyle='--', label='±0.25m Boundary')
    ax6.add_patch(circle_02m)
    ax6.set_title("6. 2D Horizontal Stationkeeping Drift (X vs Y)", fontweight='bold')
    ax6.set_xlabel("X (m)")
    ax6.set_ylabel("Y (m)")
    max_range = max(abs(df['pos_x']).max(), abs(df['pos_y']).max()) * 1.15
    ax6.set_xlim(-max(max_range, 1.5), max(max_range, 1.5))
    ax6.set_ylim(-max(max_range, 1.5), max(max_range, 1.5))
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
        generate_roll_360_plot(sys.argv[1])
    else:
        print("Sử dụng: python3 plot_roll_360_analysis.py <path_to_csv>")
