#!/usr/bin/env python3
"""
==============================================================================
 Virgo_UAV Flight Data Plotter
 Generates publication-quality 6-panel analysis plots from recorded CSV flight logs.
 Panels:
   1. 3D Trajectory & Position vs Time (X, Y, Z)
   2. Euler Angles (Roll, Pitch, Yaw) - Actual vs Commanded
   3. 8 BLDC Motor Throttles (4 Coaxial Counter-Rotating Pairs)
   4. 8 Gimbal Servo Angles (Alpha: Pitch Tilt, Beta: Roll Tilt)
   5. Decoupling Metrics (Attitude Error during translation / Position Drift during tilt)
   6. 2D Horizontal Flight Trajectory (X-Y)
==============================================================================
"""

import os
import sys
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def plot_flight_log(csv_file, output_png=None):
    if not os.path.exists(csv_file):
        print(f"[ERROR] CSV file not found: {csv_file}")
        return False

    df = pd.read_csv(csv_file)
    t = (df['time'] - df['time'].iloc[0]).to_numpy()

    if output_png is None:
        base = os.path.splitext(csv_file)[0]
        output_png = f"{base}_analysis.png"

    fig = plt.figure(figsize=(16, 12))
    plt.suptitle(f"Virgo_UAV 6-DOF Flight Analysis: {os.path.basename(csv_file)}", fontsize=15, fontweight='bold')

    # Panel 1: Position Tracking (X, Y, Z)
    ax1 = plt.subplot(3, 2, 1)
    ax1.plot(t, df['pos_x'].to_numpy(), 'b-', label='X (Actual)')
    if 'sp_x' in df.columns:
        ax1.plot(t, df['sp_x'].to_numpy(), 'b--', alpha=0.7, label='X (Setpoint)')
    ax1.plot(t, df['pos_y'].to_numpy(), 'g-', label='Y (Actual)')
    if 'sp_y' in df.columns:
        ax1.plot(t, df['sp_y'].to_numpy(), 'g--', alpha=0.7, label='Y (Setpoint)')
    ax1.plot(t, -df['pos_z'].to_numpy(), 'r-', label='Altitude (-Z)')
    if 'sp_z' in df.columns:
        ax1.plot(t, -df['sp_z'].to_numpy(), 'r--', alpha=0.7, label='Alt Setpoint')
    ax1.set_title("1. Position Tracking (m)", fontweight='bold')
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position (m)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best', fontsize=8)

    # Panel 2: Euler Angles (Roll, Pitch, Yaw)
    ax2 = plt.subplot(3, 2, 2)
    ax2.plot(t, np.degrees(df['roll'].to_numpy()), 'r-', label='Roll (Actual)')
    if 'sp_roll' in df.columns:
        ax2.plot(t, np.degrees(df['sp_roll'].to_numpy()), 'r--', alpha=0.7, label='Roll (Setpoint)')
    ax2.plot(t, np.degrees(df['pitch'].to_numpy()), 'g-', label='Pitch (Actual)')
    if 'sp_pitch' in df.columns:
        ax2.plot(t, np.degrees(df['sp_pitch'].to_numpy()), 'g--', alpha=0.7, label='Pitch (Setpoint)')
    ax2.plot(t, np.degrees(df['yaw'].to_numpy()), 'b-', label='Yaw (Actual)')
    if 'sp_yaw' in df.columns:
        ax2.plot(t, np.degrees(df['sp_yaw'].to_numpy()), 'b--', alpha=0.7, label='Yaw (Setpoint)')
    ax2.set_title("2. Attitude Angles (deg)", fontweight='bold')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Angle (deg)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=8)

    # Panel 3: Motor Throttles (8 Coaxial Motors)
    ax3 = plt.subplot(3, 2, 3)
    colors = ['#1f77b4', '#aec7e8', '#ff7f0e', '#ffbb78', '#2ca02c', '#98df8a', '#d62728', '#ff9896']
    for i in range(8):
        col = f'motor_{i}'
        if col in df.columns:
            arm_idx = i // 2
            motor_type = "Top" if i % 2 == 0 else "Bottom"
            ax3.plot(t, df[col].to_numpy(), label=f'Arm {arm_idx} {motor_type} (M{i})', color=colors[i], alpha=0.85)
    ax3.set_title("3. Coaxial Motor Throttles (0.0 to 1.0)", fontweight='bold')
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Normalized Throttle")
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right', ncol=2, fontsize=7)

    # Panel 4: Gimbal Servo Angles (Alpha: Pitch tilt, Beta: Roll tilt)
    ax4 = plt.subplot(3, 2, 4)
    servo_colors = ['#1f77b4', '#17becf', '#ff7f0e', '#e377c2', '#2ca02c', '#bcbd22', '#d62728', '#8c564b']
    for i in range(4):
        col_a = f'servo_alpha_{i}'
        col_b = f'servo_beta_{i}'
        if col_a in df.columns:
            ax4.plot(t, np.degrees(df[col_a].to_numpy()), label=f'Arm {i} α (Pitch)', color=servo_colors[2*i])
        if col_b in df.columns:
            ax4.plot(t, np.degrees(df[col_b].to_numpy()), '--', label=f'Arm {i} β (Roll)', color=servo_colors[2*i+1])
    ax4.set_title("4. Gimbal Vectoring Angles (deg)", fontweight='bold')
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Tilt Angle (deg)")
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc='upper right', ncol=2, fontsize=7)

    # Panel 5: Decoupling Metric (Horizontal Drift vs Body Attitude Tilt)
    ax5 = plt.subplot(3, 2, 5)
    tilt_mag = np.degrees(np.sqrt(df['roll'].to_numpy()**2 + df['pitch'].to_numpy()**2))
    ax5.plot(t, tilt_mag, 'purple', label='Total Body Tilt |θ+φ| (deg)')
    ax5.set_title("5. Body Tilt Magnitude (deg)", fontweight='bold')
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Tilt (deg)")
    ax5.grid(True, alpha=0.3)
    ax5.legend(loc='best', fontsize=8)

    # Panel 6: 2D XY Path Trajectory
    ax6 = plt.subplot(3, 2, 6)
    ax6.plot(df['pos_x'].to_numpy(), df['pos_y'].to_numpy(), 'b-', label='Actual Flight Path')
    if 'sp_x' in df.columns and 'sp_y' in df.columns:
        ax6.plot(df['sp_x'].to_numpy(), df['sp_y'].to_numpy(), 'r--', label='Target Path')
    ax6.plot(df['pos_x'].iloc[0], df['pos_y'].iloc[0], 'go', markersize=8, label='Start')
    ax6.plot(df['pos_x'].iloc[-1], df['pos_y'].iloc[-1], 'rs', markersize=8, label='End')
    ax6.set_title("6. Horizontal Flight Trajectory (X-Y)", fontweight='bold')
    ax6.set_xlabel("X (m)")
    ax6.set_ylabel("Y (m)")
    ax6.grid(True, alpha=0.3)
    ax6.axis('equal')
    ax6.legend(loc='best', fontsize=8)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(output_png, dpi=200)
    plt.close()
    print(f"[OK] Flight analysis plot saved to: {output_png}")
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 plot_flight_data.py <path_to_flight_log.csv> [output.png]")
        sys.exit(1)
    csv_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else None
    plot_flight_log(csv_path, out_path)
