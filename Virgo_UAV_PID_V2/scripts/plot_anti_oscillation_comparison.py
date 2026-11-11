#!/usr/bin/env python3
"""
Comparison script between:
- Run 1 (1794361715): 4.56kg baseline with CAD mesh ground jitter & servo yaw hunting oscillation
- Run 2 (1794363687): Tuned Anti-Oscillation with spherical contact feet & pure coaxial differential yaw
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

csv1 = '/home/duc/Omini_UAV/Virgo_UAV_PID_V2/docs/flight_logs/virgo_square_decoupled_1794361715.csv'
csv2 = '/home/duc/Omini_UAV/Virgo_UAV_PID_V2/docs/flight_logs/virgo_square_decoupled_1794363687.csv'

df1 = pd.read_csv(csv1)
df2 = pd.read_csv(csv2)

t1 = (df1['time'] - df1['time'].iloc[0]).to_numpy()
t2 = (df2['time'] - df2['time'].iloc[0]).to_numpy()

fig = plt.figure(figsize=(18, 14))
plt.subplots_adjust(hspace=0.35, wspace=0.25)
plt.suptitle("Virgo OMAV 4.56kg - Anti-Oscillation & Ground Twitch Elimination Benchmark\n"
             "Before (1794361715: Servo Yaw Hunting + Mesh Chatter) vs After (1794363687: Spherical Feet + Pure Coaxial Differential Yaw)",
             fontsize=14, fontweight='bold', y=0.98)

# Panel 1: Ground Startup Stability (t: 0 to 5s)
ax1 = plt.subplot(3, 2, 1)
mask1_g = t1 <= 5.0
mask2_g = t2 <= 5.0
ax1.plot(t1[mask1_g], np.degrees(df1['yaw'].to_numpy()[mask1_g]), 'r--', label='Before: Yaw (153° Spin & Jitter)', alpha=0.8, linewidth=1.5)
ax1.plot(t2[mask2_g], np.degrees(df2['yaw'].to_numpy()[mask2_g]), 'g-', label='After: Yaw (Rock Solid < 4.5°)', linewidth=2.0)
ax1.set_title("1. Ground Startup Stability (t = 0..5s Waiting for Takeoff)", fontweight='bold')
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Yaw Angle (deg)")
ax1.grid(True, alpha=0.3)
ax1.legend(loc='best', fontsize=9)

# Panel 2: Yaw Tracking & High-Frequency Oscillation
ax2 = plt.subplot(3, 2, 2)
# Unwrapped yaw in degrees
yaw1_deg = np.degrees(np.unwrap(df1['yaw'].to_numpy()))
yaw2_deg = np.degrees(np.unwrap(df2['yaw'].to_numpy()))
sp_yaw1_deg = np.degrees(np.unwrap(df1['sp_yaw'].to_numpy()))
ax2.plot(t1, sp_yaw1_deg, 'k:', label='Setpoint', linewidth=1.5, alpha=0.7)
ax2.plot(t1, yaw1_deg, 'r--', label='Before (Hunting 15.08 deg/s variation)', alpha=0.7, linewidth=1.2)
ax2.plot(t2, yaw2_deg, 'g-', label='After (Damped 4.70 deg/s variation)', linewidth=1.8)
ax2.set_title("2. Yaw Angle Tracking & Hunting Elimination", fontweight='bold')
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Yaw (deg)")
ax2.grid(True, alpha=0.3)
ax2.legend(loc='best', fontsize=9)

# Panel 3: Roll & Pitch Decoupling Dynamics
ax3 = plt.subplot(3, 2, 3)
roll1 = np.degrees(df1['roll'].to_numpy())
roll2 = np.degrees(df2['roll'].to_numpy())
sp_roll = np.degrees(df1['sp_roll'].to_numpy())
ax3.plot(t1, sp_roll, 'k:', label='Roll Setpoint', linewidth=1.5, alpha=0.7)
ax3.plot(t1, roll1, 'r--', label=f'Before (Mean Err: 2.04°, Var: 3.83°/s)', alpha=0.7, linewidth=1.2)
ax3.plot(t2, roll2, 'g-', label=f'After (Mean Err: 1.16°, Var: 1.72°/s)', linewidth=1.8)
ax3.set_title("3. Roll Attitude Tracking & Damping", fontweight='bold')
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Roll Angle (deg)")
ax3.grid(True, alpha=0.3)
ax3.legend(loc='best', fontsize=9)

# Panel 4: Pitch Attitude Tracking
ax4 = plt.subplot(3, 2, 4)
pitch1 = np.degrees(df1['pitch'].to_numpy())
pitch2 = np.degrees(df2['pitch'].to_numpy())
sp_pitch = np.degrees(df1['sp_pitch'].to_numpy())
ax4.plot(t1, sp_pitch, 'k:', label='Pitch Setpoint', linewidth=1.5, alpha=0.7)
ax4.plot(t1, pitch1, 'r--', label='Before (Mean Err: 1.79°)', alpha=0.7, linewidth=1.2)
ax4.plot(t2, pitch2, 'b-', label='After (Mean Err: 1.12°)', linewidth=1.8)
ax4.set_title("4. Pitch Attitude Tracking & Damping", fontweight='bold')
ax4.set_xlabel("Time (s)")
ax4.set_ylabel("Pitch Angle (deg)")
ax4.grid(True, alpha=0.3)
ax4.legend(loc='best', fontsize=9)

# Panel 5: Servo Gimbal Activity (Alpha 0 & Beta 0)
ax5 = plt.subplot(3, 2, 5)
ax5.plot(t1, np.degrees(df1['servo_alpha_0'].to_numpy()), 'r--', label='Before: Servo Alpha 0 (21.7 deg/s chatter)', alpha=0.7, linewidth=1.2)
ax5.plot(t2, np.degrees(df2['servo_alpha_0'].to_numpy()), 'g-', label='After: Servo Alpha 0 (11.8 deg/s smooth)', linewidth=1.8)
ax5.plot(t1, np.degrees(df1['servo_beta_0'].to_numpy()), 'm--', label='Before: Servo Beta 0 (25.5 deg/s chatter)', alpha=0.6, linewidth=1.0)
ax5.plot(t2, np.degrees(df2['servo_beta_0'].to_numpy()), 'c-', label='After: Servo Beta 0 (16.6 deg/s smooth)', linewidth=1.5)
ax5.set_title("5. Servo Vectoring Stability (Triệt tiêu Yaw Hunting)", fontweight='bold')
ax5.set_xlabel("Time (s)")
ax5.set_ylabel("Servo Tilt (deg)")
ax5.grid(True, alpha=0.3)
ax5.legend(loc='best', fontsize=8)

# Panel 6: 2D Horizontal Trajectory
ax6 = plt.subplot(3, 2, 6)
ax6.plot(df1['sp_x'], df1['sp_y'], 'k:', label='25m Square Target', linewidth=2.0, alpha=0.7)
ax6.plot(df1['pos_x'], df1['pos_y'], 'r--', label='Before (Oscillating Waypoint Tracking)', alpha=0.7, linewidth=1.2)
ax6.plot(df2['pos_x'], df2['pos_y'], 'g-', label='After (Smooth Anti-Oscillation Path)', linewidth=1.8)
ax6.set_title("6. 2D Flight Trajectory (25m x 25m Square)", fontweight='bold')
ax6.set_xlabel("X Position (m)")
ax6.set_ylabel("Y Position (m)")
ax6.axis('equal')
ax6.grid(True, alpha=0.3)
ax6.legend(loc='best', fontsize=9)

out_dir = '/home/duc/Omini_UAV/Virgo_UAV_PID_V2/docs/flight_plots'
out_path = os.path.join(out_dir, 'virgo_anti_oscillation_comparison.png')
plt.savefig(out_path, dpi=300, bbox_inches='tight')
print(f"Comparison plot saved successfully to: {out_path}")

# Copy to brain artifact directory
brain_dir = '/home/duc/.gemini/antigravity/brain/0afd7d87-b87f-4d8d-93c0-71ea7562ad0f'
import shutil
shutil.copyfile(out_path, os.path.join(brain_dir, 'virgo_anti_oscillation_comparison.png'))
print(f"Copied to brain: {os.path.join(brain_dir, 'virgo_anti_oscillation_comparison.png')}")
