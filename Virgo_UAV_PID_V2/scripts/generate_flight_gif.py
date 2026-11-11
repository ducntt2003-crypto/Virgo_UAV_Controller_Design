#!/usr/bin/env python3
"""
==============================================================================
  Virgo OMAV - 3D Simulation Replay GIF Generator
  Renders an animated 3D visualization GIF of Virgo executing maneuvers.
  Displays 3D H-frame pose, tilting pods, and real-time telemetry curves.
==============================================================================
"""

import os
import sys
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation, PillowWriter

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PLOT_DIR = os.path.join(SCRIPT_DIR, "../docs/flight_plots")
os.makedirs(PLOT_DIR, exist_ok=True)

def rotation_matrix(roll, pitch, yaw):
    """Computes DCM from Euler angles (Roll -> Pitch -> Yaw, standard ZYX)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    
    # R_b^w
    R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return R_z @ R_y @ R_x

def generate_gif(csv_path, output_gif=None, max_frames=120, fps=15):
    print(f"\033[1;36m[GIF Generator]\033[0m Đọc dữ liệu từ: {csv_path}")
    df = pd.read_csv(csv_path)
    
    # Filter active flight (start from arming / altitude > 0.1m)
    active = df[df['pos_z'] <= -0.1].copy()
    if len(active) < 10:
        active = df.copy()
        
    n_samples = len(active)
    step = max(1, n_samples // max_frames)
    sampled = active.iloc[::step].reset_index(drop=True)
    n_frames = len(sampled)
    print(f"\033[1;36m[GIF Generator]\033[0m Tổng số frames: {n_frames} (fps={fps}, thời lượng={n_frames/fps:.1f}s)")
    
    # Virgo geometry parameters (H-Frame)
    lx = 0.28  # m
    ly = 0.22  # m
    pod_r = 0.08 # prop radius
    
    # Time vector
    t_start = sampled['time'].iloc[0]
    time_vec = sampled['time'] - t_start
    
    # Full flight time for background curve
    t_full = active['time'] - active['time'].iloc[0]
    roll_full_deg = np.degrees(active['roll'])
    sp_roll_full_deg = np.degrees(active['sp_roll'])
    alt_full = -active['pos_z']
    sp_alt_full = -active['sp_z']
    beta_full_deg = np.degrees(active['servo_beta_0'])
    
    # Setup Figure
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(16, 9), dpi=100)
    fig.patch.set_facecolor('#111318')
    
    # 3D View Axis
    ax_3d = fig.add_subplot(1, 2, 1, projection='3d')
    ax_3d.set_facecolor('#111318')
    
    # Telemetry Subplots (Right column)
    ax_roll = fig.add_subplot(3, 2, 2)
    ax_alt  = fig.add_subplot(3, 2, 4)
    ax_beta = fig.add_subplot(3, 2, 6)
    
    for ax in [ax_roll, ax_alt, ax_beta]:
        ax.set_facecolor('#1a1d24')
        ax.grid(True, linestyle=':', color='#44475a', alpha=0.6)
        ax.tick_params(colors='#e0e0e0', labelsize=9)
        for spine in ax.spines.values():
            spine.set_color('#44475a')
            
    # Static Background Lines on Telemetry
    ax_roll.plot(t_full, sp_roll_full_deg, 'r--', alpha=0.5, label='Setpoint')
    ax_roll.plot(t_full, roll_full_deg, color='#50fa7b', alpha=0.4, label='Measured Roll')
    ax_roll.set_ylabel('Roll Angle [deg]', fontweight='bold', color='#50fa7b', fontsize=10)
    ax_roll.set_title('Roll Attitude Dynamics (0° -> 360°)', fontweight='bold', color='#f8f8f2', fontsize=11)
    ax_roll.legend(loc='upper right', fontsize=8, facecolor='#1a1d24', edgecolor='#44475a')
    
    ax_alt.plot(t_full, sp_alt_full, 'r--', alpha=0.5, label='Target Alt')
    ax_alt.plot(t_full, alt_full, color='#8be9fd', alpha=0.4, label='Measured Alt')
    ax_alt.set_ylabel('Altitude [m]', fontweight='bold', color='#8be9fd', fontsize=10)
    ax_alt.set_title('Altitude Profile (Inverted Lift Analysis)', fontweight='bold', color='#f8f8f2', fontsize=11)
    ax_alt.legend(loc='lower right', fontsize=8, facecolor='#1a1d24', edgecolor='#44475a')
    
    ax_beta.plot(t_full, beta_full_deg, color='#ffb86c', alpha=0.4, label='Servo Beta (Roll)')
    ax_beta.axhline(90, color='#ff5555', linestyle=':', label='Max Limit (+90°)')
    ax_beta.axhline(-90, color='#ff5555', linestyle=':', label='Min Limit (-90°)')
    ax_beta.set_ylabel('Servo Beta [deg]', fontweight='bold', color='#ffb86c', fontsize=10)
    ax_beta.set_xlabel('Flight Time [s]', fontweight='bold', color='#e0e0e0', fontsize=10)
    ax_beta.set_title('Gimbal Servo Tilt Vectoring Deflection', fontweight='bold', color='#f8f8f2', fontsize=11)
    ax_beta.legend(loc='lower right', fontsize=8, facecolor='#1a1d24', edgecolor='#44475a')
    
    # Progress Indicators (points that move)
    pt_roll, = ax_roll.plot([], [], 'o', color='#50fa7b', markersize=7)
    pt_alt,  = ax_alt.plot([], [], 'o', color='#8be9fd', markersize=7)
    pt_beta, = ax_beta.plot([], [], 'o', color='#ffb86c', markersize=7)
    
    def update(frame_idx):
        row = sampled.iloc[frame_idx]
        cur_t = time_vec.iloc[frame_idx]
        
        # Current states
        roll = row['roll']
        pitch = row['pitch']
        yaw = row['yaw']
        pos_z = -row['pos_z']
        beta_0 = row['servo_beta_0']
        
        # 1. Update Telemetry Points
        pt_roll.set_data([cur_t], [math.degrees(roll)])
        pt_alt.set_data([cur_t], [pos_z])
        pt_beta.set_data([cur_t], [math.degrees(beta_0)])
        
        # 2. Update 3D Model
        ax_3d.cla()
        ax_3d.set_facecolor('#111318')
        
        R = rotation_matrix(roll, pitch, yaw)
        
        # Define H-Frame in Body FLU Frame
        # Central beam: along Y from -ly to +ly
        c_beam = np.array([[0, -ly, 0], [0, ly, 0]]).T
        # Left longitudinal beam: at +ly, from -lx to +lx
        l_beam = np.array([[-lx, ly, 0], [lx, ly, 0]]).T
        # Right longitudinal beam: at -ly, from -lx to +lx
        r_beam = np.array([[-lx, -ly, 0], [lx, -ly, 0]]).T
        
        # Arm tip coordinates (4 pods)
        # Pod 0: Front-Left (+lx, +ly)
        # Pod 1: Back-Left (-lx, +ly)
        # Pod 2: Back-Right (-lx, -ly)
        # Pod 3: Front-Right (+lx, -ly)
        pods_b = [
            np.array([lx, ly, 0]),
            np.array([-lx, ly, 0]),
            np.array([-lx, -ly, 0]),
            np.array([lx, -ly, 0])
        ]
        
        # Rotate to World Frame
        c_beam_w = R @ c_beam
        l_beam_w = R @ l_beam
        r_beam_w = R @ r_beam
        
        # Plot Frame Beams
        ax_3d.plot(c_beam_w[0], c_beam_w[1], c_beam_w[2], color='#bd93f9', linewidth=4.0)
        ax_3d.plot(l_beam_w[0], l_beam_w[1], l_beam_w[2], color='#ff79c6', linewidth=4.0)
        ax_3d.plot(r_beam_w[0], r_beam_w[1], r_beam_w[2], color='#ff79c6', linewidth=4.0)
        
        # Plot Body Axes (Red=X_front, Green=Y_left, Blue=Z_up)
        ax_length = 0.20
        origin = np.array([0, 0, 0])
        x_axis = R @ np.array([ax_length, 0, 0])
        y_axis = R @ np.array([0, ax_length, 0])
        z_axis = R @ np.array([0, 0, ax_length])
        
        ax_3d.plot([0, x_axis[0]], [0, x_axis[1]], [0, x_axis[2]], color='#ff5555', linewidth=2.5, label='X (Front)')
        ax_3d.plot([0, y_axis[0]], [0, y_axis[1]], [0, y_axis[2]], color='#50fa7b', linewidth=2.5, label='Y (Left)')
        ax_3d.plot([0, z_axis[0]], [0, z_axis[1]], [0, z_axis[2]], color='#8be9fd', linewidth=2.5, label='Z (Top)')
        
        # Plot 4 Gimbal Pods and Propeller Discs
        colors = ['#f1fa8c', '#50fa7b', '#8be9fd', '#ffb86c']
        for i, pb in enumerate(pods_b):
            pw = R @ pb
            
            # Pod tilt direction (servo beta tilt around X_b)
            # Thrust vector unit in body
            beta_i = row.get(f'servo_beta_{i}', 0.0)
            alpha_i = row.get(f'servo_alpha_{i}', 0.0)
            
            # Pod thrust vector in body: pointing up/tilted
            t_pod_b = np.array([math.sin(alpha_i), -math.sin(beta_i), math.cos(alpha_i) * math.cos(beta_i)])
            t_pod_w = R @ t_pod_b
            
            # Draw pod gimbal cylinder
            ax_3d.scatter([pw[0]], [pw[1]], [pw[2]], color=colors[i], s=70, edgecolor='white')
            
            # Draw thrust vector arrow
            arrow_len = 0.18 * (row.get(f'motor_{2*i}', 0.3) / 0.4)
            ax_3d.quiver(pw[0], pw[1], pw[2],
                         t_pod_w[0]*arrow_len, t_pod_w[1]*arrow_len, t_pod_w[2]*arrow_len,
                         color='#ff5555', linewidth=2.0, arrow_length_ratio=0.3)
                         
            # Draw prop disc ring (circle orthogonal to thrust vector)
            theta = np.linspace(0, 2*np.pi, 16)
            # normal vector is t_pod_w
            v_ref = np.array([1, 0, 0]) if abs(t_pod_w[0]) < 0.8 else np.array([0, 1, 0])
            u_disc = np.cross(t_pod_w, v_ref)
            u_disc /= (np.linalg.norm(u_disc) + 1e-6)
            w_disc = np.cross(t_pod_w, u_disc)
            
            disc_pts = np.array([pw + pod_r * (math.cos(th)*u_disc + math.sin(th)*w_disc) for th in theta])
            ax_3d.plot(disc_pts[:, 0], disc_pts[:, 1], disc_pts[:, 2], color=colors[i], alpha=0.7, linewidth=1.2)
        
        # Ground plane grid reference
        gx = np.linspace(-0.6, 0.6, 5)
        gy = np.linspace(-0.6, 0.6, 5)
        GX, GY = np.meshgrid(gx, gy)
        GZ = np.zeros_like(GX) - 0.5  # reference plane below
        ax_3d.plot_wireframe(GX, GY, GZ, color='#282a36', alpha=0.4, linewidth=0.8)
        
        # Formatting 3D Box
        ax_3d.set_xlim([-0.5, 0.5])
        ax_3d.set_ylim([-0.5, 0.5])
        ax_3d.set_zlim([-0.5, 0.5])
        ax_3d.set_xlabel('X [m]', color='#e0e0e0', fontsize=8)
        ax_3d.set_ylabel('Y [m]', color='#e0e0e0', fontsize=8)
        ax_3d.set_zlabel('Z [m]', color='#e0e0e0', fontsize=8)
        
        deg_roll = math.degrees(roll)
        ax_3d.set_title(f'Virgo OMAV 3D Attitude Replay\nRoll = {deg_roll:+6.1f}° | Alt = {pos_z:.2f}m',
                        fontweight='bold', color='#f8f8f2', fontsize=12)
        ax_3d.view_init(elev=20, azim=45)
        
        return pt_roll, pt_alt, pt_beta

    if output_gif is None:
        base_name = os.path.splitext(os.path.basename(csv_path))[0]
        output_gif = os.path.join(PLOT_DIR, f"{base_name}_simulation.gif")
        
    print(f"\033[1;33m[GIF Generator]\033[0m Đang render {n_frames} frames sang: {output_gif} ...")
    anim = FuncAnimation(fig, update, frames=n_frames, interval=1000//fps, blit=False)
    writer = PillowWriter(fps=fps)
    anim.save(output_gif, writer=writer)
    plt.close(fig)
    print(f"\033[1;32m[GIF Generator THÀNH CÔNG]\033[0m Đã xuất video GIF tại:\n  -> {output_gif}")
    
    # Copy to artifacts directory
    artifact_gif = f"/home/duc/.gemini/antigravity/brain/063c79dc-5a8e-480b-8076-b8daeeb23834/virgo_roll_360_simulation.gif"
    os.system(f"cp {output_gif} {artifact_gif}")
    print(f"  -> Đã copy sang artifacts: {artifact_gif}")
    return output_gif

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 generate_flight_gif.py <path_to_csv> [output_gif]")
        sys.exit(1)
    csv_file = sys.argv[1]
    out_file = sys.argv[2] if len(sys.argv) > 2 else None
    generate_gif(csv_file, out_file)
