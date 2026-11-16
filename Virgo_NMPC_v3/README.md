# Virgo OMAV SE(3) Nonlinear Model Predictive Control (NMPC)

[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD--3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Python: 3.10+](https://img.shields.io/badge/Python-3.10%2B-brightgreen.svg)](https://www.python.org/)
[![CasADi: 3.8+](https://img.shields.io/badge/CasADi-3.8%2B-orange.svg)](https://web.casadi.org/)
[![PX4: v1.15](https://img.shields.io/badge/PX4-v1.15-red.svg)](https://px4.io/)
[![Control: NMPC on SE(3)](https://img.shields.io/badge/Control-NMPC%20on%20SE(3)-purple.svg)](#)

---

## 1. Overview & Research Motivation

The **Virgo OMAV (MorphQuad 4.56 kg)** is an overactuated, fully decoupled 6-DOF aerial vehicle equipped with **4 coaxial tilting pods (8 BLDC motors and 8 gimbal servos)**.

While standard cascaded PID controllers with static pseudo-inverse allocation can achieve steady hover, they suffer from fundamental physical limitations during aggressive or decoupled maneuvers:
1. **Actuator Time-Scale Separation**: BLDC motors respond within $\sim 10\text{ ms}$, whereas mechanical gimbal servos have a finite slew rate ($\sim 150\text{ ms}$ delay, $`\dot{\alpha}_{\max} \approx 10\text{ rad/s}`$). Reactive feedback controllers cannot predict trajectory curvatures, leading to phase lag and lateral cross-track drift.
2. **Coupled Actuator Saturation**: Static allocation cannot enforce joint acceleration and rate limits simultaneously with non-convex trigonometric thrust vectoring bounds.
3. **Parasitic Force Antagonism**: Without explicit posture regularization, pods can tilt in opposing directions, creating internal stresses and wasting energy.

**Virgo_NMPC_v3** resolves these challenges by formulating a full-state **Nonlinear Model Predictive Controller (NMPC) on Lie group** $\mathrm{SE}(3)$, explicitly optimizing over a look-ahead prediction horizon with servo rate constraints.

<p align="center">
  <img src="docs/nmpc_vs_pid_comparison.png" width="92%" alt="NMPC vs PID Benchmark Comparison"/>
  <br/>
  <em>Figure 1: Telemetry comparison between SE(3) NMPC and Baseline Cascaded PID on the 25m Decoupled Square Trajectory.</em>
</p>

---

## 2. Mathematical Formulation on $\mathrm{SE}(3)$

### 2.1. State and Control Vectors
To strictly respect mechanical servo rate limits, the servo tilt angles are modeled as system dynamic states integrated from slew rate commands:

```math
\mathbf{x} = \begin{bmatrix} \mathbf{p} \\ \mathbf{v} \\ \mathbf{q} \\ \boldsymbol{\omega} \\ \boldsymbol{\alpha} \\ \boldsymbol{\beta} \end{bmatrix} \in \mathbb{R}^{21}, \quad
\mathbf{u} = \begin{bmatrix} \mathbf{T} \\ \dot{\boldsymbol{\alpha}} \\ \dot{\boldsymbol{\beta}} \\ \tau_{z,\mathrm{rotor}} \end{bmatrix} \in \mathbb{R}^{13}
```

Where:
- $`\mathbf{p} = [x, y, z]^T \in \mathbb{R}^3`$: Vehicle position in World NED frame.
- $`\mathbf{v} = [v_x, v_y, v_z]^T \in \mathbb{R}^3`$: Linear velocity in World NED frame.
- $`\mathbf{q} = [q_w, q_x, q_y, q_z]^T \in \mathbb{S}^3`$: Unit quaternion mapping Body frame $\mathcal{B}$ to World frame $\mathcal{W}$.
- $`\boldsymbol{\omega} = [p, q, r]^T \in \mathbb{R}^3`$: Body angular velocity vector.
- $`\boldsymbol{\alpha} = [\alpha_1, \alpha_2, \alpha_3, \alpha_4]^T`$: Outer pitch gimbal angles of 4 pods.
- $`\boldsymbol{\beta} = [\beta_1, \beta_2, \beta_3, \beta_4]^T`$: Inner roll gimbal angles of 4 pods.
- $`\mathbf{T} = [T_1, T_2, T_3, T_4]^T`$: Individual pod thrust magnitudes in Newtons.
- $`\dot{\boldsymbol{\alpha}}, \dot{\boldsymbol{\beta}} \in \mathbb{R}^4`$: Commanded servo slew rates in rad/s.
- $`\tau_{z,\mathrm{rotor}} \in \mathbb{R}`$: Pure coaxial differential rotor reaction torque.

### 2.2. Continuous-Time Equations of Motion
The non-linear rigid-body dynamics are governed by:

```math
\begin{aligned}
\dot{\mathbf{p}} &= \mathbf{v} \\
m \dot{\mathbf{v}} &= m \mathbf{g} + \mathbf{R}(\mathbf{q}) \mathbf{f}_{\mathcal{B}} \\
\dot{\mathbf{q}} &= \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix} \\
\mathbf{J}_{\mathcal{B}} \dot{\boldsymbol{\omega}} &= -\boldsymbol{\omega} \times (\mathbf{J}_{\mathcal{B}} \boldsymbol{\omega}) + \boldsymbol{\tau}_{\mathcal{B}} \\
\dot{\boldsymbol{\alpha}} &= \mathbf{u}_{\dot{\alpha}} \\
\dot{\boldsymbol{\beta}} &= \mathbf{u}_{\dot{\beta}}
\end{aligned}
```

The net resultant body force and torque produced by the 4 tilting pods are:

```math
\mathbf{f}_{\mathcal{B}} = \sum_{i=1}^4 T_i \hat{\mathbf{t}}_i(\alpha_i, \beta_i), \quad
\boldsymbol{\tau}_{\mathcal{B}} = \sum_{i=1}^4 \left( \mathbf{l}_i \times \left( T_i \hat{\mathbf{t}}_i(\alpha_i, \beta_i) \right) \right) + \begin{bmatrix} 0 \\ 0 \\ \tau_{z,\mathrm{rotor}} \end{bmatrix}
```

In North-East-Down (NED) body coordinates, the unit thrust vector of pod $i$ is:

```math
\hat{\mathbf{t}}_i(\alpha_i, \beta_i) = \begin{bmatrix} -\sin\alpha_i \cos\beta_i \\ -\sin\beta_i \\ -\cos\alpha_i \cos\beta_i \end{bmatrix}
```

At neutral posture ($`\alpha_i = 0, \beta_i = 0`$), $`\hat{\mathbf{t}}_i = [0, 0, -1]^T`$ produces purely upward lift, canceling gravitational acceleration $g = [0, 0, +9.80665]^T\text{ m/s}^2$.

---

## 3. Optimal Control Problem (OCP) Formulation

The NMPC solves a **Direct Multiple Shooting NLP** over horizon $N = 20$ with sampling time $dt = 0.04\text{s}$ ($`T_f = 0.8\text{s}`$, 25 Hz control loop):

```math
\min_{\mathbf{X}, \mathbf{U}} \sum_{k=0}^{N-1} \left( \|\mathbf{x}_k - \mathbf{x}_{\mathrm{ref},k}\|_{\mathbf{Q}}^2 + \|\mathbf{u}_k - \mathbf{u}_{\mathrm{ref},k}\|_{\mathbf{R}}^2 \right) + \|\mathbf{x}_N - \mathbf{x}_{\mathrm{ref},N}\|_{\mathbf{Q}_N}^2
```

Subject to:
- Initial state constraint: $`\mathbf{x}_0 = \mathbf{x}_{\mathrm{measured}}`$
- Continuity constraints: $`\mathbf{x}_{k+1} = \mathbf{f}_{\mathrm{RK4}}(\mathbf{x}_k, \mathbf{u}_k)`$
- Pod thrust bounds: $`0.5\text{ N} \le T_i \le 18.5\text{ N}`$
- Gimbal angle bounds: $`-\pi/2 \le \alpha_i, \beta_i \le +\pi/2\text{ rad}`$
- Slew rate limits: $`-10.0\text{ rad/s} \le \dot{\alpha}_i, \dot{\beta}_i \le +10.0\text{ rad/s}`$
- Reaction yaw torque bounds: $`-0.5\text{ N}\cdot\text{m} \le \tau_{z,\mathrm{rotor}} \le +0.5\text{ N}\cdot\text{m}`$

The orientation error is computed on the Lie algebra $\mathfrak{so}(3)$ using quaternion conjugate multiplication:

```math
\mathbf{e}_{\mathrm{att}} = \left[ \mathbf{q}_{\mathrm{ref}}^{-1} \otimes \mathbf{q} \right]_{\mathrm{vec}}
```

---

## 4. Benchmark Performance & Comparative Results

We validated the NMPC controller on the **25m × 25m Decoupled Square Trajectory** with corner stationary attitude switching:
- **Takeoff**: Vertical ascent to $Z = 1.5\text{ m}$.
- **At Origin (0, 0)**: Roll tilted to $+30^\circ$ while holding position.
- **Edge 1**: $25\text{ m}$ translation along $+X$ holding Roll strictly at $+30^\circ$.
- **At (+25, 0)**: Sequential attitude transition Roll $+30^\circ \to 0^\circ \to$ Pitch $+30^\circ$.
- **Edge 2**: $25\text{ m}$ translation along $+Y$ holding Pitch strictly at $+30^\circ$.
- **At (+25, +25)**: Sequential attitude transition Pitch $+30^\circ \to 0^\circ \to$ Yaw $+30^\circ$.
- **Edge 3**: $25\text{ m}$ translation along $-X$ holding Yaw strictly at $+30^\circ$.
- **At (0, +25)**: Attitude transition Yaw $+30^\circ \to 0^\circ$ (Flat neutral).
- **Edge 4**: $25\text{ m}$ translation along $-Y$ returning to origin $(0, 0)$ in Flat $0^\circ$ posture.
- **Landing**: Hover settle for 1.5s, 6-DOF descent with $0^\circ$ attitude until touchdown, and disarm.

### 4.1. Real-Time Hardware-in-the-Loop & Gazebo Harmonic Simulation Results (Run ID: 1794796926)

| Flight Metric | Baseline Cascaded PID | SE(3) NMPC (Gazebo Harmonic) | Quantitative Improvement |
| :--- | :---: | :---: | :---: |
| **Horizontal Position RMSE** | $0.482\text{ m}$ | **0.322 m** | **-33.2% Error Reduction** |
| **Maximum Lateral Drift** | $1.120\text{ m}$ | **0.496 m** | **-55.7% Drift Suppression** |
| **Mean Roll Tracking Error** | $3.85^\circ$ | **1.55°** | **-59.7% Error Reduction** |
| **Mean Pitch Tracking Error** | $2.42^\circ$ | **1.11°** | **-54.1% Error Reduction** |
| **Mean Yaw Tracking Error** | $7.60^\circ$ | **2.93°** | **-61.4% Hunting Damping** |
| **Max Landing Roll Angle** | $20.26^\circ$ *(Severe Snap)* | **0.81°** *(Level Deck Touchdown)* | **Snap Completely Eliminated** |
| **Max Landing Pitch Angle** | $14.58^\circ$ | **0.33°** *(Level Deck Touchdown)* | **Snap Completely Eliminated** |
| **Mean NMPC Solver Latency** | N/A (Reactive) | **18.97 ms** | **Real-Time (< 40 ms Deadline)** |

<p align="center">
  <img src="docs/gazebo_nmpc_flight_1794796926_analysis.png" width="95%" alt="Gazebo Harmonic 6-Panel Flight Telemetry"/>
  <br/>
  <em>Figure 2: 6-Panel telemetry of the 25m decoupled square mission in Gazebo Harmonic with PX4 Autopilot SITL (Run ID: 1794796926).</em>
</p>

### 4.2. Stability Innovations & Oscillation Mitigation
1. **Landing Mode-Switching Snap Elimination**: Reduced `near_ground` threshold in `MulticopterPositionControl.cpp` from $0.40\text{ m}$ to $0.08\text{ m}$. This prevents PX4 from abruptly disengaging 6-DOF allocation and falling back to 4-DOF quadrotor mode mid-air, guaranteeing level touchdown.
2. **Sequential Decoupling Transitions**: Separating corner attitude maneuvers eliminates cross-axis gyroscopic reaction torque $\boldsymbol{\omega} \times (\mathbf{J}_B \boldsymbol{\omega})$.
3. **Actuator Slew Smoothing & Phase-Lead**: Integrated a 1st-order low-pass exponential filter ($\sim 15\text{ Hz}$) and $15\text{ ms}$ phase-lead compensator in `ActuatorEffectivenessVirgo.cpp`, removing servo jitter and compensating mechanical gear-train lag.
4. **Septic Minimum-Jerk Trajectory**: 7th-order polynomial with zero initial and terminal acceleration/jerk.

---

## 5. Repository Structure

```text
Virgo_NMPC_v3/
├── README.md                          # Comprehensive academic documentation & formulation
├── setup_environment.sh               # One-click environment & ACADOS installer
├── config/
│   ├── virgo_params.yaml              # Mass (4.56kg), Inertia, Geometry, Limits
│   └── nmpc_config.yaml               # Horizon N, dt, weights Q, R, solver options
├── models/
│   ├── se3_kinematics.py              # SO(3)/SE(3) Lie group & quaternion library
│   └── virgo_casadi_model.py          # CasADi symbolic dynamics and RK4 integrator
├── solver/
│   ├── casadi_nmpc_solver.py          # Real-time Multiple Shooting IPOPT solver (11 ms)
│   └── generate_acados_solver.py      # ACADOS C-code generator (HPIPM / RTI SQP)
├── controllers/
│   ├── trajectory_generator.py        # 6-DOF decoupled 25m square trajectory generator
│   └── virgo_nmpc_controller.py       # High-level NMPC controller manager
├── simulation/
│   ├── run_gazebo_nmpc_test.sh        # Automated Gazebo Harmonic test launcher
│   ├── run_nmpc_standalone_sim.py     # Closed-loop numerical flight simulation
│   └── benchmark_nmpc_square.py       # NMPC vs PID benchmark analysis script
├── ros2_px4_bridge/
│   ├── px4_mavlink_nmpc_bridge.py     # Direct MAVLink bridge for PX4 SITL & Gazebo
│   └── virgo_nmpc_node.py             # ROS 2 Humble node (Micro-XRCE-DDS)
└── docs/
    ├── gazebo_nmpc_flight_1794796926.csv.gz       # Flight telemetry log (Gazebo Harmonic)
    ├── gazebo_nmpc_flight_1794796926_analysis.png # 6-panel flight telemetry analysis plot
    ├── nmpc_square_decoupled_telemetry.csv.gz     # Standalone simulation telemetry
    ├── nmpc_square_decoupled_analysis.png         # Standalone simulation plot
    └── nmpc_vs_pid_comparison.png                 # NMPC vs PID comparison plot
```

---

## 6. Setup & Quickstart Reproduction Guide

### Step 1: Install Dependencies
Run the automated environment setup script:
```bash
cd Virgo_NMPC_v3
chmod +x setup_environment.sh
./setup_environment.sh
```

### Step 2: Automated Gazebo Harmonic Flight Benchmark (Recommended)
To launch the full PX4 SITL + Gazebo Harmonic simulation and run the 25m decoupled mission with the 3D graphical window:
```bash
cd Virgo_NMPC_v3
./simulation/run_gazebo_nmpc_test.sh --gui
```
*(For headless CI/CD execution without 3D GUI, simply run `./simulation/run_gazebo_nmpc_test.sh`)*

### Step 3: Run Closed-Loop Standalone Simulation
To simulate the 25m square decoupled mission with full 6-DOF non-linear RK4 physics:
```bash
python3 simulation/run_nmpc_standalone_sim.py
```
This will generate `docs/nmpc_square_decoupled_telemetry.csv.gz` and `docs/nmpc_square_decoupled_analysis.png`.

### Step 4: Run Benchmark Comparison Against PID Baseline
To compare NMPC telemetry side-by-side with the baseline PID flight test:
```bash
python3 simulation/benchmark_nmpc_square.py
```
