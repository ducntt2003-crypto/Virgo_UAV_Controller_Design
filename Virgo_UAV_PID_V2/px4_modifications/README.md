# PX4 Firmware Modifications for Virgo OMAV

This folder contains the firmware modifications for PX4 Autopilot v1.15 required to run the **Virgo_UAV_PID** fully actuated omnidirectional vehicle.

## Files
1. `ActuatorEffectivenessVirgo.cpp` & `ActuatorEffectivenessVirgo.hpp`:
   - Copy to: `PX4-Autopilot/src/modules/control_allocator/VehicleActuatorEffectiveness/`
   - Features: Closed-form analytical 6-DOF force/torque allocation, decoupled coaxial yaw reaction torque, and gated ground-lock for safe takeoff/landing.

2. `MulticopterPositionControl.cpp`:
   - Copy to: `PX4-Autopilot/src/modules/mc_pos_control/`
   - Features: Actual-heading (`states.yaw`) world-to-body force projection decoupling. Eliminates lateral position drift during yaw rotations.

3. `px4_virgo_omav.patch`:
   - Unified patch against PX4 v1.15. Can be applied via:
     ```bash
     cd /path/to/PX4-Autopilot
     git apply /path/to/Virgo_UAV_PID/px4_modifications/px4_virgo_omav.patch
     ```
