/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ActuatorEffectivenessVirgo.cpp
 *
 * Actuator effectiveness for Virgo OMAV: H-Frame Omnidirectional Multi-Rotor UAV
 * with 4 Coaxial Counter-Rotating Rotor Pairs and 8 Independent 2-DOF Gimbal Servos.
 *
 * Mathematical derivation: Closed-form energy-optimal control allocation adapted
 * to rectangular H-Frame geometry (Lx = 0.20m, Ly = 0.20m).
 */

#include "ActuatorEffectivenessVirgo.hpp"
#include <px4_platform_common/log.h>
#include <cmath>

using namespace matrix;

ActuatorEffectivenessVirgo::ActuatorEffectivenessVirgo(ModuleParams *parent)
	: ModuleParams(parent)
{
	_param_rotor_count = param_find("CA_ROTOR_COUNT");
	_param_thr_hover   = param_find("MPC_THR_HOVER");
	_param_thr_min     = param_find("MPC_THR_MIN");
	_param_thr_max     = param_find("MPC_THR_MAX");

	updateParams();
}

ActuatorEffectivenessVirgo::~ActuatorEffectivenessVirgo()
{
	delete _rotors;
}

void ActuatorEffectivenessVirgo::updateParams()
{
	ModuleParams::updateParams();

	if (_param_rotor_count != PARAM_INVALID) {
		param_get(_param_rotor_count, &_rotor_count);
	}

	if (_param_thr_hover != PARAM_INVALID) {
		param_get(_param_thr_hover, &_hover_throttle);
	}

	if (_param_thr_min != PARAM_INVALID) {
		param_get(_param_thr_min, &_min_throttle);
	}

	if (_param_thr_max != PARAM_INVALID) {
		param_get(_param_thr_max, &_max_throttle);
	}

	if (!_rotors) {
		_rotors = new ActuatorEffectivenessRotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards);
	}

	_r = sqrtf(_lx * _lx + _ly * _ly);
}

bool ActuatorEffectivenessVirgo::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// 1. Add Rotors (8 motors: 4 co-axial pairs)
	bool rotors_added = _rotors && _rotors->addActuators(configuration);

	_first_servo_idx = configuration.num_actuators_matrix[0];

	// 2. Add 8 Gimbal Servos (2 per arm)
	// Arm 0: alpha_0 (Pitch), beta_0 (Roll)
	// Arm 1: alpha_1 (Pitch), beta_1 (Roll)
	// Arm 2: alpha_2 (Pitch), beta_2 (Roll)
	// Arm 3: alpha_3 (Pitch), beta_3 (Roll)
	for (int i = 0; i < 8; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}

	return rotors_added;
}

void ActuatorEffectivenessVirgo::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index,
		ActuatorVector &actuator_sp,
		const ActuatorVector &actuator_min,
		const ActuatorVector &actuator_max)
{
	// -------------------------------------------------------------------------
	// 1. Convert Control Setpoint from PX4 FRD to Paper FLU (Section II)
	// -------------------------------------------------------------------------
	// PX4 FRD:
	//   control_sp(0): tau_x (Roll, right = +)
	//   control_sp(1): tau_y (Pitch, nose up = +)
	//   control_sp(2): tau_z (Yaw, clockwise = +)
	//   control_sp(3): fx (Forward = +)
	//   control_sp(4): fy (Right = +)
	//   control_sp(5): fz (Down = +, hover thrust is negative!)
	//
	// Paper FLU:
	//   fx_flu = fx_frd
	//   fy_flu = -fy_frd (Left = +)
	//   fz_flu = -fz_frd (Up = +)
	//   tau_x_flu = tau_x_frd
	//   tau_y_flu = -tau_y_frd
	//   tau_z_flu = -tau_z_frd
	const float fx_flu    = control_sp(3);
	const float fy_flu    = -control_sp(4);
	const float fz_flu    = -control_sp(5);

	const float tau_x_flu = control_sp(0);
	const float tau_y_flu = -control_sp(1);
	const float tau_z_flu = -control_sp(2);

	const Vector3f fd_flu{fx_flu, fy_flu, fz_flu};
	const Vector3f kB{0.0f, 0.0f, 1.0f};

	// Tangential unit vectors for yaw torque for H-Frame:
	// Arm 0 (Front-Left:  +lx, +ly): [-ly/r,  lx/r, 0]
	// Arm 1 (Back-Left:   -lx, +ly): [-ly/r, -lx/r, 0]
	// Arm 2 (Back-Right:  -lx, -ly): [ ly/r, -lx/r, 0]
	// Arm 3 (Front-Right: +lx, -ly): [ ly/r,  lx/r, 0]
	const Vector3f t_psi[4] = {
		Vector3f{-_ly / _r,  _lx / _r, 0.0f},
		Vector3f{-_ly / _r, -_lx / _r, 0.0f},
		Vector3f{ _ly / _r, -_lx / _r, 0.0f},
		Vector3f{ _ly / _r,  _lx / _r, 0.0f}
	};

	// -------------------------------------------------------------------------
	// 2. Closed-Form Energy-Optimal Wrench Allocation (Adapted for H-Frame)
	// Scaling normalized PX4 torques [-1, 1] by nominal arm hover thrust and arm spans.
	// -------------------------------------------------------------------------
	const float arm_hover = 0.25f * _hover_throttle;
	const float tau_x_scaled = tau_x_flu * (arm_hover / _ly);
	const float tau_y_scaled = tau_y_flu * (arm_hover / _lx);
	// Soften servo yaw vectoring contribution to eliminate phase-lag limit cycle
	const float tau_z_scaled = tau_z_flu * (arm_hover / _r) * 0.35f;

	Vector3f t[4];

	// Arm 0: Front-Left (+lx, +ly)
	t[0] = fd_flu * 0.25f
	       + kB * ( tau_x_scaled * 0.25f)
	       + kB * (-tau_y_scaled * 0.25f)
	       + t_psi[0] * (tau_z_scaled * 0.25f);

	// Arm 1: Back-Left (-lx, +ly)
	t[1] = fd_flu * 0.25f
	       + kB * ( tau_x_scaled * 0.25f)
	       + kB * ( tau_y_scaled * 0.25f)
	       + t_psi[1] * (tau_z_scaled * 0.25f);

	// Arm 2: Back-Right (-lx, -ly)
	t[2] = fd_flu * 0.25f
	       + kB * (-tau_x_scaled * 0.25f)
	       + kB * ( tau_y_scaled * 0.25f)
	       + t_psi[2] * (tau_z_scaled * 0.25f);

	// Arm 3: Front-Right (+lx, -ly)
	t[3] = fd_flu * 0.25f
	       + kB * (-tau_x_scaled * 0.25f)
	       + kB * (-tau_y_scaled * 0.25f)
	       + t_psi[3] * (tau_z_scaled * 0.25f);

	// Mode gating: lock servos strictly upright (0°) during ground ops, takeoff, and landing
	vehicle_land_detected_s land_detected{};
	_vehicle_land_detected_sub.copy(&land_detected);

	vehicle_local_position_s local_pos{};
	_vehicle_local_position_sub.copy(&local_pos);

	const float alt = -local_pos.z;
	const float f_norm = fd_flu.norm();

	// Servos locked strictly upright (0°) below 0.40m or on ground -> fixed UAV configuration for takeoff/landing
	const bool lock_servos_upright = land_detected.landed || (local_pos.z_valid && alt < 0.40f);

	// Touchdown detection: vehicle is only on ground if land detector confirms landed OR vehicle has settled on deck at idle
	// During active flight descent in air, full yaw reaction torque and attitude authority remain 100% active!
	const bool is_on_ground = land_detected.landed ||
				  (land_detected.ground_contact && f_norm < 0.15f) ||
				  (local_pos.z_valid && alt < 0.05f && f_norm < 0.15f);

	// -------------------------------------------------------------------------
	// 3. Extract Rotor Speeds & Gimbal Angles (Eq. 14, 15)
	// -------------------------------------------------------------------------
	for (int i = 0; i < 4; ++i) {
		const float T_mag = t[i].norm();

		float alpha_i = 0.0f;
		float beta_i  = 0.0f;

		// Keep upright (alpha=0, beta=0) on the ground, during takeoff, or during landing (lock_servos_upright)
		if (!lock_servos_upright && T_mag > 0.03f && t[i](2) > 0.01f) {
			const Vector3f t_hat = t[i] / T_mag;

			// Roll angle beta: beta = -asin(t_hat_y)
			const float t_hat_y_clamped = math::constrain(t_hat(1), -0.999f, 0.999f);
			beta_i = -asinf(t_hat_y_clamped);

			const float cos_beta = cosf(beta_i);

			if (fabsf(cos_beta) > 1e-3f) {
				const float sin_alpha_ratio = math::constrain(t_hat(0) / cos_beta, -1.0f, 1.0f);
				alpha_i = -asinf(sin_alpha_ratio);
			}
		}

		// Gimbal Lock mitigation:
		// If |beta| > 80 deg, apply hybrid differential thrust for yaw authority
		float yaw_differential = 0.0f;
		if (fabsf(beta_i) > 1.4f) {
			yaw_differential = math::constrain(tau_z_flu / (2.0f * _r), -0.1f, 0.1f);
			if (i == 1 || i == 2) yaw_differential = -yaw_differential;
		}

		// ---------------------------------------------------------------------
		// 4. Map to Physical Actuators (FRD Gazebo Convention)
		// ---------------------------------------------------------------------
		const float joint_pitch = math::constrain(alpha_i, -M_PI_2_F, M_PI_2_F);
		const float joint_roll  = math::constrain(beta_i, -M_PI_2_F, M_PI_2_F);

		// Servos are mapped to channels 201..208:
		const int s_pitch_idx = _first_servo_idx + 2 * i;
		const int s_roll_idx  = _first_servo_idx + 2 * i + 1;

		if (s_roll_idx < NUM_ACTUATORS) {
			// Output normalized to [-1, 1] representing [-pi/2, +pi/2]
			actuator_sp(s_pitch_idx) = joint_pitch / M_PI_2_F;
			actuator_sp(s_roll_idx)  = joint_roll  / M_PI_2_F;
		}

		// Co-Axial Motor pair throttle:
		const float nominal_arm_scale = 0.25f;
		float throttle = (T_mag / nominal_arm_scale) + yaw_differential;
		throttle = math::constrain(throttle, _min_throttle, _max_throttle);

		// Fast Instantaneous Reaction Torque for Coaxial Pairs (Zero Servo Delay):
		// control_sp(2) is PX4 FRD Yaw torque (+ = Clockwise).
		// Upper motor (m0: CCW) reaction torque is Clockwise (+).
		// Lower motor (m1: CW) reaction torque is Counter-Clockwise (-).
		// Coaxial differential reaction torque: 100% active during air descent!
		// Only zeroed when truly resting on ground at idle to prevent friction twisting
		const float yaw_rotor_diff = is_on_ground ? 0.0f : math::constrain(control_sp(2) * 0.15f, -0.20f, 0.20f);

		const int m0_idx = 2 * i;     // Upper rotor (CCW)
		const int m1_idx = 2 * i + 1; // Lower rotor (CW)

		if (m1_idx < _first_servo_idx) {
			actuator_sp(m0_idx) = math::constrain(throttle + yaw_rotor_diff, _min_throttle, _max_throttle);
			actuator_sp(m1_idx) = math::constrain(throttle - yaw_rotor_diff, _min_throttle, _max_throttle);
		}
	}
}
