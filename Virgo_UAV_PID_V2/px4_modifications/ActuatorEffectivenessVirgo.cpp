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

	// Mode gating: lock servos strictly upright (0°) during ground ops, takeoff, and landing
	vehicle_land_detected_s land_detected{};
	_vehicle_land_detected_sub.copy(&land_detected);

	vehicle_local_position_s local_pos{};
	_vehicle_local_position_sub.copy(&local_pos);

	const float alt = -local_pos.z;
	const float f_norm = fd_flu.norm();

	// Servos locked strictly upright (0°) below 0.08m or on ground -> fixed UAV configuration for touchdown
	const bool lock_servos_upright = land_detected.landed || (local_pos.z_valid && alt < 0.08f);

	// Touchdown detection: vehicle is only on ground if land detector confirms landed OR vehicle has settled on deck at idle
	// During active flight descent in air, full yaw reaction torque and attitude authority remain 100% active!
	const bool is_on_ground = land_detected.landed ||
				  (land_detected.ground_contact && f_norm < 0.15f) ||
				  (local_pos.z_valid && alt < 0.05f && f_norm < 0.15f);

	// -------------------------------------------------------------------------
	// 2. Hybrid Dual-Layer Frequency-Divided Yaw Control with Parasitic Decoupling
	// -------------------------------------------------------------------------
	// Layer 1 (Servos): Smooth Low-Pass Filtered (~0.8 Hz) Tangential Vectoring
	//   Completely isolates mechanical servos from fast rate loop feedback,
	//   eliminating phase-lag limit cycle hunting while delivering up to 1.5 N*m
	//   of steady-state yaw authority to reject aerodynamic drag moments.
	if (lock_servos_upright) {
		_tau_z_servo_lp = 0.0f;
	} else {
		const float alpha_yaw_lp = 0.025f; // ~0.8 Hz low-pass cutoff at 250 Hz loop
		_tau_z_servo_lp += alpha_yaw_lp * (tau_z_flu - _tau_z_servo_lp);
	}
	const float arm_hover = 0.25f * _hover_throttle;
	const float tau_z_scaled = math::constrain(_tau_z_servo_lp * (arm_hover / _r) * 0.40f, -0.07f, 0.07f);

	// Layer 2 (Rotors): Instantaneous Differential Rotor Reaction Torque
	//   Provides zero-delay rate damping directly through BLDC coaxial motor pairs.
	// Cross-Axis Parasitic Decoupling:
	//   When pods are tilted by (alpha, beta), differential rotor reaction torque
	//   projects onto vehicle body Roll and Pitch axes. We calculate this projection
	//   from the current pod angles and subtract it from the Roll/Pitch allocation channels.
	float sum_proj_x = 0.0f;
	float sum_proj_y = 0.0f;
	float sum_proj_z = 0.0f;
	for (int i = 0; i < 4; ++i) {
		const float cos_b = cosf(_beta_smoothed[i]);
		const float sin_b = sinf(_beta_smoothed[i]);
		const float sin_a = sinf(_alpha_smoothed[i]);
		const float cos_a = cosf(_alpha_smoothed[i]);
		// Unit vector of pod axis in FLU:
		// x_flu = -sin(alpha)*cos(beta), y_flu = -sin(beta), z_flu = cos(alpha)*cos(beta)
		sum_proj_x += -sin_a * cos_b;
		sum_proj_y += -sin_b;
		sum_proj_z += cos_a * cos_b;
	}
	const float avg_proj_x = 0.25f * sum_proj_x;
	const float avg_proj_y = 0.25f * sum_proj_y;
	const float avg_proj_z = math::max(0.25f * sum_proj_z, 0.60f);

	// Boost rotor yaw differential by 1/avg_proj_z to maintain 100% yaw authority under tilt
	const float yaw_rotor_diff = is_on_ground ? 0.0f : math::constrain((control_sp(2) / avg_proj_z) * 0.30f, -0.30f, 0.30f);

	// Feedforward anti-parasitic cancellation:
	// Differential reaction torque in FLU generates:
	// tau_x_parasitic = yaw_rotor_diff * avg_proj_x
	// tau_y_parasitic = yaw_rotor_diff * avg_proj_y
	// Subtracting these keeps Roll and Pitch strictly decoupled from Yaw actions!
	const float tau_x_decoupled = tau_x_flu - yaw_rotor_diff * avg_proj_x;
	const float tau_y_decoupled = tau_y_flu - yaw_rotor_diff * avg_proj_y;

	const float tau_x_scaled = tau_x_decoupled * (arm_hover / _ly);
	const float tau_y_scaled = tau_y_decoupled * (arm_hover / _lx);

	// Vectoring force vector for gimbal pods (purely translational + filtered yaw trim):
	// NOTE: Decoupling Roll/Pitch attitude torque from servo angle extraction is crucial!
	// Servos ONLY orient the pods to deliver translational force and slow yaw trim.
	// Fast attitude stabilization (tau_x, tau_y) is handled 100% by differential motor thrust.
	// This isolates servos from 250 Hz attitude rate feedback chatter, eliminating jitter completely.
	Vector3f t_vec[4];
	for (int i = 0; i < 4; ++i) {
		t_vec[i] = fd_flu * 0.25f + t_psi[i] * (tau_z_scaled * 0.25f);
	}

	// -------------------------------------------------------------------------
	// 3. Extract Rotor Speeds & Gimbal Angles (Eq. 14, 15)
	// -------------------------------------------------------------------------
	for (int i = 0; i < 4; ++i) {
		const float T_vec_mag = t_vec[i].norm();

		float alpha_i = 0.0f;
		float beta_i  = 0.0f;

		// Keep upright (alpha=0, beta=0) on the ground, during takeoff, or during landing (lock_servos_upright)
		if (!lock_servos_upright && T_vec_mag > 0.03f && t_vec[i](2) > 0.01f) {
			const Vector3f t_hat = t_vec[i] / T_vec_mag;

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
		// 4. Map to Physical Actuators (FRD Gazebo Convention) with Phase-Lead
		// ---------------------------------------------------------------------
		const hrt_abstime now_us = hrt_absolute_time();
		float dt_alloc = (_last_alloc_time > 0) ? (float)(now_us - _last_alloc_time) * 1e-6f : 0.002f;
		dt_alloc = math::constrain(dt_alloc, 0.001f, 0.02f);
		if (i == 3) {
			_last_alloc_time = now_us;
		}

		if (lock_servos_upright) {
			alpha_i = 0.0f;
			beta_i  = 0.0f;
			_alpha_prev[i] = 0.0f;
			_beta_prev[i]  = 0.0f;
			_alpha_smoothed[i] = 0.0f;
			_beta_smoothed[i]  = 0.0f;
			_d_alpha_filtered[i] = 0.0f;
			_d_beta_filtered[i]  = 0.0f;
		} else {
			// Low-pass filters aligned with MorphQuad DYNAMIXEL actuators (arXiv:2607.02764):
			// Outer axis (XC330-M288-T via belt-pulley, rise time 119 ms): cutoff ~8.0 Hz (filter_alpha = 0.18f)
			// Inner axis (XL330-M288-T inline, rise time 182 ms): cutoff ~5.2 Hz (filter_beta = 0.12f)
			const float filter_alpha = 0.18f;
			const float filter_beta  = 0.12f;
			const float alpha_target = _alpha_smoothed[i] + filter_alpha * (alpha_i - _alpha_smoothed[i]);
			const float beta_target  = _beta_smoothed[i]  + filter_beta  * (beta_i  - _beta_smoothed[i]);

			// Physical Slew Rate Limiters matching DYNAMIXEL hardware:
			// Outer axis (XC330 at 11.1V): 100 RPM ~ 10.5 rad/s (600 deg/s)
			// Inner axis (XL330 at 5.0V): 61 RPM ~ 6.4 rad/s (366 deg/s)
			const float max_servo_rate_alpha = 10.0f; // rad/s
			const float max_servo_rate_beta  = 6.4f;  // rad/s

			const float max_delta_alpha = max_servo_rate_alpha * dt_alloc;
			const float max_delta_beta  = max_servo_rate_beta  * dt_alloc;

			const float delta_alpha = math::constrain(alpha_target - _alpha_smoothed[i], -max_delta_alpha, max_delta_alpha);
			const float delta_beta  = math::constrain(beta_target  - _beta_smoothed[i],  -max_delta_beta,  max_delta_beta);

			const float d_alpha = delta_alpha / dt_alloc;
			const float d_beta  = delta_beta  / dt_alloc;
			_alpha_smoothed[i] += delta_alpha;
			_beta_smoothed[i]  += delta_beta;

			_d_alpha_filtered[i] = 0.85f * _d_alpha_filtered[i] + 0.15f * d_alpha;
			_d_beta_filtered[i]  = 0.85f * _d_beta_filtered[i]  + 0.15f * d_beta;
		}

		// Phase-lead compensation tuned to cancel mechanical actuator lag:
		// 5 ms for outer axis (XC330), 8 ms for inner axis (XL330)
		const float tau_lead_alpha = 0.005f;
		const float tau_lead_beta  = 0.008f;
		const float joint_pitch = lock_servos_upright ? 0.0f : math::constrain(_alpha_smoothed[i] + tau_lead_alpha * _d_alpha_filtered[i], -M_PI_2_F, M_PI_2_F);
		const float joint_roll  = lock_servos_upright ? 0.0f : math::constrain(_beta_smoothed[i]  + tau_lead_beta  * _d_beta_filtered[i],  -M_PI_2_F, M_PI_2_F);

		// Servos are mapped to channels 201..208:
		const int s_pitch_idx = _first_servo_idx + 2 * i;
		const int s_roll_idx  = _first_servo_idx + 2 * i + 1;

		if (s_roll_idx < NUM_ACTUATORS) {
			// Output normalized to [-1, 1] representing [-pi/2, +pi/2]
			actuator_sp(s_pitch_idx) = joint_pitch / M_PI_2_F;
			actuator_sp(s_roll_idx)  = joint_roll  / M_PI_2_F;
		}

		// Instantaneous Differential Thrust for Roll and Pitch Attitude Stabilization:
		// BLDC motors deliver zero-delay roll/pitch reaction without servo movement.
		float delta_thrust = 0.0f;
		if (i == 0) delta_thrust =  (tau_x_scaled * 0.25f) - (tau_y_scaled * 0.25f);
		if (i == 1) delta_thrust =  (tau_x_scaled * 0.25f) + (tau_y_scaled * 0.25f);
		if (i == 2) delta_thrust = -(tau_x_scaled * 0.25f) + (tau_y_scaled * 0.25f);
		if (i == 3) delta_thrust = -(tau_x_scaled * 0.25f) - (tau_y_scaled * 0.25f);

		const float T_mag = math::max(0.01f, T_vec_mag + delta_thrust);

		// Co-Axial Motor pair throttle:
		const float nominal_arm_scale = 0.25f;
		float throttle = (T_mag / nominal_arm_scale) + yaw_differential;
		throttle = math::constrain(throttle, _min_throttle, _max_throttle);

		// Fast Instantaneous Reaction Torque for Coaxial Pairs (Zero Servo Delay):
		// Upper motor (m0: CCW) reaction torque is Clockwise (+).
		// Lower motor (m1: CW) reaction torque is Counter-Clockwise (-).
		// Coaxial differential reaction torque provides rate damping with zero phase lag.

		const int m0_idx = 2 * i;     // Upper rotor (CCW)
		const int m1_idx = 2 * i + 1; // Lower rotor (CW)

		if (m1_idx < _first_servo_idx) {
			actuator_sp(m0_idx) = math::constrain(throttle + yaw_rotor_diff, _min_throttle, _max_throttle);
			actuator_sp(m1_idx) = math::constrain(throttle - yaw_rotor_diff, _min_throttle, _max_throttle);
		}
	}
}
