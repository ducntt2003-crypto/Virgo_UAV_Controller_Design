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
 * @file ActuatorEffectivenessVirgo.hpp
 *
 * Actuator effectiveness for Virgo OMAV: H-Frame Omnidirectional Multi-Rotor UAV
 * with 4 Coaxial Counter-Rotating Rotor Pairs and 8 Independent 2-DOF Gimbal Servos.
 *
 * Mathematical derivation based on closed-form energy-optimal control allocation:
 * Y. Aboudorra et al. (arXiv:2505.18270v1) adapted to H-Frame rectangular geometry (Lx != Ly).
 */

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>

class ActuatorEffectivenessVirgo : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessVirgo(ModuleParams *parent);
	virtual ~ActuatorEffectivenessVirgo();

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
			   int matrix_index,
			   ActuatorVector &actuator_sp,
			   const ActuatorVector &actuator_min,
			   const ActuatorVector &actuator_max) override;

	const char *name() const override { return "Virgo"; }

	void updateParams() override;

private:
	ActuatorEffectivenessRotors *_rotors{nullptr};
	int _first_servo_idx{0};

	param_t _param_rotor_count{PARAM_INVALID};
	param_t _param_thr_hover{PARAM_INVALID};
	param_t _param_thr_min{PARAM_INVALID};
	param_t _param_thr_max{PARAM_INVALID};

	int32_t _rotor_count{8};
	float _hover_throttle{0.35f};
	float _min_throttle{0.10f};
	float _max_throttle{0.90f};

	// Square Symmetric Frame Dimensions (CAD USDF V2 / Virgo):
	// Lx: Longitudinal half-span (X axis) = 0.20m
	// Ly: Transverse half-span (Y axis)   = 0.20m
	const float _lx{0.20f};
	const float _ly{0.20f};
	float _r{0.2828427f}; // sqrt(0.20^2 + 0.20^2) = 0.20 * sqrt(2)

	// Mode gating: lock servos upright during takeoff, landing, and ground operations
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	// Servo phase-lead compensation to cancel mechanical actuator delay
	hrt_abstime _last_alloc_time{0};
	float _alpha_prev[4]{0.0f, 0.0f, 0.0f, 0.0f};
	float _beta_prev[4]{0.0f, 0.0f, 0.0f, 0.0f};
	float _alpha_smoothed[4]{0.0f, 0.0f, 0.0f, 0.0f};
	float _beta_smoothed[4]{0.0f, 0.0f, 0.0f, 0.0f};
	float _d_alpha_filtered[4]{0.0f, 0.0f, 0.0f, 0.0f};
	float _d_beta_filtered[4]{0.0f, 0.0f, 0.0f, 0.0f};
};
