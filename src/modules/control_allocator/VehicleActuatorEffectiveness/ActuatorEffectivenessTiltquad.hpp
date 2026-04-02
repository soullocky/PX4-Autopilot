/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessTiltquad.hpp
 *
 * Actuator effectiveness for tiltrotor VTOL
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessTilts.hpp"
#include <uORB/topics/tiltrotor_extra_controls.h>
#include <uORB/Subscription.hpp>

class ActuatorEffectivenessTiltquad : public ModuleParams, public ActuatorEffectiveness
{
public:


/**
 * @enum FlightMode
 * Flight mode selection for tiltquad
 * Mode1: Fixed position, attitude control - change attitude to achieve non-zero attitude hover
 * Mode2: Fixed attitude, position control - change position with fixed or limited tilt
 */
enum class FlightMode : int32_t {
MODE1_FIXED_POSITION_ATTITUDE_CHANGE = 0,  // Fixed position, attitude control via tilts
MODE2_FIXED_ATTITUDE_POSITION_CHANGE = 1,  // Fixed attitude, position control via motors
};

	ActuatorEffectivenessTiltquad(ModuleParams *parent);
	virtual ~ActuatorEffectivenessTiltquad() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
				    int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
				    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

	const char *name() const override { return "Tiltquad"; }

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

protected:

// Flight mode control and state tracking
FlightMode _current_flight_mode{FlightMode::MODE1_FIXED_POSITION_ATTITUDE_CHANGE};
float _target_attitude_pitch{0.0f};  // Target pitch angle for mode1 (±90°)
float _target_attitude_roll{0.0f};   // Target roll angle for mode1 (±90°)
float _max_tilt_angle{90.0f};        // Max tilt angle constraint for both modes (degrees, ±90°)

	// uORB subscription for extra controls
	uORB::Subscription _tiltrotor_extra_controls_sub{ORB_ID(tiltrotor_extra_controls)};

// Parameter handles
param_t _param_flight_mode_handle;
param_t _param_max_tilt_angle_handle;


        // Flight mode-specific control methods
        void applyMode1Control(ActuatorVector &actuator_sp,
                const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
                const matrix::Vector<float, NUM_ACTUATORS> &actuator_max,
                const matrix::Vector<float, NUM_AXES> &control_sp);
        void applyMode2Control(ActuatorVector &actuator_sp,
                const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
                const matrix::Vector<float, NUM_ACTUATORS> &actuator_max);
	ActuatorVector _tilt_offsets;
	ActuatorEffectivenessRotors _mc_rotors;
	ActuatorEffectivenessTilts _tilts;
	int _first_tilt_idx{0};

	struct YawTiltSaturationFlags {
		bool tilt_yaw_pos;
		bool tilt_yaw_neg;
	};

	YawTiltSaturationFlags _yaw_tilt_saturation_flags{};
};
