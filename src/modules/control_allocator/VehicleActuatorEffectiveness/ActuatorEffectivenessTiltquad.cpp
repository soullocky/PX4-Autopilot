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
 * @file ActuatorEffectivenessTiltquad.cpp
 *
 * Actuator effectiveness for tiltrotor VTOL
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessTiltquad.hpp"

using namespace matrix;

// 初始化三个子模块
ActuatorEffectivenessTiltquad::ActuatorEffectivenessTiltquad(ModuleParams *parent)
	: ModuleParams(parent),
          _param_flight_mode_handle(param_find("CA_TILTQUAD_MODE")),
          _param_max_tilt_angle_handle(param_find("CA_TILTQUAD_MAX_TILT")),
	  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, true),
	  _tilts(this)
{
}

// 构建控制分配矩阵
bool
ActuatorEffectivenessTiltquad::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// MC motors
	_mc_rotors.enableYawByDifferentialThrust(!_tilts.hasYawControl());
	const bool rotors_added_successfully = _mc_rotors.addActuators(configuration);

	// Tilts
	_first_tilt_idx = configuration.num_actuators_matrix[0];
	_tilts.updateTorqueSign(_mc_rotors.geometry());
	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	// Set offset such that tilts point upwards when control input == 0 (trim is 0 if min_angle == -max_angle).
	// Note that we don't set configuration.trim here, because in the case of trim == +-1, yaw is always saturated
	// and reduced to 0 with the sequential desaturation method. Instead we add it after.
	_tilt_offsets.setZero();

	for (int i = 0; i < _tilts.count(); ++i) {
		float delta_angle = _tilts.config(i).max_angle - _tilts.config(i).min_angle;

		if (delta_angle > FLT_EPSILON) {
			float trim = -1.f - 2.f * _tilts.config(i).min_angle / delta_angle;
			_tilt_offsets(_first_tilt_idx + i) = trim;
		}
	}

	return (rotors_added_successfully && tilts_added_successfully);
}

// 在控制分配之后，对执行器输出做“二次处理”
void ActuatorEffectivenessTiltquad::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
				    int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
				    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	actuator_sp += _tilt_offsets;
	// TODO: dynamic matrix update


	// Route to flight mode-specific control logic
	if (_current_flight_mode == FlightMode::MODE1_FIXED_POSITION_ATTITUDE_CHANGE) {
		applyMode1Control(actuator_sp, actuator_min, actuator_max, control_sp);
	} else if (_current_flight_mode == FlightMode::MODE2_FIXED_ATTITUDE_POSITION_CHANGE) {
		applyMode2Control(actuator_sp, actuator_min, actuator_max);
	}
	bool yaw_saturated_positive = true;
	bool yaw_saturated_negative = true;

	for (int i = 0; i < _tilts.count(); ++i) {

		// custom yaw saturation logic: only declare yaw saturated if all tilts are at the negative or positive yawing limit
		if (_tilts.getYawTorqueOfTilt(i) > FLT_EPSILON) {

			if (yaw_saturated_positive && actuator_sp(i + _first_tilt_idx) < actuator_max(i + _first_tilt_idx) - FLT_EPSILON) {
				yaw_saturated_positive = false;
			}

			if (yaw_saturated_negative && actuator_sp(i + _first_tilt_idx) > actuator_min(i + _first_tilt_idx) + FLT_EPSILON) {
				yaw_saturated_negative = false;
			}

		} else if (_tilts.getYawTorqueOfTilt(i) < -FLT_EPSILON) {
			if (yaw_saturated_negative && actuator_sp(i + _first_tilt_idx) < actuator_max(i + _first_tilt_idx) - FLT_EPSILON) {
				yaw_saturated_negative = false;
			}

			if (yaw_saturated_positive && actuator_sp(i + _first_tilt_idx) > actuator_min(i + _first_tilt_idx) + FLT_EPSILON) {
				yaw_saturated_positive = false;
			}
		}
	}

	_yaw_tilt_saturation_flags.tilt_yaw_neg = yaw_saturated_negative;
	_yaw_tilt_saturation_flags.tilt_yaw_pos = yaw_saturated_positive;
}

// 向控制器反馈 Yaw 是否已经饱和（用于rate controller anti-windup）

// Mode 1: Fixed Position, Attitude Change via Tilt
// RC roll/pitch directly control the corresponding tilt angles (±90°)
void ActuatorEffectivenessTiltquad::applyMode1Control(
	ActuatorVector &actuator_sp,
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_max,
	const matrix::Vector<float, NUM_AXES> &control_sp)
{
	// Map RC roll/pitch commands [-1, 1] directly to tilt angles [-90°, +90°]
	// This enables non-zero attitude hover with direct manual control
	_target_attitude_roll = control_sp(ControlAxis::ROLL) * _max_tilt_angle;
	_target_attitude_pitch = control_sp(ControlAxis::PITCH) * _max_tilt_angle;

	// Constrain to ±max_tilt_angle
	_target_attitude_roll = math::constrain(_target_attitude_roll, -_max_tilt_angle, _max_tilt_angle);
	_target_attitude_pitch = math::constrain(_target_attitude_pitch, -_max_tilt_angle, _max_tilt_angle);

	// Apply tilt angles to all rotors: pitch to pitch channels, roll to yaw channels
	// This creates coordinated tilting for attitude control while maintaining position
	for (int i = 0; i < _tilts.count(); ++i) {
		int pitch_idx = _first_tilt_idx + 2 * i;
		int yaw_idx = _first_tilt_idx + 2 * i + 1;

		if (pitch_idx < NUM_ACTUATORS && yaw_idx < NUM_ACTUATORS) {
			// Normalize angles to [-1, 1] servo range
			actuator_sp(pitch_idx) = _target_attitude_pitch / _max_tilt_angle;
			actuator_sp(yaw_idx) = _target_attitude_roll / _max_tilt_angle;

			// Apply saturation
			actuator_sp(pitch_idx) = math::constrain(actuator_sp(pitch_idx), actuator_min(pitch_idx), actuator_max(pitch_idx));
			actuator_sp(yaw_idx) = math::constrain(actuator_sp(yaw_idx), actuator_min(yaw_idx), actuator_max(yaw_idx));
		}
	}
}

// Mode 2: Fixed Attitude, Position Change via Tilt Variation
// RC roll/pitch sticks control position (forward/back/left/right)
// Tilts are limited to small angles to maintain stable attitude
void ActuatorEffectivenessTiltquad::applyMode2Control(
	ActuatorVector &actuator_sp,
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	// Lock tilt angles to near-zero (±5°) to maintain stable hover attitude
	// RC roll/pitch commands adjust differential thrust between rotors for position control
	float fixed_tilt_angle = 5.0f * (float)M_PI / 180.0f;  // 5 degrees max deviation
	float normalized_fixed_tilt = fixed_tilt_angle / _max_tilt_angle;

	// Note: In Mode 2, the base effectiveness matrix already provides position control
	// via the thruster allocation. We just need to limit tilt to maintain attitude.
	// The control_sp signals for thrust and position are handled by the main allocator.

	for (int i = 0; i < _tilts.count(); ++i) {
		int pitch_idx = _first_tilt_idx + 2 * i;
		int yaw_idx = _first_tilt_idx + 2 * i + 1;

		if (pitch_idx < NUM_ACTUATORS && yaw_idx < NUM_ACTUATORS) {
			// Constrain tilt angles to ±5° deviation to maintain attitude stability
			actuator_sp(pitch_idx) = math::constrain(actuator_sp(pitch_idx), -normalized_fixed_tilt, normalized_fixed_tilt);
			actuator_sp(yaw_idx) = math::constrain(actuator_sp(yaw_idx), -normalized_fixed_tilt, normalized_fixed_tilt);

			// Ensure within actuator bounds
			actuator_sp(pitch_idx) = math::constrain(actuator_sp(pitch_idx), actuator_min(pitch_idx), actuator_max(pitch_idx));
			actuator_sp(yaw_idx) = math::constrain(actuator_sp(yaw_idx), actuator_min(yaw_idx), actuator_max(yaw_idx));
		}
	}
}

void ActuatorEffectivenessTiltquad::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// Note: the values '-1', '1' and '0' are just to indicate a negative,
	// positive or no saturation to the rate controller. The actual magnitude is not used.
	if (_yaw_tilt_saturation_flags.tilt_yaw_pos) {
		status.unallocated_torque[2] = 1.f;

	} else if (_yaw_tilt_saturation_flags.tilt_yaw_neg) {
		status.unallocated_torque[2] = -1.f;

	} else {
		status.unallocated_torque[2] = 0.f;
	}
}
