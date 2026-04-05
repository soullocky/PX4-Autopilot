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
 * Actuator effectiveness for Tiltquad VTOL (2x2 dual-rotor with independent tilt servos)
 *
 * Tiltquad Configuration:
 * - 4 motors: PWM 1-4 (provides primary thrust for all modes)
 * - 8 servo channels: PWM 5-12
 *   - Servo 1-2: Tilt 1 roll/pitch
 *   - Servo 3-4: Tilt 2 roll/pitch
 *   - Servo 5-6: Tilt 3 roll/pitch
 *   - Servo 7-8: Tilt 4 roll/pitch
 *
 * Flight Modes:
 * - MODE1: Full tilt range (±90°) for attitude change via tilt
 * - MODE2: Limited tilt (±5°) for position control with stable attitude
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessTiltquad.hpp"
#include <px4_log.h>

using namespace matrix;

ActuatorEffectivenessTiltquad::ActuatorEffectivenessTiltquad(ModuleParams *parent)
: ModuleParams(parent),
  _param_flight_mode_handle(param_find("CA_TILTQUAD_MODE")),
  _param_max_tilt_angle_handle(param_find("CA_TILT_MAX_ANG")),
  _param_rc_switch_channel(param_find("CA_TILT_RC_CHN")),
  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, true),
  _tilts(this)
{
}

bool
ActuatorEffectivenessTiltquad::getEffectivenessMatrix(Configuration &configuration,
EffectivenessUpdateReason external_update)
{
if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
return false;
}

// Add motor effectiveness (4 motors for thrust)
_mc_rotors.enableYawByDifferentialThrust(!_tilts.hasYawControl());
const bool rotors_added_successfully = _mc_rotors.addActuators(configuration);

if (!rotors_added_successfully) {
PX4_ERR("Tiltquad: FAILED to add rotors to effectiveness matrix!");
return false;
}

int num_actuators_before_tilts = configuration.num_actuators_matrix[0];
PX4_INFO("Tiltquad: Motors added successfully. num_actuators = %d (should be 4)", num_actuators_before_tilts);

// Add tilt servo effectiveness (8 servos: 2 DOF per motor)
_first_tilt_idx = configuration.num_actuators_matrix[0];
_tilts.updateTorqueSign(_mc_rotors.geometry());
const bool tilts_added_successfully = _tilts.addActuators(configuration);

if (!tilts_added_successfully) {
PX4_ERR("Tiltquad: FAILED to add tilts to effectiveness matrix!");
return false;
}

int num_actuators_after_tilts = configuration.num_actuators_matrix[0];
PX4_INFO("Tiltquad: Tilts added successfully. _first_tilt_idx=%d, tilts_count=%d, total_actuators=%d",
_first_tilt_idx, _tilts.count(), num_actuators_after_tilts);

// Verify motor/tilt boundary is correct
if (_first_tilt_idx != 4) {
PX4_WARN("Tiltquad: WARNING - _first_tilt_idx=%d (expected 4)! This may indicate wrong actuator ordering.", _first_tilt_idx);
}

// Set offset so tilts point upwards when control input == 0
// (trim is 0 if min_angle == -max_angle)
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

void ActuatorEffectivenessTiltquad::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	// Update parameters from PX4 parameter system
	ModuleParams::updateParams();

	// Read parameter values
	int32_t flight_mode_param = 0;
	param_get(_param_flight_mode_handle, &flight_mode_param);
	_current_flight_mode = (FlightMode)flight_mode_param;

	float max_tilt_angle_param = 90.0f;
	param_get(_param_max_tilt_angle_handle, &max_tilt_angle_param);
	_max_tilt_angle = max_tilt_angle_param * (float)M_PI / 180.0f;  // Convert to radians

	actuator_sp += _tilt_offsets;
	// TODO: dynamic matrix update

float max_tilt_param = 5.0f;
param_get(_param_max_tilt_angle_handle, &max_tilt_param);
_max_tilt_angle = max_tilt_param;

int32_t rc_switch_param = 0;
param_get(_param_rc_switch_channel, &rc_switch_param);

// Update current nav state for QGC EXTERNAL mode detection
if (_vehicle_status_sub.update()) {
_current_nav_state = _vehicle_status_sub.get().nav_state;
}

// Priority 1: Check QGC-set flight mode (EXTERNAL modes)
// EXTERNAL1 (nav_state=23) -> MODE1
// EXTERNAL2 (nav_state=24) -> MODE2
if (_current_nav_state == vehicle_status_s::NAVIGATION_STATE_EXTERNAL1) {
_current_flight_mode = FlightMode::MODE1_FIXED_POSITION_ATTITUDE_CHANGE;
} else if (_current_nav_state == vehicle_status_s::NAVIGATION_STATE_EXTERNAL2) {
_current_flight_mode = FlightMode::MODE2_FIXED_ATTITUDE_POSITION_CHANGE;
}
// Priority 2: RC switch (if enabled)
else if (rc_switch_param > 0 && rc_switch_param <= 8) {
manual_control_setpoint_s manual_control{};
if (_manual_control_sub.update(&manual_control)) {
float rc_value = 0.0f;

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

// Compute normalized demands once (avoid repeated divisions in loop)
float pitch_demand = _target_attitude_pitch / _max_tilt_angle;
float roll_demand = _target_attitude_roll / _max_tilt_angle;

// Apply tilt angles based on each servo's configured control type and axis
// Each servo at _tilts index i maps to actuator _first_tilt_idx + i
for (int i = 0; i < _tilts.count(); ++i) {
int servo_idx = _first_tilt_idx + i;

if (servo_idx >= NUM_ACTUATORS) {
continue;
}

const auto &tilt_cfg = _tilts.config(i);

// MODE1: Override tilt servo setpoints with direct attitude control
// This provides direct stick-to-attitude mapping for precise manual control
if (tilt_cfg.control == ActuatorEffectivenessTilts::Control::None) {
// No control on this servo, skip
continue;
} else if (tilt_cfg.axis == ActuatorEffectivenessTilts::TiltAxis::PitchLike) {
// PitchLike axis: can control pitch and/or yaw
if (tilt_cfg.control == ActuatorEffectivenessTilts::Control::Pitch
    || tilt_cfg.control == ActuatorEffectivenessTilts::Control::YawAndPitch) {
// Apply pitch command to pitch-controlled servo
actuator_sp(servo_idx) = pitch_demand;
} else if (tilt_cfg.control == ActuatorEffectivenessTilts::Control::Yaw) {
// Apply roll (yaw) command to yaw-controlled servo
actuator_sp(servo_idx) = roll_demand;
}
} else if (tilt_cfg.axis == ActuatorEffectivenessTilts::TiltAxis::RollLike) {
// RollLike axis: controls roll
if (tilt_cfg.control == ActuatorEffectivenessTilts::Control::Roll
    || tilt_cfg.control == ActuatorEffectivenessTilts::Control::RollAndPitch) {
actuator_sp(servo_idx) = roll_demand;
}
}

// Apply saturation
actuator_sp(servo_idx) = math::constrain(actuator_sp(servo_idx), actuator_min(servo_idx), actuator_max(servo_idx));
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

// Apply tilt angle constraints based on each servo's configured control type
// From CA_SV_TL${i}_CT: Control Type
for (int i = 0; i < _tilts.count(); ++i) {
int servo_idx = _first_tilt_idx + i;

if (servo_idx >= NUM_ACTUATORS) {
continue;
}

const auto &tilt_cfg = _tilts.config(i);

// Constrain tilt angles to ±5° deviation to maintain attitude stability
// Only constrain active tilt servos (not "None" type)
if (tilt_cfg.control != ActuatorEffectivenessTilts::Control::None) {
actuator_sp(servo_idx) = math::constrain(actuator_sp(servo_idx),
                                           -normalized_fixed_tilt,
                                           normalized_fixed_tilt);
}

// Ensure within actuator bounds
actuator_sp(servo_idx) = math::constrain(actuator_sp(servo_idx),
                                           actuator_min(servo_idx),
                                           actuator_max(servo_idx));
}
}

void ActuatorEffectivenessTiltquad::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
// Report yaw saturation from tilt constraints to rate controller for anti-windup
// Values: -1 (negative sat), 0 (no sat), +1 (positive sat)
if (_yaw_tilt_saturation_flags.tilt_yaw_pos) {
status.unallocated_torque[2] = 1.f;
} else if (_yaw_tilt_saturation_flags.tilt_yaw_neg) {
status.unallocated_torque[2] = -1.f;
} else {
status.unallocated_torque[2] = 0.f;
}
}
