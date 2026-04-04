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
// Update parameter values
int32_t flight_mode_param = 0;
param_get(_param_flight_mode_handle, &flight_mode_param);

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

if (rc_switch_param == 1) {
rc_value = manual_control.roll;
} else if (rc_switch_param == 2) {
rc_value = manual_control.pitch;
} else if (rc_switch_param == 3) {
rc_value = manual_control.yaw;
} else if (rc_switch_param == 4) {
rc_value = manual_control.throttle;
} else if (rc_switch_param == 5) {
rc_value = manual_control.aux1;
} else if (rc_switch_param == 6) {
rc_value = manual_control.aux2;
} else if (rc_switch_param == 7) {
rc_value = manual_control.aux3;
} else if (rc_switch_param == 8) {
rc_value = manual_control.aux4;
}

// Low (<0) = MODE1, High (>=0) = MODE2
if (rc_value < 0.0f) {
_current_flight_mode = FlightMode::MODE1_FIXED_POSITION_ATTITUDE_CHANGE;
} else {
_current_flight_mode = FlightMode::MODE2_FIXED_ATTITUDE_POSITION_CHANGE;
}
}
}
// Priority 3: Use parameter value if RC switch not enabled
else {
_current_flight_mode = (FlightMode)flight_mode_param;
}

// Apply trim/offset to tilt actuators to center them
// CRITICAL: Only apply offsets starting from _first_tilt_idx (should be 4, after motors)
// If _first_tilt_idx is 0, it means motors weren't properly added and this would corrupt motor outputs
if (_first_tilt_idx >= 4) {
for (int i = 0; i < _tilts.count(); ++i) {
int tilt_idx = _first_tilt_idx + i;
if (tilt_idx < NUM_ACTUATORS) {
actuator_sp(tilt_idx) += _tilt_offsets(tilt_idx);
}
}
} else {
PX4_ERR("Tiltquad: CRITICAL ERROR - _first_tilt_idx=%d is invalid (should be >= 4)! NOT applying offsets to prevent motor corruption!", _first_tilt_idx);
}

// Store current flight mode targets for logging/debugging
_target_attitude_roll = control_sp(ControlAxis::ROLL);
_target_attitude_pitch = control_sp(ControlAxis::PITCH);

// Apply mode-specific tilt constraints
if (_current_flight_mode == FlightMode::MODE2_FIXED_ATTITUDE_POSITION_CHANGE) {
// MODE2: Limit tilt to ±5°
float max_tilt_normalized = (5.0f * (float)M_PI / 180.0f) / _max_tilt_angle;
for (int i = 0; i < _tilts.count(); ++i) {
int tilt_idx = _first_tilt_idx + i;
if (tilt_idx < NUM_ACTUATORS) {
actuator_sp(tilt_idx) = math::constrain(actuator_sp(tilt_idx), -max_tilt_normalized, max_tilt_normalized);
}
}
}
// MODE1: No constraint, allow full tilt range (±90°)

// Yaw saturation detection for rate controller anti-windup
bool yaw_saturated_positive = true;
bool yaw_saturated_negative = true;

for (int i = 0; i < _tilts.count(); ++i) {
// Custom yaw saturation logic: only declare yaw saturated if all tilts are 
// at the negative or positive yawing limit
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
