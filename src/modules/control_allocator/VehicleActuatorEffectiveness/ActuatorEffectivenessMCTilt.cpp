/****************************************************************************
 *
 *   Copyright (c) 2021-2023 PX4 Development Team. All rights reserved.
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

#include "ActuatorEffectivenessMCTilt.hpp"

#include <matrix/matrix/AxisAngle.hpp>

using namespace matrix;

ActuatorEffectivenessMCTilt::ActuatorEffectivenessMCTilt(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, true),
	  _tilts(this)
{
}

bool
ActuatorEffectivenessMCTilt::getEffectivenessMatrix(Configuration &configuration,
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

	if (!_nonlinear_initialized && _mc_rotors.geometry().num_rotors == 4 && _tilts.count() == 4) {
		// 单轴矢量四旋翼从悬停附近开始非线性求解。
		_nonlinear_sp.setZero();

		for (int i = 0; i < 4; ++i) {
			_nonlinear_sp(i) = 0.5f;
			_nonlinear_sp(_first_tilt_idx + i) = _tilt_offsets(_first_tilt_idx + i);
		}

		_nonlinear_initialized = true;
	}

	return (rotors_added_successfully && tilts_added_successfully);
}

void ActuatorEffectivenessMCTilt::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	if (!_nonlinear_initialized) {
		// 非四电机/四舵机的原有 MCTilt 继续使用静态分配。
		actuator_sp += _tilt_offsets;
		return;
	}

	ActuatorVector solution = _nonlinear_sp;
	constexpr int iterations = 5;

	for (int iteration = 0; iteration < iterations; ++iteration) {
		Jacobian jacobian;
		computeJacobian(solution, jacobian);
		matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> inverse;

		if (!matrix::geninv(jacobian, inverse)) {
			break;
		}

		const ActuatorVector delta = inverse * (control_sp - computeWrench(solution));

		for (int i = 0; i < _first_tilt_idx; ++i) {
			solution(i) = math::constrain(solution(i) + math::constrain(delta(i), -0.12f, 0.12f),
						      actuator_min(i), actuator_max(i));
		}

		for (int i = _first_tilt_idx; i < _first_tilt_idx + _tilts.count(); ++i) {
			solution(i) = math::constrain(solution(i) + math::constrain(delta(i), -0.10f, 0.10f),
						      actuator_min(i), actuator_max(i));
		}
	}

	actuator_sp = solution;
	_requested_wrench = control_sp;
}

void ActuatorEffectivenessMCTilt::setAppliedSetpoint(int matrix_index, const ActuatorVector &actuator_sp)
{
	if (matrix_index == 0 && _nonlinear_initialized) {
		_nonlinear_sp = actuator_sp;
		_achieved_wrench = computeWrench(actuator_sp);
	}
}

float ActuatorEffectivenessMCTilt::servoAngle(int servo_index, float setpoint) const
{
	const auto &config = _tilts.config(servo_index);
	return math::lerp(config.min_angle, config.max_angle, math::constrain((setpoint + 1.f) * 0.5f, 0.f, 1.f));
}

matrix::Vector3f ActuatorEffectivenessMCTilt::rotorAxis(int rotor_index, const ActuatorVector &actuator) const
{
	const auto &rotor = _mc_rotors.geometry().rotors[rotor_index];
	const int servo = rotor.tilt_index;

	if (servo < 0 || servo >= _tilts.count()) {
		return matrix::Vector3f{0.f, 0.f, -1.f};
	}

	const float direction = math::radians((float)_tilts.config(servo).tilt_direction);
	const matrix::Vector3f rotation_axis{sinf(direction), -cosf(direction), 0.f};
	return matrix::Dcmf{matrix::AxisAnglef{rotation_axis * servoAngle(servo, actuator(_first_tilt_idx + servo))}}
	       * matrix::Vector3f{0.f, 0.f, -1.f};
}

ActuatorEffectivenessMCTilt::WrenchVector
ActuatorEffectivenessMCTilt::computeWrench(const ActuatorVector &actuator) const
{
	WrenchVector wrench;
	wrench.setZero();

	for (int i = 0; i < _mc_rotors.geometry().num_rotors; ++i) {
		const auto &rotor = _mc_rotors.geometry().rotors[i];
		const matrix::Vector3f thrust = actuator(i) * rotor.thrust_coef * rotorAxis(i, actuator);
		const matrix::Vector3f torque = rotor.position.cross(thrust) - rotor.moment_ratio * thrust;

		for (int n = 0; n < 3; ++n) {
			wrench(n) += torque(n);
			wrench(n + 3) += thrust(n);
		}
	}

	return wrench;
}

void ActuatorEffectivenessMCTilt::computeJacobian(const ActuatorVector &actuator, Jacobian &jacobian) const
{
	jacobian.setZero();
	constexpr float epsilon = 0.01f;

	for (int i = 0; i < _first_tilt_idx + _tilts.count(); ++i) {
		ActuatorVector positive = actuator;
		ActuatorVector negative = actuator;
		positive(i) += epsilon;
		negative(i) -= epsilon;
		const WrenchVector derivative = (computeWrench(positive) - computeWrench(negative)) / (2.f * epsilon);

		for (int n = 0; n < NUM_AXES; ++n) {
			jacobian(n, i) = derivative(n);
		}
	}
}

void ActuatorEffectivenessMCTilt::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	if (!_nonlinear_initialized) {
		return;
	}

	const WrenchVector error = _requested_wrench - _achieved_wrench;

	for (int i = 0; i < 3; ++i) {
		status.unallocated_torque[i] = error(i);
		status.unallocated_thrust[i] = error(i + 3);
	}
}
