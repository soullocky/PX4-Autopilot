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
	// 使用上一周期真实输出作为初值，两次迭代可降低飞控实时计算负担。
	constexpr int iterations = 2;
	// 阻尼可抑制单轴构型接近奇异位形时，伪逆把微小控制误差放大成舵机抖动。
	constexpr float damping = 0.15f;
	constexpr float servo_center_gain = 0.002f;
	constexpr float motor_step = 0.12f;
	constexpr float servo_step = 0.03f;

	for (int iteration = 0; iteration < iterations; ++iteration) {
		WrenchVector target = control_sp;
		WrenchVector current_wrench = computeWrench(solution);
		// 俯仰方向单轴倾转无法直接产生 Fy，不允许不可控误差把舵机持续推向极限。
		target(ControlAxis::THRUST_Y) = current_wrench(ControlAxis::THRUST_Y);

		// 总升力独立分配，避免零推力时舵机雅可比退化导致油门无法启动。
		const float collective_delta = math::constrain(
						 current_wrench(ControlAxis::THRUST_Z) - target(ControlAxis::THRUST_Z), -0.12f, 0.12f);

		for (int i = 0; i < _first_tilt_idx; ++i) {
			solution(i) = math::constrain(solution(i) + collective_delta, actuator_min(i), actuator_max(i));
		}

		current_wrench = computeWrench(solution);
		Jacobian jacobian;
		computeJacobian(solution, jacobian);
		// J^T (J J^T + lambda^2 I)^-1 是阻尼最小二乘解，秩不足时仍保持连续。
		matrix::SquareMatrix<float, NUM_AXES> normal_matrix = jacobian * jacobian.transpose();

		for (int axis = 0; axis < NUM_AXES; ++axis) {
			normal_matrix(axis, axis) += damping * damping;
		}

		matrix::SquareMatrix<float, NUM_AXES> normal_inverse;

		if (!normal_matrix.I(normal_inverse)) {
			continue;
		}

		const ActuatorVector delta = jacobian.transpose() * (normal_inverse * (target - current_wrench));

		for (int i = 0; i < _first_tilt_idx; ++i) {
			solution(i) = math::constrain(solution(i) + math::constrain(delta(i), -motor_step, motor_step),
						      actuator_min(i), actuator_max(i));
		}

		for (int i = _first_tilt_idx; i < _first_tilt_idx + _tilts.count(); ++i) {
			// 弱回中项消除冗余自由度的长期漂移，不代替标准舵机角速度限制。
			const float center_correction = servo_center_gain * (_tilt_offsets(i) - solution(i));
			const float servo_delta = math::constrain(delta(i) + center_correction, -servo_step, servo_step);
			solution(i) = math::constrain(solution(i) + servo_delta,
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
	float total_thrust = 0.f;
	float roll_authority = 0.f;
	float pitch_authority = 0.f;
	float yaw_authority = 0.f;

	for (int i = 0; i < _mc_rotors.geometry().num_rotors; ++i) {
		const auto &rotor = _mc_rotors.geometry().rotors[i];
		const float ct = fabsf(rotor.thrust_coef);
		const matrix::Vector3f thrust = actuator(i) * rotor.thrust_coef * rotorAxis(i, actuator);
		const matrix::Vector3f torque = rotor.position.cross(thrust) - rotor.moment_ratio * thrust;
		total_thrust += ct;
		roll_authority += ct * fabsf(rotor.position(1));
		pitch_authority += ct * fabsf(rotor.position(0));
		yaw_authority += ct * fabsf(rotor.moment_ratio);

		for (int n = 0; n < 3; ++n) {
			wrench(n) += torque(n);
			wrench(n + 3) += thrust(n);
		}
	}

	// 控制器给出的是归一化六维设定值，不能直接与物理 CT/力臂量纲比较。
	wrench(ControlAxis::ROLL) /= math::max(roll_authority, FLT_EPSILON);
	wrench(ControlAxis::PITCH) /= math::max(pitch_authority, FLT_EPSILON);
	wrench(ControlAxis::YAW) /= math::max(yaw_authority, FLT_EPSILON);

	for (int i = ControlAxis::THRUST_X; i <= ControlAxis::THRUST_Z; ++i) {
		wrench(i) /= math::max(total_thrust, FLT_EPSILON);
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
