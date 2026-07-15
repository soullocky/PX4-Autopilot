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

#include <matrix/matrix/AxisAngle.hpp>

using namespace matrix;

ActuatorEffectivenessTiltquad::ActuatorEffectivenessTiltquad(ModuleParams *parent)
	: ModuleParams(parent),
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

	_mc_rotors.enableYawByDifferentialThrust(!_tilts.hasYawControl());
	const bool rotors_added_successfully = _mc_rotors.addActuators(configuration);

	_first_tilt_idx = configuration.num_actuators_matrix[0];
	// 双轴 Tiltquad 才使用 TR/TP 的成对映射。
	_tilts.updateTorqueSign(_mc_rotors.geometry(), false, ActuatorEffectivenessTilts::Mapping::DualAxis);
	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	_tilt_offsets.setZero();

	for (int i = 0; i < _tilts.count(); ++i) {
		float delta_angle = _tilts.config(i).max_angle - _tilts.config(i).min_angle;

		if (delta_angle > FLT_EPSILON) {
			float trim = -1.f - 2.f * _tilts.config(i).min_angle / delta_angle;
			_tilt_offsets(_first_tilt_idx + i) = trim;
		}
	}

	if (!_nonlinear_initialized) {
		// 初值取悬停附近：电机中等推力，舵机位于零倾转位置。
		_nonlinear_sp.setZero();

		for (int i = 0; i < _mc_rotors.geometry().num_rotors; ++i) {
			_nonlinear_sp(i) = 0.5f;
		}

		for (int i = 0; i < _tilts.count(); ++i) {
			_nonlinear_sp(_first_tilt_idx + i) = _tilt_offsets(_first_tilt_idx + i);
		}

		_nonlinear_initialized = true;
	}

	return (rotors_added_successfully && tilts_added_successfully);
}

void ActuatorEffectivenessTiltquad::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	// 非线性高斯-牛顿迭代：联合求解四个电机和八个双轴倾转舵机。
	ActuatorVector solution = _nonlinear_sp;
	constexpr int iterations = 5;
	// 以下步长仅限制单次数值迭代，实际角速度由 CA_Rx_SLEW/CA_SVx_SLEW 统一限制。
	constexpr float motor_step = 0.12f;
	constexpr float servo_step = 0.10f;

	for (int iteration = 0; iteration < iterations; ++iteration) {
		Jacobian jacobian;
		computeJacobian(solution, jacobian);
		matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> inverse;

		if (!matrix::geninv(jacobian, inverse)) {
			break;
		}

		const WrenchVector error = control_sp - computeWrench(solution);
		const ActuatorVector delta = inverse * error;

		for (int i = 0; i < _first_tilt_idx; ++i) {
			solution(i) = math::constrain(solution(i) + math::constrain(delta(i), -motor_step, motor_step),
						      actuator_min(i), actuator_max(i));
		}

		for (int i = _first_tilt_idx; i < _first_tilt_idx + _tilts.count(); ++i) {
			solution(i) = math::constrain(solution(i) + math::constrain(delta(i), -servo_step, servo_step),
						      actuator_min(i), actuator_max(i));
		}
	}

	actuator_sp = solution;
	_requested_wrench = control_sp;
}

void ActuatorEffectivenessTiltquad::setAppliedSetpoint(int matrix_index, const ActuatorVector &actuator_sp)
{
	if (matrix_index != 0) {
		return;
	}

	// 必须使用最终发布值，避免标准输出限速后模型状态与真实舵机指令不同步。
	_nonlinear_sp = actuator_sp;
	_achieved_wrench = computeWrench(actuator_sp);
}

// 将控制分配器使用的归一化舵机量转换为机械倾转角。
float ActuatorEffectivenessTiltquad::servoAngle(int servo_index, float setpoint) const
{
	const auto &config = _tilts.config(servo_index);
	const float normalized = math::constrain((setpoint + 1.f) * 0.5f, 0.f, 1.f);
	return math::lerp(config.min_angle, config.max_angle, normalized);
}

// 依次应用 Roll-like 和 Pitch-like 旋转，得到旋翼当前推力方向。
matrix::Vector3f ActuatorEffectivenessTiltquad::rotorAxis(int rotor_index, const ActuatorVector &actuator) const
{
	const auto &rotor = _mc_rotors.geometry().rotors[rotor_index];
	matrix::Vector3f axis{0.f, 0.f, -1.f};
	const int roll_servo = rotor.tilt_index_roll >= 0 ? rotor.tilt_index_roll * 2 : -1;
	const int pitch_servo = rotor.tilt_index_pitch >= 0 ? rotor.tilt_index_pitch * 2 + 1 : -1;
	const int servos[2] {roll_servo, pitch_servo};

	for (int servo : servos) {
		if (servo < 0 || servo >= _tilts.count()) {
			continue;
		}

		// TD 表示正倾转的水平指向，据此构造水平转轴并依次旋转推力方向。
		const float direction = math::radians((float)_tilts.config(servo).tilt_direction);
		const matrix::Vector3f rotation_axis{sinf(direction), -cosf(direction), 0.f};
		axis = matrix::Dcmf{matrix::AxisAnglef{rotation_axis * servoAngle(servo, actuator(_first_tilt_idx + servo))}} * axis;
	}

	return axis.normalized();
}

ActuatorEffectivenessTiltquad::WrenchVector
ActuatorEffectivenessTiltquad::computeWrench(const ActuatorVector &actuator) const
{
	WrenchVector wrench;
	wrench.setZero();
	const auto &geometry = _mc_rotors.geometry();

	for (int i = 0; i < geometry.num_rotors; ++i) {
		const auto &rotor = geometry.rotors[i];
		const matrix::Vector3f axis = rotorAxis(i, actuator);
		const matrix::Vector3f thrust = actuator(i) * rotor.thrust_coef * axis;
		// 与 ActuatorEffectivenessRotors 保持相同的反扭矩符号约定。
		const matrix::Vector3f torque = rotor.position.cross(thrust) - rotor.moment_ratio * thrust;

		for (int n = 0; n < 3; ++n) {
			wrench(n) += torque(n);
			wrench(n + 3) += thrust(n);
		}
	}

	return wrench;
}

// 对非线性六维模型进行数值线性化，供每次迭代计算执行器修正量。
void ActuatorEffectivenessTiltquad::computeJacobian(const ActuatorVector &actuator, Jacobian &jacobian) const
{
	jacobian.setZero();
	constexpr float epsilon = 0.01f;
	const int actuator_count = _first_tilt_idx + _tilts.count();

	// 数值差分可同时覆盖双轴旋转顺序和推力/倾角的乘积耦合。
	for (int i = 0; i < actuator_count; ++i) {
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

void ActuatorEffectivenessTiltquad::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// 线性分配结果已被非线性求解覆盖，因此用真实模型回填六轴分配误差。
	const WrenchVector error = _requested_wrench - _achieved_wrench;

	for (int i = 0; i < 3; ++i) {
		status.unallocated_torque[i] = error(i);
		status.unallocated_thrust[i] = error(i + 3);
	}
}
