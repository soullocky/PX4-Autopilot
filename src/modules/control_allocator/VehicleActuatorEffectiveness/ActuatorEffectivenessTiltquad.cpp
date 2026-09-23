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

	// 输入异常时保持上一周期的有效输出，避免 NaN 传播到电机和舵机。
	if (!control_sp.isAllFinite()) {
		actuator_sp = solution;
		return;
	}

	// 内环每周期运行；单次迭代可保持响应，同时避免数值分配挤占 rate_ctrl 工作队列。
	constexpr int iterations = 1;
	// 阻尼用于抑制双轴机构接近奇异位形时的解跳变。
	constexpr float damping = 0.15f;
	constexpr float servo_center_gain = 0.002f;
	// 以下步长仅限制单次数值迭代，实际角速度由 CA_Rx_SLEW/CA_SVx_SLEW 统一限制。
	constexpr float motor_step = 0.12f;
	constexpr float servo_step = 0.03f;
	// 当执行器行程不足时，优先保持姿态和升力，允许水平位置误差暂时增大。
	constexpr float axis_weight[NUM_AXES] {3.f, 3.f, 4.f, 0.5f, 0.5f, 3.f};
	constexpr int num_tiltquad_rotors = 4;
	int roll_servo_indices[num_tiltquad_rotors] {-1, -1, -1, -1};
	int pitch_servo_indices[num_tiltquad_rotors] {-1, -1, -1, -1};
	bool roll_yaw_configured = _mc_rotors.geometry().num_rotors == num_tiltquad_rotors
				   && _first_tilt_idx == num_tiltquad_rotors && _tilts.count() >= 8;

	for (int rotor = 0; rotor < num_tiltquad_rotors && roll_yaw_configured; ++rotor) {
		const auto &rotor_config = _mc_rotors.geometry().rotors[rotor];
		roll_servo_indices[rotor] = rotor_config.tilt_index_roll >= 0 ? rotor_config.tilt_index_roll * 2 : -1;
		pitch_servo_indices[rotor] = rotor_config.tilt_index_pitch >= 0 ? rotor_config.tilt_index_pitch * 2 + 1 : -1;
		const int roll_servo = roll_servo_indices[rotor];
		const int pitch_servo = pitch_servo_indices[rotor];

		roll_yaw_configured &= roll_servo >= 0 && roll_servo < _tilts.count()
				       && pitch_servo >= 0 && pitch_servo < _tilts.count();

		if (roll_yaw_configured) {
			roll_yaw_configured &= _tilts.config(roll_servo).axis == ActuatorEffectivenessTilts::TiltAxis::RollLike
					       && _tilts.config(roll_servo).control == ActuatorEffectivenessTilts::Control::YawAndRoll
					       && _tilts.config(pitch_servo).axis == ActuatorEffectivenessTilts::TiltAxis::PitchLike;
		}
	}

	for (int iteration = 0; iteration < iterations; ++iteration) {
		const WrenchVector current_wrench = computeWrench(solution);
		Jacobian jacobian;
		computeJacobian(solution, current_wrench, jacobian);
		const WrenchVector error = control_sp - current_wrench;
		Jacobian weighted_jacobian = jacobian;
		WrenchVector weighted_error = error;

		for (int axis = 0; axis < NUM_AXES; ++axis) {
			// 加权最小二乘：Mx/My/Mz/Fz 的误差权重大于 Fx/Fy。
			weighted_error(axis) *= axis_weight[axis];

			for (int actuator = 0; actuator < NUM_ACTUATORS; ++actuator) {
				weighted_jacobian(axis, actuator) *= axis_weight[axis];
			}
		}

		bool roll_yaw_available = roll_yaw_configured;

		for (int rotor = 0; rotor < num_tiltquad_rotors && roll_yaw_available; ++rotor) {
			const int actuator_index = _first_tilt_idx + roll_servo_indices[rotor];
			// 内层机构接近限位时恢复等权分配，让外层机构作为偏航备用。
			roll_yaw_available &= solution(actuator_index) > actuator_min(actuator_index) + 0.1f
					      && solution(actuator_index) < actuator_max(actuator_index) - 0.1f;
		}

		matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> mobility_jacobian_transpose = weighted_jacobian.transpose();

		if (roll_yaw_available) {
			const float pitch_differential_mobility = math::constrain(_param_ca_tq_pit_dw.get(), 0.05f, 1.f);

			for (int axis = 0; axis < NUM_AXES; ++axis) {
				float pitch_common = 0.f;

				for (int rotor = 0; rotor < num_tiltquad_rotors; ++rotor) {
					pitch_common += mobility_jacobian_transpose(_first_tilt_idx + pitch_servo_indices[rotor], axis);
				}

				pitch_common /= (float)num_tiltquad_rotors;

				for (int rotor = 0; rotor < num_tiltquad_rotors; ++rotor) {
					const int actuator_index = _first_tilt_idx + pitch_servo_indices[rotor];
					const float pitch_differential = mobility_jacobian_transpose(actuator_index, axis) - pitch_common;
					// 仅降低外层四舵机的差动自由度，共同俯仰变姿的响应保持不变。
					mobility_jacobian_transpose(actuator_index, axis) = pitch_common
							+ pitch_differential_mobility * pitch_differential;
				}
			}
		}

		// D J^T (J D J^T + lambda^2 I)^-1：D 用于降低大惯量外层机构的差动代价。
		matrix::SquareMatrix<float, NUM_AXES> normal_matrix = weighted_jacobian * mobility_jacobian_transpose;

		for (int axis = 0; axis < NUM_AXES; ++axis) {
			normal_matrix(axis, axis) += damping * damping;
		}

		matrix::SquareMatrix<float, NUM_AXES> normal_inverse;

		if (!normal_matrix.I(normal_inverse)) {
			break;
		}

		ActuatorVector delta = mobility_jacobian_transpose * (normal_inverse * weighted_error);

		// 偏航有舵机矢量控制余量时，将过大的电机推力差作为次级目标；不强制四电机等速。
		// 投影到六维加权雅可比的近似零空间，避免直接削弱姿态、升力及位置主目标。
		const float balance_gain = _param_ca_tq_mbal.get();

		if (balance_gain > FLT_EPSILON && roll_yaw_available) {
			float motor_sum = 0.f;
			float motor_min = solution(0);
			float motor_max = solution(0);
			bool motors_available = true;

			for (int rotor = 0; rotor < num_tiltquad_rotors; ++rotor) {
				motors_available &= actuator_max(rotor) - actuator_min(rotor) > 0.5f;
				motor_sum += solution(rotor);
				motor_min = math::min(motor_min, solution(rotor));
				motor_max = math::max(motor_max, solution(rotor));
			}

			// 小的正常电机差异不驱动额外的舵机运动；分化越严重，均衡作用越充分。
			const float spread_scale = math::constrain((motor_max - motor_min - 0.2f) / 0.4f, 0.f, 1.f);

			if (motors_available && spread_scale > FLT_EPSILON) {
				ActuatorVector balance_direction;
				balance_direction.setZero();
				const float motor_mean = motor_sum * 0.25f;

				for (int rotor = 0; rotor < num_tiltquad_rotors; ++rotor) {
					balance_direction(rotor) = motor_mean - solution(rotor);
				}

				const WrenchVector balance_wrench = weighted_jacobian * balance_direction;
				const ActuatorVector projected_balance = balance_direction
					- mobility_jacobian_transpose * (normal_inverse * balance_wrench);

				if (projected_balance.isAllFinite()) {
					delta += balance_gain * spread_scale * projected_balance;
				}
			}
		}

		if (!delta.isAllFinite()) {
			break;
		}

		for (int i = 0; i < _first_tilt_idx; ++i) {
			solution(i) = math::constrain(solution(i) + math::constrain(delta(i), -motor_step, motor_step),
						      actuator_min(i), actuator_max(i));
		}

		for (int i = _first_tilt_idx; i < _first_tilt_idx + _tilts.count(); ++i) {
			// 弱回中项约束双轴冗余解漂移，不取代输出层的标准舵机限速。
			const float center_correction = servo_center_gain * (_tilt_offsets(i) - solution(i));
			const float servo_delta = math::constrain(delta(i) + center_correction, -servo_step, servo_step);
			solution(i) = math::constrain(solution(i) + servo_delta,
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

	if (!actuator_sp.isAllFinite()) {
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
	float total_thrust = 0.f;
	float roll_authority = 0.f;
	float pitch_authority = 0.f;
	float yaw_authority = 0.f;

	for (int i = 0; i < geometry.num_rotors; ++i) {
		const auto &rotor = geometry.rotors[i];
		const float ct = fabsf(rotor.thrust_coef);
		const matrix::Vector3f axis = rotorAxis(i, actuator);
		const matrix::Vector3f thrust = actuator(i) * rotor.thrust_coef * axis;
		// 与 ActuatorEffectivenessRotors 保持相同的反扭矩符号约定。
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

	// 将物理 CT 和力臂产生的六维量转换为 PX4 控制器使用的归一化尺度。
	wrench(ControlAxis::ROLL) /= math::max(roll_authority, FLT_EPSILON);
	wrench(ControlAxis::PITCH) /= math::max(pitch_authority, FLT_EPSILON);
	wrench(ControlAxis::YAW) /= math::max(yaw_authority, FLT_EPSILON);

	for (int i = ControlAxis::THRUST_X; i <= ControlAxis::THRUST_Z; ++i) {
		wrench(i) /= math::max(total_thrust, FLT_EPSILON);
	}

	return wrench;
}

// 对非线性六维模型进行数值线性化，供每次迭代计算执行器修正量。
void ActuatorEffectivenessTiltquad::computeJacobian(const ActuatorVector &actuator,
		const WrenchVector &current_wrench, Jacobian &jacobian) const
{
	jacobian.setZero();
	constexpr float epsilon = 0.01f;
	const int actuator_count = _first_tilt_idx + _tilts.count();

	// 前向差分可同时覆盖双轴旋转顺序和推力/倾角的乘积耦合。
	// 当前点输出由调用方复用，每周期少做 12 次完整推力模型计算。
	for (int i = 0; i < actuator_count; ++i) {
		ActuatorVector positive = actuator;
		positive(i) += epsilon;
		const WrenchVector derivative = (computeWrench(positive) - current_wrench) / epsilon;

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
