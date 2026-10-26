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

	// 几何或舵机配置更新后清除旧的差速记忆，避免跨构型施加均衡修正。
	_balance_motor_filtered.setZero();
	_balance_filter_initialized = false;
	_secondary_last_update = 0;
	_roll_priority_filtered = 0.f;
	_priority_last_update = 0;

	return (rotors_added_successfully && tilts_added_successfully);
}

void ActuatorEffectivenessMCTilt::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	const hrt_abstime now = hrt_absolute_time();
	const float dt = _secondary_last_update > 0 && now > _secondary_last_update
			 ? math::constrain((now - _secondary_last_update) * 1e-6f, 0.f, 0.05f) : 0.f;
	_secondary_last_update = now;
	updateSetpointWithDt(control_sp, actuator_sp, actuator_min, actuator_max, dt);
}

void ActuatorEffectivenessMCTilt::updateSetpointWithDt(const matrix::Vector<float, NUM_AXES> &control_sp,
		ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max, float dt)
{
	if (!_nonlinear_initialized) {
		// 非四电机/四舵机的原有 MCTilt 继续使用静态分配。
		actuator_sp += _tilt_offsets;
		return;
	}

	ActuatorVector solution = _nonlinear_sp;

	// 输入异常时保持上一周期的有效输出，避免 NaN 传播到电机和舵机。
	if (!control_sp.isAllFinite()) {
		actuator_sp = solution;
		return;
	}

	// 内环每周期运行；单次迭代可保持响应，同时降低 CUAV 7 Nano 上 rate_ctrl 的计算负载。
	constexpr int iterations = 1;
	// 阻尼可抑制单轴构型接近奇异位形时，伪逆把微小控制误差放大成舵机抖动。
	constexpr float damping = 0.15f;
	constexpr float motor_step = 0.12f;
	constexpr float servo_step = 0.03f;
	// 新次级目标只用于四个独立的 Yaw and Pitch 舵机，保留其他 MCTilt 映射的原有行为。
	bool reference_available = _first_tilt_idx == 4 && _tilts.count() == 4;
	int used_tilts = 0;

	for (int rotor = 0; rotor < 4 && reference_available; ++rotor) {
		const int servo = _mc_rotors.geometry().rotors[rotor].tilt_index;

		if (servo < 0 || servo >= 4 || (used_tilts & (1 << servo))) {
			reference_available = false;
			break;
		}

		used_tilts |= 1 << servo;
		const auto &tilt = _tilts.config(servo);
		reference_available = tilt.axis == ActuatorEffectivenessTilts::TiltAxis::PitchLike
				      && tilt.control == ActuatorEffectivenessTilts::Control::YawAndPitch
				      && tilt.max_angle - tilt.min_angle > FLT_EPSILON;
	}

	// 行程不足时优先保持姿态、偏航和升力，允许前向位置误差暂时增大。
	float axis_weight[NUM_AXES] {3.f, 3.f, 4.f, 0.5f, 0.5f, 3.f};

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
		bool roll_priority_available = _param_ca_mct_rpri.get() > FLT_EPSILON
					       && _first_tilt_idx == 4 && _tilts.count() == 4;
		int used_tilts_for_priority = 0;

		for (int rotor = 0; rotor < 4 && roll_priority_available; ++rotor) {
			const int servo = _mc_rotors.geometry().rotors[rotor].tilt_index;

			if (servo < 0 || servo >= 4 || (used_tilts_for_priority & (1 << servo))) {
				roll_priority_available = false;
				break;
			}

			used_tilts_for_priority |= 1 << servo;
			const auto &tilt = _tilts.config(servo);
			roll_priority_available = tilt.axis == ActuatorEffectivenessTilts::TiltAxis::PitchLike
						  && tilt.control == ActuatorEffectivenessTilts::Control::YawAndPitch;
		}

		if (roll_priority_available) {
			float lowest_motor = FLT_MAX;
			float mean_tilt_angle = 0.f;

			for (int i = 0; i < 4; ++i) {
				lowest_motor = math::min(lowest_motor, solution(i));
				mean_tilt_angle += fabsf(servoAngle(i, solution(_first_tilt_idx + i))) * 0.25f;
			}

			// 只有大俯仰、低端电机余量不足且滚转力矩欠分配同时出现时才让出少量偏航权重。
			const float motor_scale = math::constrain((0.18f - lowest_motor) / 0.12f, 0.f, 1.f);
			const float tilt_scale = math::constrain((mean_tilt_angle - math::radians(15.f))
						 / math::radians(10.f), 0.f, 1.f);
			const float roll_shortfall = math::max(fabsf(target(ControlAxis::ROLL) - current_wrench(ControlAxis::ROLL)),
							       fabsf(_requested_wrench(ControlAxis::ROLL) - _achieved_wrench(ControlAxis::ROLL)));
			const float roll_scale = math::constrain((roll_shortfall - 0.004f) / 0.012f, 0.f, 1.f);
			const float requested_priority = motor_scale * tilt_scale * roll_scale;
			const hrt_abstime now = hrt_absolute_time();
			const float priority_dt = _priority_last_update > 0 && now > _priority_last_update
						  ? math::constrain((now - _priority_last_update) * 1e-6f, 0.f, 0.05f) : 0.f;
			_priority_last_update = now;
			_roll_priority_filtered += priority_dt / (0.2f + priority_dt)
						   * (requested_priority - _roll_priority_filtered);
			// 滚转权重保持 3；偏航最低为 1.75，避免直接放弃航向控制。
			axis_weight[ControlAxis::YAW] = 4.f - 2.25f * math::constrain(_param_ca_mct_rpri.get(), 0.f, 1.f)
							* _roll_priority_filtered;

		} else {
			_roll_priority_filtered = 0.f;
			_priority_last_update = 0;
		}

		Jacobian jacobian;
		computeJacobian(solution, current_wrench, jacobian);
		const WrenchVector error = target - current_wrench;
		Jacobian weighted_jacobian = jacobian;
		WrenchVector weighted_error = error;

		for (int axis = 0; axis < NUM_AXES; ++axis) {
			// 单轴构型不可直接控制 Fy；其余轴按姿态、偏航、升力优先级加权。
			weighted_error(axis) *= axis_weight[axis];

			for (int actuator = 0; actuator < NUM_ACTUATORS; ++actuator) {
				weighted_jacobian(axis, actuator) *= axis_weight[axis];
			}
		}

		// J^T (J J^T + lambda^2 I)^-1 是阻尼最小二乘解，秩不足时仍保持连续。
		matrix::SquareMatrix<float, NUM_AXES> normal_matrix = weighted_jacobian * weighted_jacobian.transpose();

		for (int axis = 0; axis < NUM_AXES; ++axis) {
			normal_matrix(axis, axis) += damping * damping;
		}

		matrix::SquareMatrix<float, NUM_AXES> normal_inverse;

		if (!normal_matrix.I(normal_inverse)) {
			break;
		}

		ActuatorVector delta = weighted_jacobian.transpose() * (normal_inverse * weighted_error);

		if (!delta.isAllFinite()) {
			break;
		}

		// 主目标尚未跟上时暂停次级优化；不通过增加电机最低输出强行抬升总推力。
		const bool secondary_available = reference_available && dt > 0.f
						 && fabsf(error(ControlAxis::ROLL)) < 0.05f
						 && fabsf(error(ControlAxis::PITCH)) < 0.05f
						 && fabsf(error(ControlAxis::YAW)) < 0.05f
						 && fabsf(error(ControlAxis::THRUST_Z)) < 0.05f;
		ActuatorVector secondary_direction;
		secondary_direction.setZero();
		bool balance_available = secondary_available && _param_ca_mct_mbal.get() > FLT_EPSILON;
		float motor_mean = 0.f;

		for (int rotor = 0; rotor < _first_tilt_idx; ++rotor) {
			motor_mean += solution(rotor) * 0.25f;
			balance_available &= actuator_max(rotor) - actuator_min(rotor) > 0.5f;
		}

		balance_available &= motor_mean > 0.15f && motor_mean < 0.85f;

		if (balance_available) {
			const float alpha = dt / (0.35f + dt);
			float filtered_mean = 0.f;
			float filtered_min = FLT_MAX;
			float filtered_max = -FLT_MAX;

			for (int rotor = 0; rotor < 4; ++rotor) {
				// 从当前输出初始化，避免参数切换后滤波器从零爬升误判电机余量。
				_balance_motor_filtered(rotor) = _balance_filter_initialized
								 ? _balance_motor_filtered(rotor) + alpha * (solution(rotor) - _balance_motor_filtered(rotor))
								 : solution(rotor);
				filtered_mean += _balance_motor_filtered(rotor) * 0.25f;
				filtered_min = math::min(filtered_min, _balance_motor_filtered(rotor));
				filtered_max = math::max(filtered_max, _balance_motor_filtered(rotor));
			}

			_balance_filter_initialized = true;
			// 小差速保留给正常配平；差值超过 0.2 后平滑启用均衡，避免平飞时多余舵机动作。
			const float spread_scale = math::constrain((filtered_max - filtered_min - 0.2f) / 0.2f, 0.f, 1.f);
			// 额外保护低端余量：这是软目标，低油门时停用，不设置硬电机下限。
			const float reserve = math::min(0.15f, 0.5f * filtered_mean);

			for (int rotor = 0; rotor < 4; ++rotor) {
				secondary_direction(rotor) = _param_ca_mct_mbal.get()
							     * (spread_scale * (filtered_mean - _balance_motor_filtered(rotor))
								+ 2.f * math::max(reserve - _balance_motor_filtered(rotor), 0.f));
			}

		} else {
			_balance_filter_initialized = false;
		}

		if (secondary_available) {
			const Vector3f force{target(3), target(4), target(5)};
			// 低推力时平滑回零；悬停时跟随期望合力，不能持续拉向机械零位。
			const float force_scale = math::constrain((force.norm() - 0.02f) / 0.08f, 0.f, 1.f);

			for (int servo = 0; servo < 4; ++servo) {
				const auto &tilt = _tilts.config(servo);
				const float direction = math::radians((float)tilt.tilt_direction);
				const float horizontal_force = force(0) * cosf(direction) + force(1) * sinf(direction);
				const float angle = force_scale * atan2f(horizontal_force, -force(2));
				const float reference = math::constrain(2.f * (angle - tilt.min_angle)
									/ (tilt.max_angle - tilt.min_angle) - 1.f, -1.f, 1.f);
				secondary_direction(_first_tilt_idx + servo) = _param_ca_mct_sref.get()
						* (reference - solution(_first_tilt_idx + servo));
			}

			// 合力方向参考与电机均衡一起投影；保留姿态力矩需要的角差，不强制四舵机同角。
			const ActuatorVector projected = secondary_direction - weighted_jacobian.transpose()
							 * (normal_inverse * (weighted_jacobian * secondary_direction));

			if (projected.isAllFinite()) {
				float scale = dt;

				for (int i = 0; i < 8; ++i) {
					const float step = i < _first_tilt_idx ? motor_step : servo_step;
					// 先确定主分配的可行增量，再整体缩小次级方向，避免逐项截断破坏投影。
					const float lower = math::max(actuator_min(i) - solution(i), -step);
					const float upper = math::min(actuator_max(i) - solution(i), step);
					delta(i) = math::constrain(delta(i), lower, upper);
					const float speed = fabsf(projected(i));

					if (speed > FLT_EPSILON) {
						scale = math::min(scale, 0.25f * dt / speed);
						const float room = projected(i) > 0.f ? upper - delta(i) : delta(i) - lower;
						scale = math::min(scale, math::max(room, 0.f) / speed);
					}
				}

				delta += scale * projected;
			}
		}

		for (int i = 0; i < _first_tilt_idx; ++i) {
			solution(i) = math::constrain(solution(i) + math::constrain(delta(i), -motor_step, motor_step),
						      actuator_min(i), actuator_max(i));
		}

		for (int i = _first_tilt_idx; i < _first_tilt_idx + _tilts.count(); ++i) {
			// 目标四舵机构型已采用按秒、经投影的合力参考；其他映射保留原有回中。
			const float center_correction = reference_available ? 0.f : 0.002f * (_tilt_offsets(i) - solution(i));
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
	if (matrix_index != 0 || !_nonlinear_initialized || !actuator_sp.isAllFinite()) {
		return;
	}

	// 必须使用最终发布值，避免标准输出限速后模型状态与真实舵机指令不同步。
	_nonlinear_sp = actuator_sp;
	_achieved_wrench = computeWrench(actuator_sp);
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

void ActuatorEffectivenessMCTilt::computeJacobian(const ActuatorVector &actuator,
		const WrenchVector &current_wrench, Jacobian &jacobian) const
{
	jacobian.setZero();
	constexpr float epsilon = 0.01f;
	const int actuator_count = _first_tilt_idx + _tilts.count();

	// 前向差分配合当前输出复用，每周期少做 8 次完整模型计算。
	for (int i = 0; i < actuator_count; ++i) {
		ActuatorVector positive = actuator;
		positive(i) += epsilon;
		const WrenchVector derivative = (computeWrench(positive) - current_wrench) / epsilon;

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
