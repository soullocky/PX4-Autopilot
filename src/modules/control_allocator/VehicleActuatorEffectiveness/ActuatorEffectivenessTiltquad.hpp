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
#include <matrix/matrix/PseudoInverse.hpp>

class ActuatorEffectivenessTiltquad : public ModuleParams, public ActuatorEffectiveness
{
public:
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

	/**
	 * 使用非线性迭代联合分配四个电机推力和八个双轴倾转舵机。
	 * @param control_sp 期望六维控制量，顺序为 Mx、My、Mz、Fx、Fy、Fz
	 * @param actuator_sp 输出执行器设定值，顺序为电机在前、倾转舵机在后
	 */
	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

	/** 保存输出层限速、限幅后的真实执行器指令，作为下一周期非线性求解初值。 */
	void setAppliedSetpoint(int matrix_index, const ActuatorVector &actuator_sp) override;

	const char *name() const override { return "Tiltquad"; }

	/** 使用非线性模型的实际六维输出计算并回填未分配控制量。 */
	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

protected:
	using WrenchVector = matrix::Vector<float, NUM_AXES>;
	using Jacobian = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>;

	/**
	 * 根据当前电机推力和双轴倾转角计算机体系六维合力/力矩。
	 * @return 顺序为 Mx、My、Mz、Fx、Fy、Fz 的六维向量
	 */
	WrenchVector computeWrench(const ActuatorVector &actuator) const;

	/**
	 * 使用中心差分计算六维输出对全部执行器设定值的雅可比矩阵。
	 * 该矩阵用于高斯-牛顿非线性控制分配。
	 */
	void computeJacobian(const ActuatorVector &actuator, Jacobian &jacobian) const;

	/**
	 * 根据指定旋翼对应的 Roll/Pitch 舵机角度计算机体系推力单位向量。
	 */
	matrix::Vector3f rotorAxis(int rotor_index, const ActuatorVector &actuator) const;

	/**
	 * 将归一化舵机设定值 [-1, 1] 转换为实际倾转角（rad）。
	 */
	float servoAngle(int servo_index, float setpoint) const;

	ActuatorVector _tilt_offsets;
	ActuatorEffectivenessRotors _mc_rotors;
	ActuatorEffectivenessTilts _tilts;
	ActuatorVector _nonlinear_sp{};
	WrenchVector _requested_wrench{};
	WrenchVector _achieved_wrench{};
	bool _nonlinear_initialized{false};
	int _first_tilt_idx{0};

};
