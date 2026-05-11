/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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


#include "ActuatorEffectivenessTilts.hpp"

#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

ActuatorEffectivenessTilts::ActuatorEffectivenessTilts(ModuleParams *parent)
	: ModuleParams(parent)
{
	for (int i = 0; i < MAX_COUNT; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_CT", i);
		_param_handles[i].control = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_AX", i);
    		_param_handles[i].axis = param_find(buffer);          // 新增
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_MINA", i);
		_param_handles[i].min_angle = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_MAXA", i);
		_param_handles[i].max_angle = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_TD", i);
		_param_handles[i].tilt_direction = param_find(buffer);
	}

	_count_handle = param_find("CA_SV_TL_COUNT");
	updateParams();
}

void ActuatorEffectivenessTilts::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_count_handle, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_count = count;

	for (int i = 0; i < count; i++) {
		param_get(_param_handles[i].control, (int32_t *)&_params[i].control);
		param_get(_param_handles[i].axis, (int32_t *)&_params[i].axis);  // 新增
		param_get(_param_handles[i].tilt_direction, (int32_t *)&_params[i].tilt_direction);
		param_get(_param_handles[i].min_angle, &_params[i].min_angle);
		param_get(_param_handles[i].max_angle, &_params[i].max_angle);

		_params[i].min_angle = math::radians(_params[i].min_angle);
		_params[i].max_angle = math::radians(_params[i].max_angle);

		_torque[i].setZero();
	}
}

bool ActuatorEffectivenessTilts::addActuators(Configuration &configuration)
{
	for (int i = 0; i < _count; i++) {
		configuration.addActuator(ActuatorType::SERVOS, _torque[i], Vector3f{});
	}

	return true;
}

void ActuatorEffectivenessTilts::updateTorqueSign(const ActuatorEffectivenessRotors::Geometry &geometry,
		bool disable_pitch)
{
	// 保险起见：先清零
	for (int i = 0; i < _count; i++) {
		_torque[i].setAll(0.f);
	}

	for (int i = 0; i < geometry.num_rotors; ++i) {

	const auto &rotor = geometry.rotors[i];

        // ================ 1) pitch-like tilt（前后倾：pitch / yaw） ====================
	{
            // Tiltquad uses 8 servos ordered as [roll0, pitch0, roll1, pitch1, ...].
            // CA_ROTORx_TP stays in the 4-element pitch index space, so map it to odd servo slots.
            const int tilt_index = rotor.tilt_index_pitch >= 0 ? rotor.tilt_index_pitch * 2 + 1 : -1;

            if (tilt_index >= 0 && tilt_index < _count
                && _params[tilt_index].axis == TiltAxis::PitchLike) {

                // ---- yaw 符号：沿用原来的旋转 + y 判号逻辑 ----
                if (_params[tilt_index].control == Control::Yaw
                    || _params[tilt_index].control == Control::YawAndPitch) {

                    const float tilt_direction = math::radians((float)_params[tilt_index].tilt_direction);
                    matrix::Vector3f rotated_pos = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, -tilt_direction}} * rotor.position;

                    if (rotated_pos(1) < -0.01f) {
                        _torque[tilt_index](2) = 1.f;    // +control -> +yaw
                    } else if (rotated_pos(1) > 0.01f) {
                        _torque[tilt_index](2) = -1.f;   // +control -> -yaw
                    }
                }

                // ---- pitch 符号：沿用原逻辑，通过 tilt_direction 判断前/后 ----
                if (!disable_pitch && (_params[tilt_index].control == Control::Pitch
                                       || _params[tilt_index].control == Control::YawAndPitch)) {

                    const int tilt_dir_deg = (int)_params[tilt_index].tilt_direction;
                    const bool tilting_forwards = (tilt_dir_deg < 90 || tilt_dir_deg > 270);
                    _torque[tilt_index](1) = tilting_forwards ? -1.f : 1.f;
                }
            }

	}
        // ================ 2) roll-like tilt（左右倾：roll） ===========================
	{
            // CA_ROTORx_TR stays in the 4-element roll index space, so map it to even servo slots.
            const int tilt_index = rotor.tilt_index_roll >= 0 ? rotor.tilt_index_roll * 2 : -1;

            if (tilt_index >= 0 && tilt_index < _count
                && _params[tilt_index].axis == TiltAxis::RollLike) {

                if (_params[tilt_index].control == Control::Roll
                    || _params[tilt_index].control == Control::RollAndPitch) {

                    // 用 rotor.position.y 判断左右：
                    // y > 0 → 右侧电机，+control -> +roll；
                    // y < 0 → 左侧电机，+control -> -roll。
                    const float y = rotor.position(1);

                    if (y > 0.01f) {
                        _torque[tilt_index](0) = 1.f;    // 右侧：+control -> +roll
                    } else if (y < -0.01f) {
                        _torque[tilt_index](0) = -1.f;   // 左侧：+control -> -roll
                    }
                }

                // 如要 roll-like 也帮忙 pitch / yaw，可以在这里设置 _torque(1/2)
            }

        }
	}
}

bool ActuatorEffectivenessTilts::hasYawControl() const
{
	for (int i = 0; i < _count; i++) {
		if (_params[i].control == Control::Yaw || _params[i].control == Control::YawAndPitch) {
			return true;
		}
	}

	return false;
}
