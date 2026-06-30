/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_params.c
 * Parameters for multicopter attitude controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLL_P, 6.5f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.5f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_P, 2.8f);

/**
 * Yaw weight
 *
 * A fraction [0,1] deprioritizing yaw compared to roll and pitch in non-linear attitude control.
 * Deprioritizing yaw is necessary because multicopters have much less control authority
 * in yaw compared to the other axes and it makes sense because yaw is not critical for
 * stable hovering or 3D navigation.
 *
 * For yaw control tuning use MC_YAW_P. This ratio has no impact on the yaw gain.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_WEIGHT, 0.4f);

/**
 * Max roll rate
 *
 * Limit for roll rate in manual and auto modes (except acro).
 * Has effect for large rotations in autonomous mode, to avoid large control
 * output and mixer saturation.
 *
 * This is not only limited by the vehicle's properties, but also by the maximum
 * measurement rate of the gyro.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_MAX, 220.0f);

/**
 * Max pitch rate
 *
 * Limit for pitch rate in manual and auto modes (except acro).
 * Has effect for large rotations in autonomous mode, to avoid large control
 * output and mixer saturation.
 *
 * This is not only limited by the vehicle's properties, but also by the maximum
 * measurement rate of the gyro.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_MAX, 220.0f);

/**
 * Max yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_MAX, 200.0f);

/**
 * Enable AUX attitude target input
 *
 * When enabled, AUX5 and AUX6 act as an independent attitude bias composed
 * (via quaternion multiplication) on top of the stick-driven roll/pitch
 * setpoint. The stick range is therefore not compressed by the bias.
 * In Manual/Stabilized mode the bias will still cause horizontal drift
 * because attitude directly drives acceleration; in Position mode the
 * position controller is expected to compensate this drift, fully
 * decoupling stick direction from the bias.
 *
 * 使能 AUX 通道姿态目标输入。
 * 置 1 时, AUX5/AUX6 作为独立的姿态偏置, 通过四元数乘法 q_sp = q_aux * q_stick
 * 与摇杆姿态合成 —— 摇杆量程不会被 AUX 偏置压缩。
 *   - 手动/自稳模式: 飞行方向由合成姿态决定, AUX 偏置会引起水平漂移,
 *     飞手需自行补偿;
 *   - 位置模式(后续扩展): 位置控制器会通过反馈/积分自动补偿偏置引起的
 *     漂移, 从而摇杆方向真正与 AUX 基础姿态独立。
 *
 * @boolean
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(MC_AUX_ATT_EN, 0);

/**
 * Maximum roll angle commanded by AUX5
 *
 * AUX5 通道对应的最大滚转角，AUX5 满量程时对应的滚转角度。
 *
 * @unit deg
 * @min 0
 * @max 90
 * @decimal 1
 * @increment 1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_AUX_ROLL_MAX, 0.0f);

/**
 * Maximum pitch angle commanded by AUX6
 *
 * AUX6 通道对应的最大俯仰角，AUX6 满量程时对应的俯仰角度。
 *
 * @unit deg
 * @min 0
 * @max 90
 * @decimal 1
 * @increment 1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_AUX_PITCH_MAX, 0.0f);

/**
 * Manual tilt input filter time constant
 *
 * Setting this parameter to 0 disables the filter
 *
 * @unit s
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_MAN_TILT_TAU, 0.0f);
