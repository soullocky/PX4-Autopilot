/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>
#include "ActuatorEffectivenessMCTilt.hpp"

using namespace matrix;

class TestMCTilt : public ActuatorEffectivenessMCTilt
{
public:
	TestMCTilt() : ActuatorEffectivenessMCTilt(nullptr)
	{
		updateParams();
		Configuration config{};
		getEffectivenessMatrix(config, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
	}

	using ActuatorEffectivenessMCTilt::computeWrench;
	using ActuatorEffectivenessMCTilt::servoAngle;
	using ActuatorEffectivenessMCTilt::updateSetpointWithDt;

	void gains(float balance, float reference)
	{
		_param_ca_mct_mbal.set(balance);
		_param_ca_mct_sref.set(reference);
		_param_ca_mct_rpri.set(0.f);
	}

	ActuatorVector state(float motor, float angle_deg)
	{
		ActuatorVector out{};

		for (int i = 0; i < 4; ++i) {
			out(i) = motor;
			const auto &tilt = _tilts.config(i);
			out(4 + i) = 2.f * (math::radians(angle_deg) - tilt.min_angle) / (tilt.max_angle - tilt.min_angle) - 1.f;
		}

		return out;
	}

	ActuatorVector run(const WrenchVector &target, ActuatorVector out, float seconds, int hz)
	{
		ActuatorVector lower{}, upper{};
		upper.setAll(1.f);

		for (int i = 4; i < NUM_ACTUATORS; ++i) { lower(i) = -1.f; }

		setAppliedSetpoint(0, out);
		const float dt = 1.f / hz;

		for (int n = 0; n < (int)(seconds * hz); ++n) {
			const ActuatorVector previous = out;
			updateSetpointWithDt(target, out, lower, upper, dt);

			for (int i = 4; i < 8; ++i) {
				// 日志中的 3.5 是全行程秒数，重放真实输出层限速并反馈最终指令。
				out(i) = previous(i) + math::constrain(out(i) - previous(i), -2.f * dt / 3.5f, 2.f * dt / 3.5f);
			}

			setAppliedSetpoint(0, out);
		}

		return out;
	}
};

class MCTiltAllocation : public ::testing::Test
{
protected:
	template<typename T> void set(const char *name, T value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set(handle, &value), 0) << name;
	}

	void SetUp() override
	{
		param_control_autosave(false);
		set("CA_ROTOR_COUNT", int32_t{4});
		set("CA_SV_TL_COUNT", int32_t{4});
		const float x[] {.092f, -.090f, .092f, -.092f};
		const float y[] {.090f, -.090f, -.092f, .092f};

		for (int i = 0; i < 4; ++i) {
			char name[24];
			snprintf(name, sizeof(name), "CA_ROTOR%d_PX", i); set(name, x[i]);
			snprintf(name, sizeof(name), "CA_ROTOR%d_PY", i); set(name, y[i]);
			snprintf(name, sizeof(name), "CA_ROTOR%d_PZ", i); set(name, 0.f);
			snprintf(name, sizeof(name), "CA_ROTOR%d_CT", i); set(name, 6.5f);
			snprintf(name, sizeof(name), "CA_ROTOR%d_KM", i); set(name, i < 2 ? .05f : -.05f);
			snprintf(name, sizeof(name), "CA_ROTOR%d_TILT", i); set(name, int32_t{i + 1});
			snprintf(name, sizeof(name), "CA_SV_TL%d_CT", i); set(name, int32_t{3});
			snprintf(name, sizeof(name), "CA_SV_TL%d_AX", i); set(name, int32_t{0});
			snprintf(name, sizeof(name), "CA_SV_TL%d_TD", i); set(name, int32_t{0});
			snprintf(name, sizeof(name), "CA_SV_TL%d_MINA", i); set(name, -80.f);
			snprintf(name, sizeof(name), "CA_SV_TL%d_MAXA", i); set(name, 80.f);
		}
	}

	TestMCTilt::ActuatorVector loggedState()
	{
		TestMCTilt::ActuatorVector out{};
		const float values[] {.589f, .644f, .069f, .115f, 50.2f / 80.f, 41.8f / 80.f, 10.6f / 80.f, 19.3f / 80.f};

		for (int i = 0; i < 8; ++i) { out(i) = values[i]; }

		return out;
	}
};

TEST_F(MCTiltAllocation, TiltedHoverHasNoMechanicalZeroBias)
{
	for (float angle : {-60.f, -30.f, 0.f, 30.f, 60.f}) {
		TestMCTilt allocator;
		allocator.gains(.3f, .2f);
		const auto initial = allocator.state(.38f, angle);
		const auto target = allocator.computeWrench(initial);
		const auto result = allocator.run(target, initial, 8.f, 800);
		EXPECT_LT(TestMCTilt::ActuatorVector(result - initial).norm(), .003f) << angle;
		EXPECT_LT((Vector<float, 6>(allocator.computeWrench(result) - target).norm()), .001f);
	}
}

TEST_F(MCTiltAllocation, LoggedImbalanceRecoversWithoutChangingWrench)
{
	TestMCTilt allocator;
	allocator.gains(.3f, .2f);
	const auto initial = loggedState();
	const float target_data[] {-.094f, -.034f, .149f, .235f, 0.f, -.267f};
	const Vector<float, 6> target(target_data);
	const auto result = allocator.run(target, initial, 20.f, 800);
	float minimum = 1.f, maximum = 0.f, min_angle = 180.f, max_angle = -180.f;

	for (int i = 0; i < 4; ++i) {
		minimum = math::min(minimum, result(i));
		maximum = math::max(maximum, result(i));
		const float angle = math::degrees(allocator.servoAngle(i, result(4 + i)));
		min_angle = math::min(min_angle, angle);
		max_angle = math::max(max_angle, angle);
	}

	EXPECT_GT(minimum, .15f);
	EXPECT_LT(maximum - minimum, .42f);
	EXPECT_LT(max_angle - min_angle, 32.f);
	EXPECT_LT((Vector<float, 6>(allocator.computeWrench(result) - target).norm()), .003f);
}

TEST_F(MCTiltAllocation, SecondaryMotionIsIndependentOfLoopFrequency)
{
	TestMCTilt slow, fast;
	slow.gains(.3f, .2f);
	fast.gains(.3f, .2f);
	const auto initial = loggedState();
	const auto target = slow.computeWrench(initial);
	const auto a = slow.run(target, initial, 15.f, 200);
	const auto b = fast.run(target, initial, 15.f, 800);
	EXPECT_LT(TestMCTilt::ActuatorVector(a - b).norm(), .015f);
}

TEST_F(MCTiltAllocation, SmallLevelMotorDifferencesDoNotRecruitTilts)
{
	TestMCTilt allocator;
	allocator.gains(.3f, .2f);
	auto initial = allocator.state(.38f, 0.f);
	initial(0) = .41f;
	initial(1) = .45f;
	initial(2) = .31f;
	initial(3) = .35f;
	const auto target = allocator.computeWrench(initial);
	const auto result = allocator.run(target, initial, 10.f, 800);
	EXPECT_LT(TestMCTilt::ActuatorVector(result - initial).norm(), .001f);
}

TEST_F(MCTiltAllocation, ZeroThrustDoesNotEnforceMotorFloor)
{
	TestMCTilt allocator;
	allocator.gains(.3f, .2f);
	const auto result = allocator.run(Vector<float, 6> {}, allocator.state(.38f, 40.f), 25.f, 400);
	ASSERT_TRUE(result.isAllFinite());

	for (int i = 0; i < 4; ++i) {
		EXPECT_NEAR(result(i), 0.f, .001f);
		EXPECT_LT(fabsf(allocator.servoAngle(i, result(4 + i))), math::radians(1.f));
	}
}

TEST_F(MCTiltAllocation, PermutedMappingAndAsymmetricTravel)
{
	const int mapping[] {3, 1, 4, 2};

	for (int i = 0; i < 4; ++i) {
		char name[24];
		snprintf(name, sizeof(name), "CA_ROTOR%d_TILT", i); set(name, int32_t{mapping[i]});
		snprintf(name, sizeof(name), "CA_SV_TL%d_MINA", i); set(name, -50.f - 5.f * i);
	}

	TestMCTilt allocator;
	allocator.gains(.3f, .2f);
	const auto initial = allocator.state(.38f, 35.f);
	const auto target = allocator.computeWrench(initial);
	const auto result = allocator.run(target, initial, 8.f, 400);
	EXPECT_LT(TestMCTilt::ActuatorVector(result - initial).norm(), .003f);
}

TEST_F(MCTiltAllocation, UnreachableTargetAndInvalidInputRemainBounded)
{
	TestMCTilt allocator;
	allocator.gains(.3f, .2f);
	const float target_data[] {2.f, -2.f, 2.f, 1.f, 0.f, -1.f};
	const Vector<float, 6> target(target_data);
	const auto result = allocator.run(target, allocator.state(.38f, 60.f), 3.f, 400);
	ASSERT_TRUE(result.isAllFinite());

	for (int i = 0; i < 8; ++i) {
		EXPECT_GE(result(i), i < 4 ? 0.f : -1.f);
		EXPECT_LE(result(i), 1.f);
	}

	control_allocator_status_s status{};
	allocator.getUnallocatedControl(0, status);
	EXPECT_GT(fabsf(status.unallocated_torque[2]), .1f);
	auto invalid = target;
	invalid(0) = NAN;
	const auto held = allocator.run(invalid, result, .1f, 400);
	EXPECT_LT(TestMCTilt::ActuatorVector(held - result).norm(), .00001f);
}
