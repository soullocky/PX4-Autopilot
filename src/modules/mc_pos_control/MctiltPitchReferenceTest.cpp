#include <gtest/gtest.h>
#include "MctiltPitchReference.hpp"

TEST(MctiltPitchReference, RejectInvalidAndStaleInput)
{
	EXPECT_TRUE(MctiltPitchReference::fresh(1000000, 900000, 0.3f, 0.5f));
	EXPECT_FALSE(MctiltPitchReference::fresh(1000000, 500000, 0.3f, 0.5f));
	EXPECT_FALSE(MctiltPitchReference::fresh(1000000, 0, 0.3f, 0.5f));
	EXPECT_FALSE(MctiltPitchReference::fresh(1000000, 1000001, 0.3f, 0.5f));
	EXPECT_FALSE(MctiltPitchReference::fresh(1000000, 900000, NAN, 0.5f));
	EXPECT_FALSE(MctiltPitchReference::fresh(1000000, 900000, 0.3f, NAN));
}

TEST(MctiltPitchReference, ManualUnchanged)
{
	MctiltPitchReference reference;
	EXPECT_FLOAT_EQ(reference.update(0.4f, 0.f, false, 0.02f, 0.1f, 0.3f), 0.4f);
	EXPECT_FLOAT_EQ(reference.update(-0.2f, 0.f, false, 0.02f, 0.1f, 0.3f), -0.2f);
}

TEST(MctiltPitchReference, EntryTimeoutAndExitHaveNoStep)
{
	MctiltPitchReference reference;
	float previous = reference.update(0.4f, 0.f, false, 0.02f, 0.1f, 0.3f);

	for (int phase = 0; phase < 3; ++phase) {
		for (int i = 0; i < 1000; ++i) {
			// 外部接管到负俯仰、超时回零、退出模式接回正俯仰。
			const float value = reference.update(0.4f, phase == 0 ? -0.4f : 0.f,
							     phase < 2, 0.02f, 0.1f, 0.3f);
			EXPECT_LE(fabsf(value - previous), 0.00201f);
			EXPECT_LE(fabsf(value), 0.40001f);
			previous = value;
		}

		EXPECT_NEAR(previous, phase == 0 ? -0.4f : (phase == 1 ? 0.f : 0.4f), 1e-4f);
	}
}

TEST(MctiltPitchReference, TimeBasedRateAndReset)
{
	MctiltPitchReference slow, fast;
	float a = 0.f, b = 0.f;

	for (int i = 0; i < 50; ++i) { a = slow.update(0.f, 1.f, true, 0.02f, 0.1f, 0.f); }

	for (int i = 0; i < 200; ++i) { b = fast.update(0.f, 1.f, true, 0.005f, 0.1f, 0.f); }

	EXPECT_NEAR(a, 0.1f, 1e-5f);
	EXPECT_NEAR(a, b, 1e-5f);
	slow.reset();
	EXPECT_FLOAT_EQ(slow.update(-0.2f, 0.f, false, 0.02f, 0.1f, 0.f), -0.2f);
}
