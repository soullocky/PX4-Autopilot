#pragma once

#include <lib/mathlib/mathlib.h>
#include <cmath>
#include <cstdint>

// 单轴外部俯仰的无跳变接管；无外部接管时不改变原有 AUX6 滤波响应。
class MctiltPitchReference
{
public:
	// 使用本地接收时钟，拒绝从未接收、未来时间戳、非法值及超时目标。
	static bool fresh(uint64_t now, uint64_t received, float pitch, float timeout_s)
	{
		return received > 0 && now >= received && std::isfinite(pitch)
		       && std::isfinite(timeout_s) && timeout_s > 0.f
		       && (now - received) * 1e-6f < timeout_s;
	}

	void reset() { _initialized = false; _transition = false; }
	float update(float manual, float target, bool external, float dt, float rate, float tau)
	{
		if (!_initialized) {
			_value = manual;
			_initialized = true;
		}

		if (external) { _transition = true; }

		if (external || _transition) {
			const float desired = external ? target : manual;
			const float alpha = dt / (math::max(tau, 0.f) + dt);
			const float step = math::max(rate, 0.f) * dt;
			_value += math::constrain(alpha * (desired - _value), -step, step);

			if (!external && fabsf(_value - manual) < 1e-4f) { _transition = false; }

		} else {
			_value = manual;
		}

		return _value;
	}

private:
	float _value{0.f};
	bool _initialized{false};
	bool _transition{false};
};
