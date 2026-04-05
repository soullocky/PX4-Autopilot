/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include "FunctionProviderBase.hpp"
#include <uORB/topics/actuator_servos.h>

/**
 * Functions: Tilt_Roll1...Tilt_Roll4, Tilt_Pitch1...Tilt_Pitch4
 * 
 * Tiltquad VTOL tilt servo control functions:
 * - Tilt_Roll1-4: Roll (X-axis) tilt servos for Motors 1-4
 * - Tilt_Pitch1-4: Pitch (Y-axis) tilt servos for Motors 1-4
 */
class FunctionTilts : public FunctionProviderBase
{
public:
static_assert(actuator_servos_s::NUM_CONTROLS == 8,
      "Unexpected num tilt servos");

FunctionTilts(const Context &context) :
_topic(&context.work_item, ORB_ID(actuator_servos))
{
for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; ++i) {
_tilt_data.control[i] = NAN;
}
}

static FunctionProviderBase *allocate(const Context &context) { return new FunctionTilts(context); }

void update() override { _topic.update(&_tilt_data); }

float value(OutputFunction func) override 
{ 
// Map output function to tilt servo index
// Tilt_Roll1-4: 501-504 → indices 0,2,4,6
// Tilt_Pitch1-4: 505-508 → indices 1,3,5,7

int func_num = (int)func;
int index = -1;

if (func_num >= 501 && func_num <= 504) {
// Tilt_Roll: maps to even indices (0,2,4,6)
index = (func_num - 501) * 2;
} else if (func_num >= 505 && func_num <= 508) {
// Tilt_Pitch: maps to odd indices (1,3,5,7)
index = (func_num - 505) * 2 + 1;
}

if (index >= 0 && index < actuator_servos_s::NUM_CONTROLS) {
return _tilt_data.control[index];
}

return NAN;
}

uORB::SubscriptionCallbackWorkItem *subscriptionCallback() override { return &_topic; }

float defaultFailsafeValue(OutputFunction func) const override { return 0.f; }

private:
uORB::SubscriptionCallbackWorkItem _topic;
actuator_servos_s _tilt_data{};
};
