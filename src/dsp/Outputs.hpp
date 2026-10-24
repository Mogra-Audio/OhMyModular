#pragma once

#include "Angle.hpp"
#include "PendulumPhysics.hpp"

#include <cmath>

constexpr double ANGLE_TO_VOLT = 5.0 / kPi;
constexpr double SLEW_RATE = 500.0;             // max V/s
constexpr float DISCRETE_STEP = 1.0f / 12.0f;   // one semitone in Rack pitch

inline float clampVolts(float volts) {
    return volts < -5.f ? -5.f : (volts > 5.f ? 5.f : volts);
}

inline float smoothVolts(double theta) {
    return clampVolts((float)(theta * ANGLE_TO_VOLT));
}

inline float discreteVolts(double theta) {
    const float raw = (float)(theta * ANGLE_TO_VOLT);
    return clampVolts(std::round(raw / DISCRETE_STEP) * DISCRETE_STEP);
}

inline float slewToward(float current, float target, float maxDelta) {
    float delta = target - current;
    if (delta > maxDelta) delta = maxDelta;
    else if (delta < -maxDelta) delta = -maxDelta;
    return current + delta;
}

struct PendulumOutputs {
    float smooth1 = 0.f;
    float discrete1 = 0.f;
    float discrete2 = 0.f;
    float smooth2 = 0.f;

    void reset() {
        smooth1 = discrete1 = discrete2 = smooth2 = 0.f;
    }

    void process(const PendulumState& s, double sampleTime) {
        const float limit = (float)(SLEW_RATE * sampleTime);
        smooth1 = slewToward(smooth1, smoothVolts(s.theta1), limit);
        discrete1 = slewToward(discrete1, discreteVolts(s.theta1), limit);
        discrete2 = slewToward(discrete2, discreteVolts(s.theta2), limit);
        smooth2 = slewToward(smooth2, smoothVolts(s.theta2), limit);
    }
};
