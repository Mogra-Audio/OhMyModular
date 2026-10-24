#pragma once

#include <cmath>

constexpr float CV_SMOOTH_TIME = 0.005f;

inline float onePoleAlpha(float sampleTime, float timeConstant) {
    return 1.f - std::exp(-sampleTime / timeConstant);
}

// One-pole smoothing; alpha is precomputed per sample rate.
inline float approach(float current, float target, float alpha) {
    return current + (target - current) * alpha;
}
