#pragma once

#include <cmath>

constexpr double kPi = 3.14159265358979323846;

// Wraps an angle into [-pi, pi). The double pendulum is 2*pi-periodic, so
// wrapping preserves the physics while keeping trig arguments bounded.
inline double normalizeAngle(double a) {
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a < 0.0) a += 2.0 * kPi;
    return a - kPi;
}

// Cheap fast path for the common case where the angle is already in range.
inline double wrapAngle(double a) {
    return (a < -kPi || a >= kPi) ? normalizeAngle(a) : a;
}
