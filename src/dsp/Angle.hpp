#pragma once

#include <cmath>

// Wraps an angle into [-pi, pi). The double pendulum is 2*pi-periodic, so
// wrapping preserves the physics while keeping trig arguments bounded.
inline double normalizeAngle(double a) {
    constexpr double pi = 3.14159265358979323846;
    a = std::fmod(a + pi, 2.0 * pi);
    if (a < 0.0) a += 2.0 * pi;
    return a - pi;
}

// Cheap fast path for the common case where the angle is already in range.
inline double wrapAngle(double a) {
    constexpr double pi = 3.14159265358979323846;
    return (a < -pi || a >= pi) ? normalizeAngle(a) : a;
}
