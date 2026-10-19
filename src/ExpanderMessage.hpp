#pragma once

#include <type_traits>

// Messages exchanged between Oh My Pendulum and its right-hand expander.
// Each sender owns a pair of buffers and flips them once per sample, so the
// receiver always sees the previous sample's data (one sample of latency).

// Main module -> expander: pendulum state and the active theme.
struct PendulumToExpanderMsg {
    double theta1 = 0.0;
    double theta2 = 0.0;
    int theme = 0;
};

// Expander -> main module: normalized CV inputs.
struct ExpanderToPendulumMsg {
    float timeCV = 0.f;
    float skewCV = 0.f;
    bool timeConnected = false;
    bool skewConnected = false;
};

static_assert(std::is_trivially_copyable<PendulumToExpanderMsg>::value,
              "PendulumToExpanderMsg must be trivially copyable");
static_assert(std::is_trivially_copyable<ExpanderToPendulumMsg>::value,
              "ExpanderToPendulumMsg must be trivially copyable");

// Maps a 0..10 V control voltage to the 0..1 range used by parameters.
inline float cvToUnit(float volts) {
    float unit = volts * 0.1f;
    return unit < 0.f ? 0.f : (unit > 1.f ? 1.f : unit);
}
