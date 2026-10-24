#pragma once

#include "Angle.hpp"
#include "PendulumPhysics.hpp"

#include <cmath>

constexpr double IMPULSE_1 = 0.05;
constexpr double IMPULSE_2 = 0.03;
constexpr double LAUNCH_DELAY_SEC = 0.085;
constexpr float ANGLE_CHANGE_EPS = 1e-6f;

struct UserActionState {
    bool simulationRunning = false;
    double stableTime = 0.0;
    float lastAngle1 = 0.f;
    float lastAngle2 = 0.f;
};

inline void initializeFromAngles(PendulumState& s, UserActionState& a,
                                 float angle1, float angle2) {
    s.theta1 = angle1 * 2.0 * kPi;
    s.theta2 = angle2 * 2.0 * kPi;
    s.p1 = IMPULSE_1;
    s.p2 = IMPULSE_2;
    a.simulationRunning = true;
    a.stableTime = 0.0;
}

// Applies retrigger, angle edits and the hold/launch gesture. Returns true when
// the caller explicitly set a new state this sample.
inline bool applyUserActions(PendulumState& s, UserActionState& a,
                             float angle1, float angle2,
                             bool retrigger, bool angleHeld, double sampleTime) {
    bool stateExplicitlySet = false;

    if (retrigger) {
        s.theta1 = angle1 * 2.0 * kPi;
        s.theta2 = angle2 * 2.0 * kPi;
        s.p1 = IMPULSE_1;
        s.p2 = IMPULSE_2;
        a.simulationRunning = true;
        a.stableTime = 0.0;
        stateExplicitlySet = true;
    }

    // Catches dropdown and precise keyboard entry, not only knob drags.
    const bool angleValueChanged =
        (std::abs(angle1 - a.lastAngle1) > ANGLE_CHANGE_EPS) ||
        (std::abs(angle2 - a.lastAngle2) > ANGLE_CHANGE_EPS);
    a.lastAngle1 = angle1;
    a.lastAngle2 = angle2;
    if (angleValueChanged && !angleHeld) {
        s.theta1 = angle1 * 2.0 * kPi;
        s.theta2 = angle2 * 2.0 * kPi;
        s.p1 = IMPULSE_1;
        s.p2 = IMPULSE_2;
        a.simulationRunning = true;
        a.stableTime = 0.0;
        stateExplicitlySet = true;
    }

    if (angleHeld) {
        s.theta1 = angle1 * 2.0 * kPi;
        s.theta2 = angle2 * 2.0 * kPi;
        s.p1 = 0.0;
        s.p2 = 0.0;
        a.simulationRunning = false;
        a.stableTime = 0.0;
        stateExplicitlySet = true;
    } else if (!a.simulationRunning) {
        a.stableTime += sampleTime;
        if (a.stableTime >= LAUNCH_DELAY_SEC) {
            a.simulationRunning = true;
            a.stableTime = 0.0;
            s.p1 = IMPULSE_1;
            s.p2 = IMPULSE_2;
        }
    }

    return stateExplicitlySet;
}
