#pragma once

#include "Angle.hpp"
#include "ArmKinematics.hpp"

#include <algorithm>
#include <cmath>

constexpr double GRAVITY = 9.81;
constexpr double M1_DEFAULT = 1.0;
constexpr double M2_DEFAULT = 1.0;
constexpr double MASS_EPS = 1e-6;

struct PendulumState {
    double theta1 = 0.0;
    double theta2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
};

struct MassPair {
    double m1 = M1_DEFAULT;
    double m2 = M2_DEFAULT;
};

struct AngularVelocity {
    double w1;
    double w2;
};

struct PendulumDerivatives {
    double dTheta1;
    double dTheta2;
    double dP1;
    double dP2;
};

inline AngularVelocity computeAngularVelocity(const PendulumState& s, const MassPair& m) {
    const double cosDiff = std::cos(s.theta1 - s.theta2);
    const double I1 = (m.m1 + m.m2) * ARM_L1 * ARM_L1;
    const double I2 = m.m2 * ARM_L2 * ARM_L2;
    const double I12 = m.m2 * ARM_L1 * ARM_L2;
    double denom = I1 * I2 - I12 * I12 * cosDiff * cosDiff;
    denom = std::max(denom, 1e-10);

    AngularVelocity w;
    w.w1 = (I2 * s.p1 - I12 * cosDiff * s.p2) / denom;
    w.w2 = (I1 * s.p2 - I12 * cosDiff * s.p1) / denom;
    return w;
}

inline PendulumDerivatives computePendulumDerivatives(const PendulumState& s,
                                                      const MassPair& m,
                                                      double timeScale) {
    const double sin1 = std::sin(s.theta1);
    const double sin2 = std::sin(s.theta2);
    const double sinDiff = std::sin(s.theta1 - s.theta2);
    const AngularVelocity w = computeAngularVelocity(s, m);

    PendulumDerivatives d;
    d.dTheta1 = w.w1 * timeScale;
    d.dTheta2 = w.w2 * timeScale;

    const double I12 = m.m2 * ARM_L1 * ARM_L2;
    const double coupling = I12 * w.w1 * w.w2 * sinDiff;
    d.dP1 = (-coupling - (m.m1 + m.m2) * GRAVITY * ARM_L1 * sin1) * timeScale;
    d.dP2 = (coupling - m.m2 * GRAVITY * ARM_L2 * sin2) * timeScale;
    return d;
}

inline void rk4Step(PendulumState& s, double dt, const MassPair& m, double timeScale) {
    const PendulumDerivatives d1 = computePendulumDerivatives(s, m, timeScale);

    PendulumState s2 = s;
    s2.theta1 += 0.5 * dt * d1.dTheta1;
    s2.theta2 += 0.5 * dt * d1.dTheta2;
    s2.p1 += 0.5 * dt * d1.dP1;
    s2.p2 += 0.5 * dt * d1.dP2;
    const PendulumDerivatives d2 = computePendulumDerivatives(s2, m, timeScale);

    PendulumState s3 = s;
    s3.theta1 += 0.5 * dt * d2.dTheta1;
    s3.theta2 += 0.5 * dt * d2.dTheta2;
    s3.p1 += 0.5 * dt * d2.dP1;
    s3.p2 += 0.5 * dt * d2.dP2;
    const PendulumDerivatives d3 = computePendulumDerivatives(s3, m, timeScale);

    PendulumState s4 = s;
    s4.theta1 += dt * d3.dTheta1;
    s4.theta2 += dt * d3.dTheta2;
    s4.p1 += dt * d3.dP1;
    s4.p2 += dt * d3.dP2;
    const PendulumDerivatives d4 = computePendulumDerivatives(s4, m, timeScale);

    s.theta1 += dt * (d1.dTheta1 + 2.0 * d2.dTheta1 + 2.0 * d3.dTheta1 + d4.dTheta1) / 6.0;
    s.theta2 += dt * (d1.dTheta2 + 2.0 * d2.dTheta2 + 2.0 * d3.dTheta2 + d4.dTheta2) / 6.0;
    s.p1 += dt * (d1.dP1 + 2.0 * d2.dP1 + 2.0 * d3.dP1 + d4.dP1) / 6.0;
    s.p2 += dt * (d1.dP2 + 2.0 * d2.dP2 + 2.0 * d3.dP2 + d4.dP2) / 6.0;
}

inline double computePotentialEnergy(const PendulumState& s, const MassPair& m) {
    return -(m.m1 + m.m2) * GRAVITY * ARM_L1 * std::cos(s.theta1)
           - m.m2 * GRAVITY * ARM_L2 * std::cos(s.theta2);
}

inline double computeEnergy(const PendulumState& s, const MassPair& m) {
    const AngularVelocity w = computeAngularVelocity(s, m);
    const double kinetic = 0.5 * (s.p1 * w.w1 + s.p2 * w.w2);
    return kinetic + computePotentialEnergy(s, m);
}

inline double computeTimeScale(float timeParam) {
    return 0.2 + timeParam * 1.6
         + std::max(0.0f, timeParam - 0.5f) * 10.4;
}

// Rebuilds momenta for new masses while keeping the angular velocities fixed.
inline bool massChanged(const MassPair& oldMasses, const MassPair& newMasses) {
    return std::abs(newMasses.m1 - oldMasses.m1) > MASS_EPS ||
           std::abs(newMasses.m2 - oldMasses.m2) > MASS_EPS;
}

inline bool preserveAngularVelocity(PendulumState& s, const MassPair& oldMasses,
                                    const MassPair& newMasses) {
    if (!massChanged(oldMasses, newMasses)) {
        return false;
    }

    const double cosDiff = std::cos(s.theta1 - s.theta2);
    const AngularVelocity oldW = computeAngularVelocity(s, oldMasses);

    const double I1 = (newMasses.m1 + newMasses.m2) * ARM_L1 * ARM_L1;
    const double I2 = newMasses.m2 * ARM_L2 * ARM_L2;
    const double I12 = newMasses.m2 * ARM_L1 * ARM_L2;
    s.p1 = I1 * oldW.w1 + I12 * cosDiff * oldW.w2;
    s.p2 = I2 * oldW.w2 + I12 * cosDiff * oldW.w1;
    return true;
}

inline void checkPendulumStability(PendulumState& s) {
    if (std::isnan(s.theta1) || std::isinf(s.theta1) ||
        std::isnan(s.theta2) || std::isinf(s.theta2) ||
        std::isnan(s.p1) || std::isinf(s.p1) ||
        std::isnan(s.p2) || std::isinf(s.p2)) {
        s.theta1 = s.theta2 = s.p1 = s.p2 = 0.0;
    }
}
