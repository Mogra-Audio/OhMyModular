#pragma once

#include "Angle.hpp"
#include "PendulumPhysics.hpp"

#include <cmath>

constexpr double GHOST_THETA1 = 2.09439510239;
constexpr double GHOST_THETA2 = 2.79252680319;
constexpr double CHAOS_DEAD = 0.1;
constexpr double CHAOS_INJ_ANGLE = 0.8;
constexpr double CHAOS_INJ_MOM = 6.0;
constexpr double CHAOS_REFERENCE_SAMPLE_RATE = 48000.0;
constexpr double P_MAX = 25.0;
constexpr int GHOST_SUBSTEPS_MAX = 5;

inline bool isChaosActive(double chaos) {
    return chaos > CHAOS_DEAD;
}

inline int ghostSubsteps(double chaos) {
    return 1 + (int)(chaos * (GHOST_SUBSTEPS_MAX - 1));
}

// Chaos knob plus its CV input, already mapped to 0..1.
inline float chaosAmount(float knob, float cvVolts) {
    float c = knob + cvVolts / 5.0f;
    return c < 0.f ? 0.f : (c > 1.f ? 1.f : c);
}

inline void checkGhostStability(PendulumState& ghost) {
    if (std::isnan(ghost.theta1) || std::isinf(ghost.theta1) ||
        std::isnan(ghost.theta2) || std::isinf(ghost.theta2) ||
        std::isnan(ghost.p1) || std::isinf(ghost.p1) ||
        std::isnan(ghost.p2) || std::isinf(ghost.p2)) {
        ghost.theta1 = GHOST_THETA1;
        ghost.theta2 = GHOST_THETA2;
        ghost.p1 = ghost.p2 = 0.0;
    }
}

inline void stepGhost(PendulumState& ghost, double dt, const MassPair& masses,
                      int substeps) {
    if (substeps < 1) substeps = 1;
    const double gdt = dt / (double)substeps;
    for (int i = 0; i < substeps; i++) {
        const PendulumDerivatives d = computePendulumDerivatives(ghost, masses, 1.0);
        ghost.theta1 += gdt * d.dTheta1;
        ghost.theta2 += gdt * d.dTheta2;
        ghost.p1 += gdt * d.dP1;
        ghost.p2 += gdt * d.dP2;
    }
    checkGhostStability(ghost);
    ghost.theta1 = wrapAngle(ghost.theta1);
    ghost.theta2 = wrapAngle(ghost.theta2);
}

inline void applyChaos(PendulumState& s, const PendulumState& ghost,
                       double chaos, double sampleTime) {
    if (chaos <= CHAOS_DEAD) return;
    const double eff = (chaos - CHAOS_DEAD) / (1.0 - CHAOS_DEAD);
    double inj = eff * eff * eff * eff;
    inj *= sampleTime * CHAOS_REFERENCE_SAMPLE_RATE;

    s.theta1 += inj * CHAOS_INJ_ANGLE * std::sin(ghost.theta1);
    s.theta2 += inj * CHAOS_INJ_ANGLE * std::sin(ghost.theta2);
    s.p1 += inj * CHAOS_INJ_MOM * ghost.p1;
    s.p2 += inj * CHAOS_INJ_MOM * ghost.p2;
    s.p1 = P_MAX * std::tanh(s.p1 / P_MAX);
    s.p2 = P_MAX * std::tanh(s.p2 / P_MAX);
}
