#pragma once

#include "PendulumPhysics.hpp"

#include <algorithm>
#include <cmath>

constexpr double SETTLE_TIME = 6.0;    // hard cap; typical convergence ~1.3s
constexpr double SETTLE_RATE = 8.0;    // convergence speed (1/sec), tau = 0.125s
constexpr double ENERGY_EPS  = 0.01;   // "close enough" threshold
constexpr double STUCK_TIME = 0.25;    // seconds of stillness before a nudge
constexpr double STUCK_MOMENTUM = 0.01;
constexpr double NUDGE_ANGLE = 0.01;   // radians

struct EnergySettle {
    double targetEnergy = 0.0;
    double timeRemaining = 0.0;
    double stuckTime = 0.0;

    bool active() const { return timeRemaining > 0.0; }

    void setTarget(double energy) { targetEnergy = energy; }
    void start() { timeRemaining = SETTLE_TIME; stuckTime = 0.0; }
    void cancel() { timeRemaining = 0.0; stuckTime = 0.0; }

    // Servos the total mechanical energy back to targetEnergy. While the target
    // lies below the current potential, only damping is possible; once it drops,
    // the same servo can add kinetic energy back.
    void process(PendulumState& s, double sampleTime, const MassPair& masses) {
        if (timeRemaining <= 0.0) return;

        const double E_cur = computeEnergy(s, masses);
        const double V_cur = computePotentialEnergy(s, masses);
        const double dE = E_cur - targetEnergy;
        double momentumFactor = 1.0;
        bool nudged = false;

        if (std::abs(dE) > ENERGY_EPS) {
            if (V_cur > targetEnergy) {
                // Shed kinetic energy and let gravity bring the pendulum down.
                momentumFactor = std::exp(-SETTLE_RATE * sampleTime);
                s.p1 *= momentumFactor;
                s.p2 *= momentumFactor;

                // A pendulum balanced at the top would stay there forever; a
                // nudge after a short stall breaks the unstable equilibrium.
                const bool nearlyStill = std::abs(s.p1) < STUCK_MOMENTUM &&
                                         std::abs(s.p2) < STUCK_MOMENTUM;
                stuckTime = nearlyStill ? stuckTime + sampleTime : 0.0;
                if (stuckTime >= STUCK_TIME) {
                    s.theta1 += NUDGE_ANGLE;
                    stuckTime = 0.0;
                    nudged = true;
                }
            } else {
                // Target kinetic energy is reachable: scale momenta both ways.
                const double T_target = std::max(targetEnergy - V_cur, 0.0);
                const double T_cur = E_cur - V_cur;
                if (T_cur > 1e-12) {
                    double scale = std::sqrt(T_target / T_cur);
                    // Bound the raw ratio: near-zero kinetic energy must not
                    // produce a huge single-sample jump.
                    scale = std::min(std::max(scale, 0.5), 2.0);
                    const double alpha = 1.0 - std::exp(-SETTLE_RATE * sampleTime);
                    scale = 1.0 + (scale - 1.0) * alpha;
                    s.p1 *= scale;
                    s.p2 *= scale;
                    momentumFactor = scale;
                }
            }
        }

        timeRemaining -= sampleTime;
        // Kinetic energy scales with the square of the momentum factor, so the
        // post-correction energy needs no second full evaluation. The nudge moves
        // the angles, so only that rare sample is recomputed.
        const double E_post = nudged
            ? computeEnergy(s, masses)
            : V_cur + (E_cur - V_cur) * momentumFactor * momentumFactor;
        if (timeRemaining <= 0.0 ||
            std::abs(E_post - targetEnergy) < ENERGY_EPS) {
            timeRemaining = 0.0;
        }
    }
};
