#pragma once

#include "Chaos.hpp"
#include "Outputs.hpp"
#include "Settle.hpp"
#include "Smoothing.hpp"
#include "UserActions.hpp"

struct PendulumCore {
    struct Inputs {
        float angle1 = 0.f;
        float angle2 = 0.f;
        float time = 0.5f;
        float skew = 0.5f;
        float timeCv = 0.f;
        float skewCv = 0.f;
        bool timeCvActive = false;
        bool skewCvActive = false;
        float chaos = 0.f;
        bool retrigger = false;
        bool angleHeld = false;
    };

    PendulumState state;
    PendulumState ghost;
    MassPair masses;
    MassPair lastMasses;
    EnergySettle settle;
    UserActionState actions;
    PendulumOutputs outputs;

    float smoothedTime = 0.5f;
    float smoothedSkew = 0.5f;
    float cvSmoothAlpha = 0.f;
    bool chaosWasActive = false;

    PendulumCore() {
        ghost.theta1 = GHOST_THETA1;
        ghost.theta2 = GHOST_THETA2;
        cvSmoothAlpha = onePoleAlpha(1.f / 48000.f, CV_SMOOTH_TIME);
    }

    void onSampleRateChange(float sampleTime) {
        cvSmoothAlpha = onePoleAlpha(sampleTime, CV_SMOOTH_TIME);
    }

    void reset(float angle1, float angle2) {
        state = PendulumState();
        initializeFromAngles(state, actions, angle1, angle2);
        settle.cancel();
        chaosWasActive = false;
        outputs.reset();
    }

    void process(const Inputs& in, double sampleTime) {
        smoothedTime = approach(smoothedTime,
                                in.timeCvActive ? in.timeCv : in.time,
                                cvSmoothAlpha);
        smoothedSkew = approach(smoothedSkew,
                                in.skewCvActive ? in.skewCv : in.skew,
                                cvSmoothAlpha);

        MassPair nextMasses;
        nextMasses.m1 = 0.5 + (1.0 - smoothedSkew) * 1.0;
        nextMasses.m2 = 0.5 + smoothedSkew * 1.0;

        const bool stateExplicitlySet =
            applyUserActions(state, actions, in.angle1, in.angle2,
                             in.retrigger, in.angleHeld, sampleTime);

        const bool chaosActive = isChaosActive(in.chaos);
        if (chaosActive) {
            if (!chaosWasActive) {
                // Entering chaos: capture the current energy as the return target.
                settle.setTarget(computeEnergy(state, nextMasses));
                settle.cancel();
            }
            if (stateExplicitlySet) {
                settle.setTarget(computeEnergy(state, nextMasses));
            }
        } else if (chaosWasActive) {
            // Exiting chaos: start the energy settle.
            settle.start();
        }
        chaosWasActive = chaosActive;

        // A user action while settling cancels the settle.
        if (stateExplicitlySet && !chaosActive) {
            settle.cancel();
        }

        if (preserveAngularVelocity(state, lastMasses, nextMasses)) {
            lastMasses = nextMasses;
        }
        masses = nextMasses;

        stepGhost(ghost, sampleTime, masses, ghostSubsteps(in.chaos));

        if (actions.simulationRunning) {
            rk4Step(state, sampleTime, masses, computeTimeScale(smoothedTime));
        }

        applyChaos(state, ghost, in.chaos, sampleTime);
        settle.process(state, sampleTime, masses);

        checkPendulumStability(state);
        state.theta1 = wrapAngle(state.theta1);
        state.theta2 = wrapAngle(state.theta2);

        outputs.process(state, sampleTime);
    }
};
