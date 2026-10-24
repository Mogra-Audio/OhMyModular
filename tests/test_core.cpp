#include "TestHarness.hpp"

#include "dsp/PendulumCore.hpp"

#include <cmath>

static PendulumCore::Inputs baseInputs(float angle1, float angle2) {
    PendulumCore::Inputs in;
    in.angle1 = angle1;
    in.angle2 = angle2;
    return in;
}

static void testReset() {
    PendulumCore core;
    core.reset(0.0f, 0.0f);

    CHECK(near(core.state.theta1, 0.0));
    CHECK(near(core.state.theta2, 0.0));
    CHECK(near(core.state.p1, IMPULSE_1));
    CHECK(near(core.state.p2, IMPULSE_2));
    CHECK(core.actions.simulationRunning);
    CHECK(!core.settle.active());
    CHECK(!core.chaosWasActive);
    CHECK(near(core.outputs.smooth1, 0.f));
    CHECK(near(core.outputs.smooth2, 0.f));
}

static void testSingleSample() {
    PendulumCore core;
    core.reset(0.0f, 0.0f);
    core.process(baseInputs(0.0f, 0.0f), 1.0 / 48000.0);

    CHECK(std::isfinite(core.state.theta1));
    CHECK(std::isfinite(core.state.theta2));
    CHECK(std::fabs(core.state.theta1) <= kPi);
    CHECK(std::fabs(core.state.theta2) <= kPi);
    CHECK(std::isfinite(core.outputs.smooth1));
    CHECK(std::isfinite(core.outputs.discrete1));
}

static void testEnergyConservation() {
    PendulumCore core;
    core.reset(0.1f, 0.2f);
    PendulumCore::Inputs in = baseInputs(0.1f, 0.2f);

    core.process(in, 1.0 / 48000.0);
    const double e0 = computeEnergy(core.state, core.masses);
    for (int i = 0; i < 48000 * 10; i++) {
        core.process(in, 1.0 / 48000.0);
    }
    const double e1 = computeEnergy(core.state, core.masses);
    CHECK(nearRel(e0, e1, 1e-6));
}

static void testChaosBounds() {
    PendulumCore core;
    core.reset(0.2f, 0.3f);
    PendulumCore::Inputs in = baseInputs(0.2f, 0.3f);
    in.chaos = 1.f;

    for (int i = 0; i < 48000 * 10; i++) {
        core.process(in, 1.0 / 48000.0);
    }
    CHECK(std::isfinite(core.state.theta1));
    CHECK(std::isfinite(core.state.theta2));
    CHECK(std::fabs(core.state.theta1) <= kPi);
    CHECK(std::fabs(core.state.theta2) <= kPi);
    CHECK(std::fabs(core.ghost.theta1) <= kPi);
    CHECK(std::fabs(core.ghost.theta2) <= kPi);
}

static void testChaosSettle() {
    PendulumCore core;
    core.reset(0.1f, 0.2f);
    PendulumCore::Inputs in = baseInputs(0.1f, 0.2f);

    for (int i = 0; i < 48000; i++) {
        core.process(in, 1.0 / 48000.0);
    }
    const double before = computeEnergy(core.state, core.masses);

    in.chaos = 1.f;
    for (int i = 0; i < 48000; i++) {
        core.process(in, 1.0 / 48000.0);
    }
    CHECK(core.settle.active() == false);

    in.chaos = 0.f;
    for (int i = 0; i < 48000 * 6; i++) {
        core.process(in, 1.0 / 48000.0);
    }
    const double after = computeEnergy(core.state, core.masses);
    CHECK(!core.settle.active());
    CHECK(near(after, before, 0.05));
}

static void testCvOverride() {
    PendulumCore core;
    core.reset(0.0f, 0.0f);
    PendulumCore::Inputs in = baseInputs(0.0f, 0.0f);
    in.time = 0.f;
    in.timeCv = 1.f;
    in.timeCvActive = true;

    for (int i = 0; i < 48000; i++) {
        core.process(in, 1.0 / 48000.0);
    }
    CHECK(near(core.smoothedTime, 1.f, 1e-4));
}

static void testSkewMasses() {
    PendulumCore core;
    core.reset(0.0f, 0.0f);
    core.smoothedSkew = 0.f;
    PendulumCore::Inputs in = baseInputs(0.0f, 0.0f);
    in.skew = 0.f;
    core.process(in, 1.0 / 48000.0);
    CHECK(near(core.masses.m1, 1.5));
    CHECK(near(core.masses.m2, 0.5));

    core.smoothedSkew = 1.f;
    in.skew = 1.f;
    core.process(in, 1.0 / 48000.0);
    CHECK(near(core.masses.m1, 0.5));
    CHECK(near(core.masses.m2, 1.5));
    CHECK(near(core.masses.m1 + core.masses.m2, 2.0));
}

static void testHoldAndLaunch() {
    PendulumCore core;
    core.reset(0.0f, 0.0f);
    PendulumCore::Inputs in = baseInputs(0.25f, 0.5f);
    in.angleHeld = true;

    core.process(in, 1.0 / 48000.0);
    CHECK(near(core.state.theta1, 0.5 * kPi));
    CHECK(near(core.state.theta2, -kPi));
    CHECK(near(core.state.p1, 0.0));
    CHECK(near(core.state.p2, 0.0));
    CHECK(!core.actions.simulationRunning);

    const PendulumState held = core.state;
    for (int i = 0; i < 48000; i++) {
        core.process(in, 1.0 / 48000.0);
    }
    CHECK(near(core.state.theta1, held.theta1));
    CHECK(near(core.state.theta2, held.theta2));

    in.angleHeld = false;
    core.process(in, 0.09);
    CHECK(core.actions.simulationRunning);
    CHECK(core.state.p1 != 0.0);
    CHECK(core.state.p2 != 0.0);
}

static void testResetClearsTransientState() {
    PendulumCore core;
    core.reset(0.1f, 0.2f);
    PendulumCore::Inputs in = baseInputs(0.1f, 0.2f);
    in.chaos = 1.f;
    for (int i = 0; i < 48000; i++) {
        core.process(in, 1.0 / 48000.0);
    }

    core.reset(0.1f, 0.2f);
    CHECK(!core.settle.active());
    CHECK(!core.chaosWasActive);
    CHECK(near(core.state.p1, IMPULSE_1));
    CHECK(near(core.outputs.smooth1, 0.f));
}

static void testDeterminism() {
    PendulumCore a;
    PendulumCore b;
    a.reset(0.3f, 0.4f);
    b.reset(0.3f, 0.4f);

    PendulumCore::Inputs in = baseInputs(0.3f, 0.4f);
    in.chaos = 0.8f;
    for (int i = 0; i < 1000; i++) {
        a.process(in, 1.0 / 48000.0);
        b.process(in, 1.0 / 48000.0);
    }
    CHECK(a.state.theta1 == b.state.theta1);
    CHECK(a.state.theta2 == b.state.theta2);
    CHECK(a.state.p1 == b.state.p1);
    CHECK(a.state.p2 == b.state.p2);
    CHECK(a.outputs.smooth1 == b.outputs.smooth1);
}


static void testSampleRates() {
    const double rates[] = {44100.0, 48000.0, 96000.0, 192000.0, 384000.0};
    for (int i = 0; i < 5; i++) {
        const double dt = 1.0 / rates[i];
        PendulumCore core;
        core.onSampleRateChange((float)dt);
        core.reset(0.2f, 0.3f);
        PendulumCore::Inputs in = baseInputs(0.2f, 0.3f);
        in.chaos = 0.9f;
        const int samples = (int)(0.1 / dt);
        for (int j = 0; j < samples; j++) {
            core.process(in, dt);
        }
        CHECK(std::isfinite(core.state.theta1));
        CHECK(std::isfinite(core.state.theta2));
        CHECK(std::fabs(core.state.theta1) <= kPi);
        CHECK(std::fabs(core.state.theta2) <= kPi);
    }
}

void testCore() {
    testReset();
    testSingleSample();
    testEnergyConservation();
    testChaosBounds();
    testChaosSettle();
    testCvOverride();
    testSkewMasses();
    testHoldAndLaunch();
    testResetClearsTransientState();
    testDeterminism();
    testSampleRates();
}
