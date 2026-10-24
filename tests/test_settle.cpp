#include "TestHarness.hpp"

#include "dsp/Settle.hpp"

#include <cmath>

static PendulumState makeState(double t1, double t2, double p1, double p2) {
    PendulumState s;
    s.theta1 = t1;
    s.theta2 = t2;
    s.p1 = p1;
    s.p2 = p2;
    return s;
}

static double runSettle(PendulumState& s, double target, double seconds) {
    EnergySettle settle;
    settle.setTarget(target);
    settle.start();

    MassPair masses;
    const double dt = 1.0 / 48000.0;
    const int samples = (int)(seconds / dt);
    for (int i = 0; i < samples; i++) {
        rk4Step(s, dt, masses, 1.0);
        settle.process(s, dt, masses);
    }
    return computeEnergy(s, masses);
}

static void testTargetReachedImmediately() {
    PendulumState s = makeState(0.3, -0.2, 0.1, -0.05);
    MassPair m;
    const double energy = computeEnergy(s, m);
    EnergySettle settle;
    settle.setTarget(energy);
    settle.start();
    settle.process(s, 1.0 / 48000.0, m);
    CHECK(!settle.active());
    CHECK(near(computeEnergy(s, m), energy));
}

static void testAddsEnergy() {
    PendulumState s = makeState(0.0, 0.0, 0.2, 0.1);
    MassPair m;
    const double target = computeEnergy(s, m) + 2.0;
    const double result = runSettle(s, target, 6.0);
    CHECK(near(result, target, 0.05));
}

static void testRemovesEnergy() {
    PendulumState s = makeState(1.0, -1.0, 0.5, 0.2);
    MassPair m;
    const double target = computeEnergy(s, m) - 5.0;
    const double result = runSettle(s, target, 6.0);
    CHECK(near(result, target, 0.05));
}

static void testTopNudge() {
    PendulumState s = makeState(kPi - 0.001, kPi - 0.001, 0.0, 0.0);
    const double result = runSettle(s, 0.0, 6.0);
    CHECK(near(result, 0.0, 0.05));
}

static void testTimeout() {
    PendulumState s = makeState(0.0, 0.0, 0.0, 0.0);
    MassPair m;
    EnergySettle settle;
    settle.setTarget(1e6);
    settle.start();
    for (int i = 0; i < 48000 * 7; i++) {
        settle.process(s, 1.0 / 48000.0, m);
    }
    CHECK(!settle.active());
    CHECK(computeEnergy(s, m) < 1e6);
}

static void testCancel() {
    PendulumState s = makeState(0.3, -0.2, 0.1, -0.05);
    MassPair m;
    EnergySettle settle;
    settle.setTarget(computeEnergy(s, m) + 2.0);
    settle.start();
    CHECK(settle.active());
    settle.cancel();
    CHECK(!settle.active());

    const PendulumState before = s;
    settle.process(s, 1.0 / 48000.0, m);
    CHECK(s.theta1 == before.theta1);
    CHECK(s.theta2 == before.theta2);
    CHECK(s.p1 == before.p1);
    CHECK(s.p2 == before.p2);
}


static void testConvergenceSweep() {
    const double cases[][4] = {
        {0.0, 0.0, 0.2, 0.1},
        {1.0, -1.0, 0.5, 0.2},
        {2.0, -2.0, 0.4, -0.3},
        {-1.5, 0.8, 0.3, 0.2},
    };
    for (int i = 0; i < 4; i++) {
        PendulumState s = makeState(cases[i][0], cases[i][1], cases[i][2], cases[i][3]);
        MassPair m;
        const double target = std::max(computeEnergy(s, m) - 5.0, -28.0);
        const double result = runSettle(s, target, 6.0);
        CHECK(near(result, target, 0.1));
    }
}

void testSettle() {
    testTargetReachedImmediately();
    testAddsEnergy();
    testRemovesEnergy();
    testTopNudge();
    testTimeout();
    testCancel();
    testConvergenceSweep();
}
