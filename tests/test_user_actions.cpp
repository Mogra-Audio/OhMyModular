#include "TestHarness.hpp"

#include "dsp/UserActions.hpp"

#include <cmath>

static PendulumState makeState(double t1, double t2, double p1, double p2) {
    PendulumState s;
    s.theta1 = t1;
    s.theta2 = t2;
    s.p1 = p1;
    s.p2 = p2;
    return s;
}

static void testInitialize() {
    PendulumState s;
    UserActionState a;
    initializeFromAngles(s, a, 0.25f, 0.5f);

    CHECK(near(s.theta1, 0.5 * kPi));
    CHECK(near(s.theta2, kPi));
    CHECK(near(s.p1, IMPULSE_1));
    CHECK(near(s.p2, IMPULSE_2));
    CHECK(a.simulationRunning);
    CHECK(near(a.stableTime, 0.0));
}

static void testRetrigger() {
    PendulumState s = makeState(1.0, 2.0, 3.0, 4.0);
    UserActionState a;
    const bool explicitSet = applyUserActions(s, a, 0.1f, 0.2f, true, false, 1.0 / 48000.0);

    CHECK(explicitSet);
    CHECK(near(s.theta1, 0.2 * kPi, 1e-6));
    CHECK(near(s.theta2, 0.4 * kPi, 1e-6));
    CHECK(near(s.p1, IMPULSE_1));
    CHECK(near(s.p2, IMPULSE_2));
    CHECK(a.simulationRunning);
    CHECK(near(a.stableTime, 0.0));
}

static void testAngleChange() {
    PendulumState s;
    UserActionState a;
    a.lastAngle1 = 0.f;
    a.lastAngle2 = 0.f;

    CHECK(applyUserActions(s, a, 0.1f, 0.2f, false, false, 1.0 / 48000.0));
    CHECK(near(s.theta1, 0.2 * kPi, 1e-6));
    CHECK(near(s.theta2, 0.4 * kPi, 1e-6));
    CHECK(near(s.p1, IMPULSE_1));
    CHECK(near(s.p2, IMPULSE_2));
    CHECK(near(a.lastAngle1, 0.1f));
    CHECK(near(a.lastAngle2, 0.2f));

    const PendulumState before = s;
    CHECK(!applyUserActions(s, a, 0.1f, 0.2f, false, false, 1.0 / 48000.0));
    CHECK(s.theta1 == before.theta1);
    CHECK(s.theta2 == before.theta2);
    CHECK(s.p1 == before.p1);
    CHECK(s.p2 == before.p2);
}

static void testHoldAndLaunch() {
    PendulumState s = makeState(1.0, 2.0, 3.0, 4.0);
    UserActionState a;

    CHECK(applyUserActions(s, a, 0.3f, 0.4f, false, true, 1.0 / 48000.0));
    CHECK(near(s.theta1, 0.6 * kPi, 1e-6));
    CHECK(near(s.theta2, 0.8 * kPi, 1e-6));
    CHECK(near(s.p1, 0.0));
    CHECK(near(s.p2, 0.0));
    CHECK(!a.simulationRunning);
    CHECK(near(a.stableTime, 0.0));

    // Still held: no countdown.
    CHECK(applyUserActions(s, a, 0.3f, 0.4f, false, true, 1.0 / 48000.0));
    CHECK(!a.simulationRunning);

    // Released: countdown, then launch exactly once.
    CHECK(!applyUserActions(s, a, 0.3f, 0.4f, false, false, 0.08));
    CHECK(!a.simulationRunning);
    CHECK(near(a.stableTime, 0.08));

    CHECK(!applyUserActions(s, a, 0.3f, 0.4f, false, false, 0.01));
    CHECK(a.simulationRunning);
    CHECK(near(a.stableTime, 0.0));
    CHECK(near(s.p1, IMPULSE_1));
    CHECK(near(s.p2, IMPULSE_2));
}

static void testHoldOverridesRetrigger() {
    PendulumState s;
    UserActionState a;
    CHECK(applyUserActions(s, a, 0.2f, 0.3f, true, true, 1.0 / 48000.0));
    CHECK(!a.simulationRunning);
    CHECK(near(s.p1, 0.0));
    CHECK(near(s.p2, 0.0));
}

void testUserActions() {
    testInitialize();
    testRetrigger();
    testAngleChange();
    testHoldAndLaunch();
    testHoldOverridesRetrigger();
}
