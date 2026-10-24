#include "TestHarness.hpp"

#include "dsp/Chaos.hpp"

#include <cmath>

static PendulumState makeGhost(double t1, double t2, double p1, double p2) {
    PendulumState g;
    g.theta1 = t1;
    g.theta2 = t2;
    g.p1 = p1;
    g.p2 = p2;
    return g;
}

static void testChaosAmount() {
    CHECK(near(chaosAmount(0.f, 0.f), 0.f));
    CHECK(near(chaosAmount(0.5f, 0.f), 0.5f));
    CHECK(near(chaosAmount(0.5f, 2.5f), 1.0f));
    CHECK(near(chaosAmount(0.2f, -1.f), 0.f));
    CHECK(near(chaosAmount(1.f, 10.f), 1.f));
    CHECK(near(chaosAmount(0.f, -10.f), 0.f));
}

static void testGhostSubsteps() {
    CHECK(ghostSubsteps(0.0) == 1);
    CHECK(ghostSubsteps(0.1) == 1);
    CHECK(ghostSubsteps(0.5) == 3);
    CHECK(ghostSubsteps(1.0) == GHOST_SUBSTEPS_MAX);
}

static void testDeadZone() {
    CHECK(!isChaosActive(0.0));
    CHECK(!isChaosActive(CHAOS_DEAD));
    CHECK(isChaosActive(CHAOS_DEAD + 1e-9));

    PendulumState s;
    s.theta1 = 0.3;
    s.theta2 = -0.4;
    s.p1 = 0.5;
    s.p2 = -0.6;
    const PendulumState original = s;
    PendulumState ghost = makeGhost(kPi / 2.0, 0.0, 1.0, 1.0);

    applyChaos(s, ghost, CHAOS_DEAD, 1.0 / 48000.0);
    CHECK(s.theta1 == original.theta1);
    CHECK(s.theta2 == original.theta2);
    CHECK(s.p1 == original.p1);
    CHECK(s.p2 == original.p2);
}

static void testSampleRateNormalization() {
    PendulumState ghost = makeGhost(kPi / 2.0, 0.0, 0.0, 0.0);
    PendulumState a;
    PendulumState b;
    applyChaos(a, ghost, 1.0, 1.0 / 48000.0);
    applyChaos(b, ghost, 1.0, 2.0 / 48000.0);
    CHECK(a.theta1 > 0.0);
    CHECK(near(b.theta1, 2.0 * a.theta1, 1e-15));
}

static void testMomentumClamp() {
    PendulumState s;
    s.p1 = 1e6;
    s.p2 = -1e6;
    PendulumState ghost = makeGhost(0.0, 0.0, 0.0, 0.0);
    applyChaos(s, ghost, 1.0, 1.0 / 48000.0);
    CHECK(std::fabs(s.p1) <= P_MAX);
    CHECK(std::fabs(s.p2) <= P_MAX);
}

static void testInjectionDirection() {
    PendulumState s;
    PendulumState ghost = makeGhost(kPi / 2.0, -kPi / 2.0, 1.0, -1.0);
    applyChaos(s, ghost, 1.0, 1.0 / 48000.0);
    CHECK(s.theta1 > 0.0);
    CHECK(s.theta2 < 0.0);
    CHECK(s.p1 > 0.0);
    CHECK(s.p2 < 0.0);
}

static void testGhostStability() {
    PendulumState g = makeGhost(NAN, INFINITY, 1.0, 2.0);
    checkGhostStability(g);
    CHECK(near(g.theta1, GHOST_THETA1));
    CHECK(near(g.theta2, GHOST_THETA2));
    CHECK(near(g.p1, 0.0));
    CHECK(near(g.p2, 0.0));
}

static void testGhostStepBounded() {
    PendulumState ghost = makeGhost(GHOST_THETA1, GHOST_THETA2, 0.0, 0.0);
    MassPair m;
    for (int i = 0; i < 20000; i++) {
        stepGhost(ghost, 1.0 / 48000.0, m, GHOST_SUBSTEPS_MAX);
    }
    CHECK(std::isfinite(ghost.theta1));
    CHECK(std::isfinite(ghost.theta2));
    CHECK(std::fabs(ghost.theta1) < kPi);
    CHECK(std::fabs(ghost.theta2) < kPi);
}

void testChaos() {
    testChaosAmount();
    testGhostSubsteps();
    testDeadZone();
    testSampleRateNormalization();
    testMomentumClamp();
    testInjectionDirection();
    testGhostStability();
    testGhostStepBounded();
}
