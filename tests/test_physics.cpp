#include "TestHarness.hpp"

#include "dsp/PendulumPhysics.hpp"

#include <cmath>

static PendulumState makeState(double t1, double t2, double p1, double p2) {
    PendulumState s;
    s.theta1 = t1;
    s.theta2 = t2;
    s.p1 = p1;
    s.p2 = p2;
    return s;
}

static MassPair makeMass(double m1, double m2) {
    MassPair m;
    m.m1 = m1;
    m.m2 = m2;
    return m;
}

static void testTimeScale() {
    CHECK(near(computeTimeScale(0.f), 0.2));
    CHECK(near(computeTimeScale(0.25f), 0.6));
    CHECK(near(computeTimeScale(0.5f), 1.0));
    CHECK(near(computeTimeScale(1.f), 7.0));
}

static void testRestState() {
    PendulumState s = makeState(0.0, 0.0, 0.0, 0.0);
    MassPair m = makeMass(1.0, 1.0);

    PendulumDerivatives d = computePendulumDerivatives(s, m, 1.0);
    CHECK(near(d.dTheta1, 0.0));
    CHECK(near(d.dTheta2, 0.0));
    CHECK(near(d.dP1, 0.0));
    CHECK(near(d.dP2, 0.0));

    AngularVelocity w = computeAngularVelocity(s, m);
    CHECK(near(w.w1, 0.0));
    CHECK(near(w.w2, 0.0));

    CHECK(near(computeEnergy(s, m), -3.0 * GRAVITY));
}

static void testGravityTorques() {
    PendulumState s = makeState(kPi / 2.0, kPi / 2.0, 0.0, 0.0);
    MassPair m = makeMass(1.0, 1.0);
    PendulumDerivatives d = computePendulumDerivatives(s, m, 1.0);

    CHECK(near(d.dTheta1, 0.0));
    CHECK(near(d.dTheta2, 0.0));
    CHECK(near(d.dP1, -2.0 * GRAVITY));
    CHECK(near(d.dP2, -GRAVITY));
}

static void testEnergyAtTop() {
    PendulumState s = makeState(kPi, kPi, 0.0, 0.0);
    MassPair m = makeMass(1.0, 1.0);
    CHECK(near(computeEnergy(s, m), 3.0 * GRAVITY));
}

static void testCoupling() {
    PendulumState s = makeState(0.0, 0.0, 1.0, 0.0);
    MassPair m = makeMass(1.0, 1.0);
    AngularVelocity w = computeAngularVelocity(s, m);
    CHECK(near(w.w1, 1.0));
    CHECK(near(w.w2, -1.0));
}

static void testEnergyConservation() {
    PendulumState s = makeState(1.2, -0.6, 0.4, -0.3);
    MassPair m = makeMass(1.1, 0.9);
    const double e0 = computeEnergy(s, m);

    const double dt = 1.0 / 48000.0;
    for (int i = 0; i < 48000; i++) {
        rk4Step(s, dt, m, 1.0);
    }
    const double e1 = computeEnergy(s, m);
    CHECK(nearRel(e0, e1, 1e-7));
    CHECK(std::isfinite(e1));
}

static void testPeriodicity() {
    PendulumState a = makeState(0.7, -1.2, 0.3, -0.2);
    PendulumState b = makeState(0.7 + 2.0 * kPi, -1.2 - 2.0 * kPi, 0.3, -0.2);
    MassPair m = makeMass(1.0, 1.0);

    CHECK(nearRel(computeEnergy(a, m), computeEnergy(b, m), 1e-12));

    PendulumDerivatives da = computePendulumDerivatives(a, m, 1.0);
    PendulumDerivatives db = computePendulumDerivatives(b, m, 1.0);
    CHECK(nearRel(da.dTheta1, db.dTheta1, 1e-12));
    CHECK(nearRel(da.dTheta2, db.dTheta2, 1e-12));
    CHECK(nearRel(da.dP1, db.dP1, 1e-12));
    CHECK(nearRel(da.dP2, db.dP2, 1e-12));
}

static void testMassPreservation() {
    PendulumState s = makeState(0.7, -0.4, 0.3, -0.2);
    MassPair oldMasses = makeMass(1.0, 1.0);
    MassPair newMasses = makeMass(0.6, 1.4);
    const AngularVelocity before = computeAngularVelocity(s, oldMasses);

    CHECK(preserveAngularVelocity(s, oldMasses, newMasses));
    const AngularVelocity after = computeAngularVelocity(s, newMasses);
    CHECK(near(before.w1, after.w1));
    CHECK(near(before.w2, after.w2));
}

static void testMassNoChange() {
    PendulumState s = makeState(0.7, -0.4, 0.3, -0.2);
    const PendulumState original = s;
    CHECK(!preserveAngularVelocity(s, makeMass(1.0, 1.0), makeMass(1.0, 1.0)));
    CHECK(near(s.theta1, original.theta1));
    CHECK(near(s.theta2, original.theta2));
    CHECK(near(s.p1, original.p1));
    CHECK(near(s.p2, original.p2));
}

static void testStability() {
    PendulumState s = makeState(NAN, INFINITY, 1.0, 2.0);
    checkPendulumStability(s);
    CHECK(near(s.theta1, 0.0));
    CHECK(near(s.theta2, 0.0));
    CHECK(near(s.p1, 0.0));
    CHECK(near(s.p2, 0.0));
}

static void testDeterminism() {
    PendulumState a = makeState(0.4, -0.8, 0.2, -0.1);
    PendulumState b = a;
    MassPair m = makeMass(1.0, 1.0);
    for (int i = 0; i < 1000; i++) {
        rk4Step(a, 1.0 / 48000.0, m, 1.0);
        rk4Step(b, 1.0 / 48000.0, m, 1.0);
    }
    CHECK(a.theta1 == b.theta1);
    CHECK(a.theta2 == b.theta2);
    CHECK(a.p1 == b.p1);
    CHECK(a.p2 == b.p2);
}

void testPhysics() {
    testTimeScale();
    testRestState();
    testGravityTorques();
    testEnergyAtTop();
    testCoupling();
    testEnergyConservation();
    testPeriodicity();
    testMassPreservation();
    testMassNoChange();
    testStability();
    testDeterminism();
}
