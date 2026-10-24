#include "TestHarness.hpp"

#include "dsp/Outputs.hpp"

#include <cmath>

static void testSmoothVolts() {
    CHECK(near(smoothVolts(0.0), 0.0));
    CHECK(near(smoothVolts(kPi), 5.0));
    CHECK(near(smoothVolts(-kPi), -5.0));
    CHECK(near(smoothVolts(10.0), 5.0));
    CHECK(near(smoothVolts(-10.0), -5.0));
}

static void testDiscreteVolts() {
    CHECK(near(discreteVolts(0.0), 0.0));
    CHECK(near(discreteVolts(kPi / 12.0), 5.0 / 12.0, 1e-6));
    CHECK(near(discreteVolts(-kPi / 12.0), -5.0 / 12.0, 1e-6));
    CHECK(near(discreteVolts(kPi / 6.0), 10.0 / 12.0, 1e-6));
    CHECK(near(discreteVolts(kPi / 2.0), 2.5, 1e-6));
}

static void testSlewToward() {
    CHECK(near(slewToward(0.f, 5.f, 0.5f), 0.5f));
    CHECK(near(slewToward(0.f, -5.f, 0.5f), -0.5f));
    CHECK(near(slewToward(0.1f, 0.2f, 0.01f), 0.11f));
    CHECK(near(slewToward(0.1f, 0.2f, 1.f), 0.2f));
}

static void testOutputBuffers() {
    PendulumOutputs outputs;
    PendulumState s;
    s.theta1 = kPi / 2.0;
    s.theta2 = -kPi / 2.0;

    outputs.process(s, 1.0);
    CHECK(near(outputs.smooth1, 2.5, 1e-6));
    CHECK(near(outputs.discrete1, 2.5, 1e-6));
    CHECK(near(outputs.discrete2, -2.5, 1e-6));
    CHECK(near(outputs.smooth2, -2.5, 1e-6));

    outputs.reset();
    CHECK(near(outputs.smooth1, 0.f));
    CHECK(near(outputs.discrete1, 0.f));
    CHECK(near(outputs.discrete2, 0.f));
    CHECK(near(outputs.smooth2, 0.f));
}

static void testSlewLimit() {
    PendulumOutputs outputs;
    PendulumState s;
    s.theta1 = kPi / 2.0;
    s.theta2 = 0.0;

    // 500 V/s at 1 ms => 0.5 V per sample.
    outputs.process(s, 0.001);
    CHECK(near(outputs.smooth1, 0.5, 1e-6));
    CHECK(near(outputs.discrete1, 0.5, 1e-6));

    for (int i = 0; i < 9; i++) {
        outputs.process(s, 0.001);
    }
    CHECK(near(outputs.smooth1, 2.5, 1e-6));
}

void testOutputs() {
    testSmoothVolts();
    testDiscreteVolts();
    testSlewToward();
    testOutputBuffers();
    testSlewLimit();
}
