#include "TestHarness.hpp"

#include "dsp/Smoothing.hpp"

#include <cmath>

static void testOnePole() {
    CHECK(near(onePoleAlpha(0.f, 0.005f), 0.f));
    const float alpha = onePoleAlpha(1.f / 48000.f, CV_SMOOTH_TIME);
    CHECK(alpha > 0.f && alpha < 1.f);
    CHECK(near(alpha, 1.f - std::exp(-(1.f / 48000.f) / CV_SMOOTH_TIME), 1e-12));
}

static void testApproach() {
    CHECK(near(approach(0.f, 1.f, 0.5f), 0.5f));
    CHECK(near(approach(1.f, 0.f, 0.25f), 0.75f));
    CHECK(near(approach(0.2f, 0.2f, 0.5f), 0.2f));
}

void testSmoothing() {
    testOnePole();
    testApproach();
}
