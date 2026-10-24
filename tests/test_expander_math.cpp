// Rack-free checks for the expander's geometry and CV contract.
#include "TestHarness.hpp"

#include "dsp/Angle.hpp"
#include "dsp/ArmKinematics.hpp"
#include "ExpanderMessage.hpp"
#include "Theme.hpp"

#include <cstring>

static void testArmPoints() {
    ArmPoints p = computeArmPoints(0.0, 0.0);
    CHECK(near(p.x1, 0.0) && near(p.y1, -1.0));
    CHECK(near(p.x2, 0.0) && near(p.y2, -2.0));

    p = computeArmPoints(M_PI / 2.0, 0.0);
    CHECK(near(p.x1, 1.0) && near(p.y1, 0.0));
    CHECK(near(p.x2, 1.0) && near(p.y2, -1.0));

    p = computeArmPoints(M_PI, 0.0);
    CHECK(near(p.x1, 0.0) && near(p.y1, 1.0));
    CHECK(near(p.x2, 0.0) && near(p.y2, 0.0));

    p = computeArmPoints(M_PI / 2.0, -M_PI / 2.0);
    CHECK(near(p.x1, 1.0) && near(p.y1, 0.0));
    CHECK(near(p.x2, 0.0) && near(p.y2, 0.0));
}

static void testVoltageScale() {
    CHECK(near(coordinateToVolts(-2.0), -5.0));
    CHECK(near(coordinateToVolts(-1.0), -2.5));
    CHECK(near(coordinateToVolts(0.0), 0.0));
    CHECK(near(coordinateToVolts(1.0), 2.5));
    CHECK(near(coordinateToVolts(2.0), 5.0));
}

static void testCvMapping() {
    CHECK(near(cvToUnit(-2.f), 0.f));
    CHECK(near(cvToUnit(0.f), 0.f));
    CHECK(near(cvToUnit(2.5f), 0.25f));
    CHECK(near(cvToUnit(5.f), 0.5f));
    CHECK(near(cvToUnit(10.f), 1.f));
    CHECK(near(cvToUnit(12.f), 1.f));
}

static void testCoordinateBounds() {
    for (int i = 0; i <= 400; i++) {
        for (int j = 0; j <= 400; j++) {
            double t1 = -4.0 * M_PI + 8.0 * M_PI * i / 400.0;
            double t2 = -4.0 * M_PI + 8.0 * M_PI * j / 400.0;
            ArmPoints p = computeArmPoints(t1, t2);
            CHECK(std::fabs(p.x1) <= 1.0 + 1e-12 && std::fabs(p.y1) <= 1.0 + 1e-12);
            CHECK(std::fabs(p.x2) <= 2.0 + 1e-12 && std::fabs(p.y2) <= 2.0 + 1e-12);
            CHECK(std::fabs(coordinateToVolts(p.x2)) <= 5.0 + 1e-6);
        }
    }
}

static void testAngleWrap() {
    CHECK(near(normalizeAngle(0.0), 0.0));
    CHECK(near(normalizeAngle(2.0 * M_PI), 0.0));
    CHECK(near(normalizeAngle(-2.0 * M_PI), 0.0));
    CHECK(near(normalizeAngle(3.0 * M_PI), -M_PI));
    CHECK(near(normalizeAngle(1000.0 * M_PI), 0.0));

    for (int i = -8; i <= 8; i++) {
        CHECK(near(normalizeAngle(0.37 + i * 2.0 * M_PI), normalizeAngle(0.37)));
    }
    for (int i = -1000; i <= 1000; i++) {
        double a = normalizeAngle(i * 0.37);
        CHECK(a >= -M_PI && a < M_PI);
    }

    CHECK(near(wrapAngle(0.5), 0.5));
    CHECK(near(wrapAngle(3.0 * M_PI), -M_PI));
    CHECK(near(wrapAngle(2.0 * M_PI), 0.0));

    ArmPoints a = computeArmPoints(0.7, -1.2);
    ArmPoints b = computeArmPoints(normalizeAngle(0.7 + 10.0 * M_PI),
                                   normalizeAngle(-1.2 + 10.0 * M_PI));
    CHECK(near(a.x1, b.x1) && near(a.y1, b.y1));
    CHECK(near(a.x2, b.x2) && near(a.y2, b.y2));
}

static void testThemeTable() {
    CHECK(THEME_COUNT == 5);
    for (int i = 0; i < THEME_COUNT; i++) {
        CHECK(THEMES[i].panel && THEMES[i].panel[0]);
        CHECK(THEMES[i].expanderPanel && THEMES[i].expanderPanel[0]);
        CHECK(THEMES[i].smallKnob && THEMES[i].smallKnob[0]);
        CHECK(THEMES[i].hugeKnob && THEMES[i].hugeKnob[0]);
        CHECK(THEMES[i].input && THEMES[i].input[0]);
        CHECK(THEMES[i].output && THEMES[i].output[0]);
    }
    for (int i = 0; i < THEME_COUNT; i++) {
        for (int j = i + 1; j < THEME_COUNT; j++) {
            CHECK(std::strcmp(THEMES[i].expanderPanel, THEMES[j].expanderPanel) != 0);
        }
    }
}

void testExpanderMath() {
    testArmPoints();
    testVoltageScale();
    testCvMapping();
    testCoordinateBounds();
    testAngleWrap();
    testThemeTable();
}
