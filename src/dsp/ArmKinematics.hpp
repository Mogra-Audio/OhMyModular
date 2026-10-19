#pragma once

#include <cmath>

// Arm lengths shared by the physics and the expander's coordinate outputs.
static constexpr double ARM_L1 = 1.0;
static constexpr double ARM_L2 = 1.0;

// One uniform scale keeps the XY picture geometrically correct: the elbow
// reaches +/-2.5 V and the tip +/-5 V.
static constexpr double EXPANDER_VOLTS_PER_METER = 2.5;

struct ArmPoints {
    double x1, y1;  // joint between the two arms
    double x2, y2;  // tip of the second arm
};

// Cartesian positions in meters. Y points up, matching an XY scope.
inline ArmPoints computeArmPoints(double theta1, double theta2) {
    ArmPoints p;
    p.x1 = ARM_L1 * std::sin(theta1);
    p.y1 = -ARM_L1 * std::cos(theta1);
    p.x2 = p.x1 + ARM_L2 * std::sin(theta2);
    p.y2 = p.y1 - ARM_L2 * std::cos(theta2);
    return p;
}

inline float coordinateToVolts(double coordinate) {
    return (float)(coordinate * EXPANDER_VOLTS_PER_METER);
}
