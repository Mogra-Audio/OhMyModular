#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

inline int& testFailures() {
    static int failures = 0;
    return failures;
}

#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++testFailures(); \
        } \
    } while (0)

inline bool near(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) <= eps;
}

inline bool nearRel(double a, double b, double eps = 1e-9) {
    const double scale = std::max(1.0, std::max(std::fabs(a), std::fabs(b)));
    return std::fabs(a - b) <= eps * scale;
}
