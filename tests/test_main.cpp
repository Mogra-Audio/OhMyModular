#include "TestHarness.hpp"

void testExpanderMath();
void testPhysics();
void testChaos();
void testSettle();
void testOutputs();
void testUserActions();
void testSmoothing();
void testCore();

int main() {
    testExpanderMath();
    testPhysics();
    testChaos();
    testSettle();
    testOutputs();
    testUserActions();
    testSmoothing();
    testCore();

    if (testFailures()) {
        std::printf("%d check(s) failed\n", testFailures());
        return 1;
    }
    std::printf("all checks passed\n");
    return 0;
}
