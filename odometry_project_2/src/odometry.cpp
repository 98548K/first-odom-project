#include "odometry.h"
#include "vex.h"

// Global Position on Field
double globalXPos = 0;
double globalYPos = 0;

// Current Angle in RADIANS
double absoluteOrientation = 0;

int positionTracking(void) {
    while (true) {
        // Odometry Math

        vex::wait(20, vex::timeUnits::msec);
    }

    return 1;
}