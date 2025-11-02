/* *******************************************************************************
 * File: tests/test_gimbal.cpp
 * Description: Test transformers behavior near gimbal-lock (pitch ~= +-90 deg).
 * *******************************************************************************/

#include <iostream>
#include "transformers.hpp"
#include <math.h>

using namespace AdasTools;

static bool approx(double a, double b, double eps=1e-6) { double d=a-b; if (d<0)d=-d; return d<=eps; }

int main() {
    Point3 p = {1.0, 0.0, 0.0};
    // Pitch near +90 degrees
    Frame3D f = {0.0,0.0,0.0, 0.0, M_PI_2 - 1e-6, 0.0};
    Point3 g = localToGlobal(p, f);
    Point3 back = globalToLocal(g, f);
    if (!approx(p.x, back.x, 1e-5) || !approx(p.y, back.y, 1e-5) || !approx(p.z, back.z, 1e-5)) {
        std::cerr << "gimbal round-trip failed\n";
        return 1;
    }
    std::cout << "gimbal test OK\n";
    return 0;
}
