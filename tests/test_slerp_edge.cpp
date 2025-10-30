/* *******************************************************************************
 * File: tests/test_slerp_edge.cpp
 * Description: Test slerp behavior when quaternions are nearly opposite.
 * *******************************************************************************/

#include <iostream>
#include "quaternion.hpp"
#include <math.h>

using namespace AdasTools;

static bool approx(double a, double b, double eps=1e-6) { double d=a-b; if (d<0)d=-d; return d<=eps; }

int main() {
    // q and -q represent the same rotation; slerp should handle the dot<0 case
    Quaternion q = quaternionFromRPY(0.1, 0.2, 0.3);
    Quaternion nq = q; nq.w = -nq.w; nq.x = -nq.x; nq.y = -nq.y; nq.z = -nq.z;
    q = normalizeQuaternion(q);
    nq = normalizeQuaternion(nq);

    Quaternion s = slerp(q, nq, 0.5);
    // s should be a valid normalized quaternion
    double norm = sqrt(s.w*s.w + s.x*s.x + s.y*s.y + s.z*s.z);
    if (!approx(norm, 1.0, 1e-6)) { std::cerr << "slerp edge normalization failed: "<<norm<<"\n"; return 1; }
    std::cout << "slerp edge test OK\n";
    return 0;
}
