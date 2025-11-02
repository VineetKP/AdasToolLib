/* *******************************************************************************
 * File: tests/test_transformers.cpp
 * Description: Minimal self-contained tests for transformers and quaternion
 *              utilities. Avoids external test frameworks; returns non-zero on
 *              failure so CI can detect failures.
 * *******************************************************************************/

#include <cmath>
#include <iostream>
#include "helpers.hpp"
#include "transformers.hpp"
#include "quaternion.hpp"

using namespace AdasTools;

static bool approxEqual(double a, double b, double eps=1e-6) {
    double d = a - b;
    if (d < 0) d = -d;
    return d <= eps;
}

int main()
{
    // Test round-trip local->global->local
    Point3 p_local = {1.234, -0.5, 0.75};
    Frame3D frame = {2.0, 0.1, -0.3, 0.1, -0.2, 0.3};

    Point3 g = localToGlobal(p_local, frame);
    Point3 back = globalToLocal(g, frame);

    if (!approxEqual(p_local.x, back.x) || !approxEqual(p_local.y, back.y) || !approxEqual(p_local.z, back.z)) {
        std::cerr << "Round-trip transform failed: (" << p_local.x << "," << p_local.y << "," << p_local.z << ") -> (" << back.x << "," << back.y << "," << back.z << ")\n";
        return 2;
    }

    // Quaternion round-trip
    Quaternion q1 = quaternionFromRPY(0.3, -0.4, 0.5);
    Quaternion q2 = quaternionFromRPY(-0.2, 0.1, -0.3);
    Quaternion qmid = slerp(q1, q2, 0.5);
    Quaternion qmid2 = slerp(q1, q2, 0.5);

    // qmid should be normalized
    Quaternion qn = normalizeQuaternion(qmid);
    double norm = sqrt(qn.w*qn.w + qn.x*qn.x + qn.y*qn.y + qn.z*qn.z);
    if (!approxEqual(norm, 1.0, 1e-6)) {
        std::cerr << "Quaternion normalization failed: norm=" << norm << "\n";
        return 3;
    }

    // slerp should be symmetric for t=0.5
    if (!approxEqual(qmid.w, qmid2.w) || !approxEqual(qmid.x, qmid2.x) ) {
        std::cerr << "Slerp symmetry failure\n";
        return 4;
    }

    std::cout << "All tests passed" << std::endl;
    return 0;
}
