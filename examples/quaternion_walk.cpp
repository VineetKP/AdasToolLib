/* *******************************************************************************
 * File: examples/quaternion_walk.cpp
 * Description: Interpolate between two sensor orientations using slerp and rotate a point
 * *******************************************************************************/

#include <iostream>
#include "helpers.hpp"
#include "quaternion.hpp"

using namespace AdasTools;

int main()
{
    Point3 p = {1.0, 0.0, 0.0};
    Quaternion a = quaternionFromRPY(0.0, 0.0, 0.0);
    Quaternion b = quaternionFromRPY(0.0, 0.5, 1.0);
    a = normalizeQuaternion(a);
    b = normalizeQuaternion(b);

    for (int i=0;i<=10;++i) {
        double t = i/10.0;
        Quaternion q = slerp(a,b,t);
    Point3 r = rotateByQuaternion(q, p);
    std::cout << "t="<<t<<" -> ("<<r.x<<","<<r.y<<","<<r.z<<")\n";
    }

    return 0;
}
