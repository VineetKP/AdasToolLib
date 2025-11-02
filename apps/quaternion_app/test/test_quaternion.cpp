#include <cmath>
#include <cassert>
#include "../include/quaternion.hpp"

int main() {
    using namespace std;
    using namespace AdasTools;
    // endpoints
    Quaternion a = quaternionFromRPY(0,0,0);
    Quaternion b = quaternionFromRPY(0,0,M_PI/2.0);
    Quaternion s0 = slerp(a,b,0.0);
    Quaternion s1 = slerp(a,b,1.0);
    // normalize checks
    auto norm = [](const Quaternion &q){ return sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z); };
    assert(fabs(norm(s0)-1.0) < 1e-9);
    assert(fabs(norm(s1)-1.0) < 1e-9);
    // endpoints equal to inputs (up to sign for quaternions)
    // compare absolute dot product near 1
    auto dot = [](const Quaternion &p, const Quaternion &q){ return fabs(p.w*q.w + p.x*q.x + p.y*q.y + p.z*q.z); };
    assert(dot(s0,a) > 0.999999999);
    assert(dot(s1,b) > 0.999999999);
    return 0;
}
