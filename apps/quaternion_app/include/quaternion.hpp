#pragma once
#include "helpers.hpp"
namespace AdasTools {
struct Quaternion;
struct Point3;
Quaternion quaternionFromRPY(double roll, double pitch, double yaw);
Quaternion multiplyQuaternion(const Quaternion &q1, const Quaternion &q2);
Quaternion normalizeQuaternion(const Quaternion &q);
Point3 rotateByQuaternion(const Quaternion &q, const Point3 &p);
Pose rotateByQuaternion(const Quaternion &q, const Pose &p);
Quaternion slerp(const Quaternion &a, const Quaternion &b, double t);
}
