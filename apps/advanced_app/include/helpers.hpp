/* *******************************************************************************
 * File: apps/advanced_app/include/helpers.hpp
 * Minimal helpers copied from the main library so this app is standalone.
 * *******************************************************************************/

#pragma once

namespace AdasTools {

struct Point3 {
    double x;
    double y;
    double z;
};

struct Frame3D {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

struct Pose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

inline Pose poseRadiansToDegrees(const Pose &p)
{
    const double RAD2DEG = 180.0 / 3.14159265358979323846;
    Pose out;
    out.x = p.x; out.y = p.y; out.z = p.z;
    out.roll = p.roll * RAD2DEG;
    out.pitch = p.pitch * RAD2DEG;
    out.yaw = p.yaw * RAD2DEG;
    return out;
}

} // namespace AdasTools
