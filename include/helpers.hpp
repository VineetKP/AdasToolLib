/* *******************************************************************************
 * File: include/helpers.hpp
 * Project: ADAS Tools Library (adas_tools)
 * Description: Small helper definitions used across the library. Keeps only
 *              plain-old-data (POD) types and simple utilities — intentionally
 *              avoids use of the STL so it can be used in constrained builds.
 * Author: VineetKP
 * Created: 2025-10-31
 * *******************************************************************************/

#pragma once

namespace AdasTools {

/**
 * @brief Simple 3D point type (POD) representing coordinates in meters.
 */
struct Point3 {
    double x; /**< X coordinate in meters */
    double y; /**< Y coordinate in meters */
    double z; /**< Z coordinate in meters */
};

/**
 * @brief Simple 3D frame: origin (x,y,z) and roll/pitch/yaw in radians.
 *
 * roll  : rotation about X-axis
 * pitch : rotation about Y-axis
 * yaw   : rotation about Z-axis
 */
struct Frame3D {
    double x;   /**< origin x (meters) */
    double y;   /**< origin y (meters) */
    double z;   /**< origin z (meters) */
    double roll;  /**< rotation about X axis in radians */
    double pitch; /**< rotation about Y axis in radians */
    double yaw;   /**< rotation about Z axis in radians */
};

/**
 * @brief 6-DOF pose: position (meters) and orientation (radians)
 *
 * This POD is convenient for passing full sensor poses (x,y,z,roll,pitch,yaw).
 */
struct Pose {
    double x;   /**< meters */
    double y;   /**< meters */
    double z;   /**< meters */
    double roll;  /**< radians */
    double pitch; /**< radians */
    double yaw;   /**< radians */
};

/**
 * @brief Convert a pose's angular components from radians to degrees (returns new Pose).
 * @param p Input pose with roll/pitch/yaw in radians
 * @return Pose with same x,y,z and roll/pitch/yaw in degrees
 */
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
