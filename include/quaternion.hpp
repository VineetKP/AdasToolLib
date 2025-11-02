/* *******************************************************************************
 * File: include/quaternion.hpp
 * Project: ADAS Tools Library (adas_tools)
 * Description: Minimal quaternion utilities (pure C++, no STL). Provides a
 *              small POD Quaternion type and basic operations used for
 *              3D orientation handling and safe interpolation.
 * Author: VineetKP
 * Created: 2025-10-31
 * *******************************************************************************/

#pragma once
#include "helpers.hpp"

namespace AdasTools {

/**
 * @brief Simple quaternion representation (w + xi + yj + zk)
 *
 * This is a POD type to avoid dynamic memory or STL usage. Use the
 * provided operations for quaternion arithmetic.
 */
struct Quaternion {
    double w;
    double x;
    double y;
    double z;
};

/**
 * @brief Create a quaternion from roll, pitch, yaw (radians).
 *
 * @param roll rotation about X-axis
 * @param pitch rotation about Y-axis
 * @param yaw rotation about Z-axis
 * @return Quaternion representing the composed rotation
 */
Quaternion quaternionFromRPY(double roll, double pitch, double yaw);

/**
 * @brief Convert quaternion to a direction cosine rotation matrix applied to a
 *        Position vector: v_rotated = R(q) * v.
 *
 * @param q Quaternion
 * @param p Input point
 * @return Rotated point
 */
Point3 rotateByQuaternion(const Quaternion &q, const Point3 &p);

/**
 * @brief Multiply two quaternions (q1 * q2)
 */
Quaternion multiplyQuaternion(const Quaternion &q1, const Quaternion &q2);

/**
 * @brief Normalize a quaternion to unit length.
 */
Quaternion normalizeQuaternion(const Quaternion &q);

/**
 * @brief Spherical linear interpolation between two quaternions.
 * @param a Start quaternion (should be normalized)
 * @param b End quaternion (should be normalized)
 * @param t Interpolation parameter [0,1]
 * @return Interpolated (normalized) quaternion
 */
Quaternion slerp(const Quaternion &a, const Quaternion &b, double t);

} // namespace AdasTools
