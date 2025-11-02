/* *******************************************************************************
 * File: include/transformers.hpp
 * Project: ADAS Tools Library (adas_tools)
 * Description: Coordinate transformers for ADAS sensors. Provides pure-C++
 *              functions (no STL) to scale, rotate, translate and convert
 *              between global and local coordinate frames.
 * Author: VineetKP
 * Created: 2025-10-31
 * *******************************************************************************/

#pragma once
#include "helpers.hpp"

namespace AdasTools {

/**
 * @brief Scale a point by a uniform scale factor.
 * @param p Input point
 * @param scale Uniform scale factor
 * @return Scaled point
 */
Point3 scalePosition(const Point3 &p, double scale);

/**
 * @brief Rotate a point around the origin by roll/pitch/yaw (radians).
 *
 * Rotation order: R = Rz(yaw) * Ry(pitch) * Rx(roll).
 * @param p Input point
 * @param roll Rotation about X axis (radians)
 * @param pitch Rotation about Y axis (radians)
 * @param yaw Rotation about Z axis (radians)
 * @return Rotated position
 */
Point3 rotatePosition(const Point3 &p, double roll, double pitch, double yaw);

/**
 * @brief Translate a point by dx, dy, dz.
 * @param p Input point
 * @param dx Translation in x
 * @param dy Translation in y
 * @param dz Translation in z
 * @return Translated position
 */
Point3 translatePosition(const Point3 &p, double dx, double dy, double dz);

/**
 * @brief Convert a point in global coordinates to a local coordinate
 *        frame defined by a 3D origin and roll/pitch/yaw orientation.
 * @param globalPos Point in global frame
 * @param frame Frame origin and orientation
 * @return Point expressed in the local frame
 */
Pose globalToLocal(const Pose &globalPose, const Frame3D &frame);

/**
 * @brief Convert a point in local coordinates to global coordinates
 *        using the provided 3D frame (origin + orientation).
 * @param localPos Point in local frame
 * @param frame Frame origin and orientation
 * @return Point expressed in the global frame
 */
Pose localToGlobal(const Pose &localPose, const Frame3D &frame);

// Point3 overloads
Point3 globalToLocal(const Point3 &globalPos, const Frame3D &frame);
Point3 localToGlobal(const Point3 &localPos, const Frame3D &frame);

/**
 * @brief Build a row-major 4x4 homogeneous matrix from a 6-DOF pose
 *        expressed as [x, y, z, roll, pitch, yaw]. The output matrix is
 *        row-major and uses the same rotation convention R = Rz * Ry * Rx.
 * @param pose6 Input array of length 6: {x,y,z,roll,pitch,yaw}
 * @param outMat16 Output 4x4 matrix in row-major order (length 16)
 */
void pose6ToMatrix(const double pose6[6], double outMat16[16]);

/**
 * @brief Overload that accepts a `Pose` POD.
 */
void poseToMatrix(const Pose &pose, double outMat16[16]);

/**
 * @brief Transform a local point to global using a 4x4 row-major
 *        homogeneous transform matrix (row-major, translation in column 3).
 * @param localPos Point in local frame
 * @param mat16 4x4 transform matrix (row-major)
 * @return Point expressed in the global frame
 */
/**
 * @brief Compose vehicle and sensor poses (both as Pose) into a global sensor Pose.
 * @param vehiclePose Pose of the vehicle in world coordinates
 * @param sensorPose Pose of the sensor in vehicle (local) coordinates
 * @return sensor pose expressed in world/global coordinates
 */
Pose localToGlobalFromMatrix(const Pose &vehiclePose, const Pose &sensorPose);

/**
 * @brief Transform a global point to local using a 4x4 row-major
 *        homogeneous transform matrix (row-major, translation in column 3).
 * @param globalPos Point in global frame
 * @param mat16 4x4 transform matrix (row-major)
 * @return Point expressed in the local frame
 */
/**
 * @brief Compute the local sensor pose given vehicle pose and sensor global pose.
 * @param vehiclePose Pose of the vehicle in world coordinates
 * @param sensorGlobalPose Pose of the sensor in world/global coordinates
 * @return sensor pose expressed in vehicle (local) coordinates
 */
Pose globalToLocalFromMatrix(const Pose &vehiclePose, const Pose &sensorGlobalPose);

/**
 * @brief Project a 3D point (local sensor coordinates) into camera image
 *        pixel coordinates using a 4x4 extrinsic matrix and a 3x3 intrinsic
 *        matrix. Extrinsic is row-major and maps the input point into camera
 *        coordinates: p_cam = Extrinsic * [X Y Z 1]^T. Intrinsic is row-major
 *        3x3 K matrix: [fx s cx; 0 fy cy; 0 0 1]. The returned Point3 holds
 *        (u, v, depth).
 * @param pointLocal Point in the sensor/local coordinate system
 * @param extrinsic 4x4 extrinsic matrix (row-major)
 * @param intrinsic 3x3 intrinsic matrix (row-major, length 9)
 * @return Point3 where x=u, y=v are pixel coordinates and z=depth (camera Z)
 */
Pose projectPointCamera(const Pose &pointLocal, const double extrinsic[16], const double intrinsic[9]);
Point3 projectPointCamera(const Point3 &pointLocal, const double extrinsic[16], const double intrinsic[9]);

} // namespace AdasTools
