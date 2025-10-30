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
 * @brief Scale a position by a uniform scale factor.
 * @param p Input position
 * @param scale Uniform scale factor
 * @return Scaled position
 */
Position scalePosition(const Position &p, double scale);

/**
 * @brief Rotate a position around the origin by roll/pitch/yaw (radians).
 *
 * Rotation order: R = Rz(yaw) * Ry(pitch) * Rx(roll).
 * @param p Input position
 * @param roll Rotation about X axis (radians)
 * @param pitch Rotation about Y axis (radians)
 * @param yaw Rotation about Z axis (radians)
 * @return Rotated position
 */
Position rotatePosition(const Position &p, double roll, double pitch, double yaw);

/**
 * @brief Translate a position by dx, dy, dz.
 * @param p Input position
 * @param dx Translation in x
 * @param dy Translation in y
 * @param dz Translation in z
 * @return Translated position
 */
Position translatePosition(const Position &p, double dx, double dy, double dz);

/**
 * @brief Convert a position in global coordinates to a local coordinate
 *        frame defined by a 3D origin and roll/pitch/yaw orientation.
 * @param globalPos Position in global frame
 * @param frame Frame origin and orientation
 * @return Position expressed in the local frame
 */
Position globalToLocal(const Position &globalPos, const Frame3D &frame);

/**
 * @brief Convert a position in local coordinates to global coordinates
 *        using the provided 3D frame (origin + orientation).
 * @param localPos Position in local frame
 * @param frame Frame origin and orientation
 * @return Position expressed in the global frame
 */
Position localToGlobal(const Position &localPos, const Frame3D &frame);

} // namespace AdasTools
