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
 * @brief Simple 3D position type (POD) representing coordinates in meters.
 */
struct Position {
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

} // namespace AdasTools
