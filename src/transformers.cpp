/* *******************************************************************************
 * File: src/transformers.cpp
 * Project: ADAS Tools Library (adas_tools)
 * Description: Implementations for coordinate transformers. These are simple
 *             , dependency-free functions intended to run in constrained
 *              environments. No STL containers are used.
 * Author: VineetKP
 * Created: 2025-10-31
 * *******************************************************************************/

#include "transformers.hpp"
#include <math.h>

namespace AdasTools {

Position scalePosition(const Position &p, double scale)
{
    Position out;
    out.x = p.x * scale;
    out.y = p.y * scale;
    out.z = p.z * scale;
    return out;
}


// Helper: rotate a point by roll (X), pitch (Y), yaw (Z)
// Rotation applied as R = Rz(yaw) * Ry(pitch) * Rx(roll)
Position rotatePosition(const Position &p, double roll, double pitch, double yaw)
{
    // Precompute sines and cosines
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    // Combined rotation matrix R = Rz * Ry * Rx
    // R elements (row-major)
    double r00 = cy*cp;
    double r01 = cy*sp*sr - sy*cr;
    double r02 = cy*sp*cr + sy*sr;

    double r10 = sy*cp;
    double r11 = sy*sp*sr + cy*cr;
    double r12 = sy*sp*cr - cy*sr;

    double r20 = -sp;
    double r21 = cp*sr;
    double r22 = cp*cr;

    Position out;
    out.x = r00 * p.x + r01 * p.y + r02 * p.z;
    out.y = r10 * p.x + r11 * p.y + r12 * p.z;
    out.z = r20 * p.x + r21 * p.y + r22 * p.z;
    return out;
}

Position translatePosition(const Position &p, double dx, double dy, double dz)
{
    Position out;
    out.x = p.x + dx;
    out.y = p.y + dy;
    out.z = p.z + dz;
    return out;
}


Position globalToLocal(const Position &globalPos, const Frame3D &frame)
{
    // Translate global so the frame origin is at the origin
    Position t;
    t.x = globalPos.x - frame.x;
    t.y = globalPos.y - frame.y;
    t.z = globalPos.z - frame.z;

    // To go from global to local we need to apply the inverse rotation.
    // For a rotation matrix R (frame orientation), the inverse is R^T.
    // Since rotatePosition applies R = Rz*Ry*Rx, the inverse is rotation
    // by -roll, -pitch, -yaw in reverse order. We can perform the inverse
    // by using the transpose of R computed from roll/pitch/yaw.

    // Precompute sines and cosines for frame angles
    double cr = cos(frame.roll);
    double sr = sin(frame.roll);
    double cp = cos(frame.pitch);
    double sp = sin(frame.pitch);
    double cy = cos(frame.yaw);
    double sy = sin(frame.yaw);

    // R (as in rotatePosition) rows
    double r00 = cy*cp;
    double r01 = cy*sp*sr - sy*cr;
    double r02 = cy*sp*cr + sy*sr;

    double r10 = sy*cp;
    double r11 = sy*sp*sr + cy*cr;
    double r12 = sy*sp*cr - cy*sr;

    double r20 = -sp;
    double r21 = cp*sr;
    double r22 = cp*cr;

    // Apply R^T * t
    Position local;
    local.x = r00 * t.x + r10 * t.y + r20 * t.z;
    local.y = r01 * t.x + r11 * t.y + r21 * t.z;
    local.z = r02 * t.x + r12 * t.y + r22 * t.z;
    return local;
}

Position localToGlobal(const Position &localPos, const Frame3D &frame)
{
    // First rotate local position by frame orientation (R * localPos)
    Position rotated = rotatePosition(localPos, frame.roll, frame.pitch, frame.yaw);

    // Then translate by frame origin
    Position global;
    global.x = rotated.x + frame.x;
    global.y = rotated.y + frame.y;
    global.z = rotated.z + frame.z;
    return global;
}

} // namespace AdasTools
