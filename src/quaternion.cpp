/* *******************************************************************************
 * File: src/quaternion.cpp
 * Project: ADAS Tools Library (adas_tools)
 * Description: Implement quaternion utilities (no STL). Basic creation from
 *              roll/pitch/yaw, multiplication, normalization and applying
 *              rotation to a Position.
 * Author: VineetKP
 * Created: 2025-10-31
 * *******************************************************************************/

#include "quaternion.hpp"
#include <math.h>

namespace AdasTools {

Quaternion quaternionFromRPY(double roll, double pitch, double yaw)
{
    // Convert half-angles
    double hr = roll * 0.5;
    double hp = pitch * 0.5;
    double hy = yaw * 0.5;

    double cr = cos(hr);
    double sr = sin(hr);
    double cp = cos(hp);
    double sp = sin(hp);
    double cy = cos(hy);
    double sy = sin(hy);

    Quaternion q;
    // Using the z-y-x intrinsic order (same as Rz*Ry*Rx)
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;
    return q;
}

Quaternion multiplyQuaternion(const Quaternion &q1, const Quaternion &q2)
{
    Quaternion r;
    r.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    r.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    r.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    r.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return r;
}

Quaternion normalizeQuaternion(const Quaternion &q)
{
    double norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    Quaternion out;
    if (norm == 0.0) {
        out.w = 1.0; out.x = 0.0; out.y = 0.0; out.z = 0.0;
    } else {
        out.w = q.w / norm;
        out.x = q.x / norm;
        out.y = q.y / norm;
        out.z = q.z / norm;
    }
    return out;
}

Position rotateByQuaternion(const Quaternion &q, const Position &p)
{
    // Convert quaternion to rotation matrix and apply to p
    double ww = q.w*q.w;
    double xx = q.x*q.x;
    double yy = q.y*q.y;
    double zz = q.z*q.z;

    double wx = q.w*q.x;
    double wy = q.w*q.y;
    double wz = q.w*q.z;

    double xy = q.x*q.y;
    double xz = q.x*q.z;
    double yz = q.y*q.z;

    double r00 = ww + xx - yy - zz;
    double r01 = 2.0*(xy - wz);
    double r02 = 2.0*(xz + wy);

    double r10 = 2.0*(xy + wz);
    double r11 = ww - xx + yy - zz;
    double r12 = 2.0*(yz - wx);

    double r20 = 2.0*(xz - wy);
    double r21 = 2.0*(yz + wx);
    double r22 = ww - xx - yy + zz;

    Position out;
    out.x = r00 * p.x + r01 * p.y + r02 * p.z;
    out.y = r10 * p.x + r11 * p.y + r12 * p.z;
    out.z = r20 * p.x + r21 * p.y + r22 * p.z;
    return out;
}

Quaternion slerp(const Quaternion &a, const Quaternion &b, double t)
{
    // Compute the cosine of the angle between the two quaternions.
    double cosom = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;

    // If negative dot, negate one quaternion to take shorter path
    Quaternion bcopy = b;
    if (cosom < 0.0) {
        cosom = -cosom;
        bcopy.w = -bcopy.w;
        bcopy.x = -bcopy.x;
        bcopy.y = -bcopy.y;
        bcopy.z = -bcopy.z;
    }

    double scale0, scale1;
    if ((1.0 - cosom) > 1e-6) {
        // Standard case (slerp)
        double omega = acos(cosom);
        double invSin = 1.0 / sin(omega);
        scale0 = sin((1.0 - t) * omega) * invSin;
        scale1 = sin(t * omega) * invSin;
    } else {
        // Quaternions are very close, use linear interpolation
        scale0 = 1.0 - t;
        scale1 = t;
    }

    Quaternion out;
    out.w = scale0 * a.w + scale1 * bcopy.w;
    out.x = scale0 * a.x + scale1 * bcopy.x;
    out.y = scale0 * a.y + scale1 * bcopy.y;
    out.z = scale0 * a.z + scale1 * bcopy.z;
    return normalizeQuaternion(out);
}

} // namespace AdasTools
