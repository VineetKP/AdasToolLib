#include "quaternion.hpp"
#include <math.h>

namespace AdasTools {

Quaternion quaternionFromRPY(double roll, double pitch, double yaw)
{
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
    if (norm == 0.0) { out.w = 1.0; out.x = out.y = out.z = 0.0; }
    else { out.w = q.w / norm; out.x = q.x / norm; out.y = q.y / norm; out.z = q.z / norm; }
    return out;
}

Point3 rotateByQuaternion(const Quaternion &q, const Point3 &p)
{
    double ww = q.w*q.w; double xx = q.x*q.x; double yy = q.y*q.y; double zz = q.z*q.z;
    double wx = q.w*q.x; double wy = q.w*q.y; double wz = q.w*q.z;
    double xy = q.x*q.y; double xz = q.x*q.z; double yz = q.y*q.z;

    double r00 = ww + xx - yy - zz;
    double r01 = 2.0*(xy - wz);
    double r10 = 2.0*(xy + wz);
    double r11 = ww - xx + yy - zz;
    double r02 = 2.0*(xz + wy);
    double r12 = 2.0*(yz - wx);
    double r20 = 2.0*(xz - wy);
    double r21 = 2.0*(yz + wx);
    double r22 = ww - xx - yy + zz;

    Point3 out;
    out.x = r00 * p.x + r01 * p.y + r02 * p.z;
    out.y = r10 * p.x + r11 * p.y + r12 * p.z;
    out.z = r20 * p.x + r21 * p.y + r22 * p.z;
    return out;
}

Pose rotateByQuaternion(const Quaternion &q, const Pose &p)
{
    Point3 vin{p.x, p.y, p.z};
    Point3 vout = rotateByQuaternion(q, vin);
    // Build rotation matrix from quaternion (same as in rotateByQuaternion for Point3)
    double ww = q.w*q.w; double xx = q.x*q.x; double yy = q.y*q.y; double zz = q.z*q.z;
    double wx = q.w*q.x; double wy = q.w*q.y; double wz = q.w*q.z;
    double xy = q.x*q.y; double xz = q.x*q.z; double yz = q.y*q.z;

    double r00 = ww + xx - yy - zz;
    double r01 = 2.0*(xy - wz);
    double r02 = 2.0*(xz + wy);

    double r10 = 2.0*(xy + wz);
    double r11 = ww - xx + yy - zz;
    double r12 = 2.0*(yz - wx);

    double r20 = 2.0*(xz - wy);
    double r21 = 2.0*(yz + wx);
    double r22 = ww - xx - yy + zz;

    // Extract roll, pitch, yaw using convention R = Rz * Ry * Rx
    double pitch = -asin(r20);
    double cp = cos(pitch);
    double roll = 0.0; double yaw = 0.0;
    if (fabs(cp) > 1e-8) {
        roll = atan2(r21 / cp, r22 / cp);
        yaw  = atan2(r10 / cp, r00 / cp);
    } else {
        yaw = 0.0;
        roll = atan2(-r01, r11);
    }

    Pose out; out.x = vout.x; out.y = vout.y; out.z = vout.z; out.roll = roll; out.pitch = pitch; out.yaw = yaw; return out;
}

Quaternion slerp(const Quaternion &a, const Quaternion &b, double t)
{
    double cosom = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    Quaternion bcopy = b;
    if (cosom < 0.0) { cosom = -cosom; bcopy.w = -bcopy.w; bcopy.x = -bcopy.x; bcopy.y = -bcopy.y; bcopy.z = -bcopy.z; }
    double scale0, scale1;
    if ((1.0 - cosom) > 1e-6) {
        double omega = acos(cosom);
        double invSin = 1.0 / sin(omega);
        scale0 = sin((1.0 - t) * omega) * invSin;
        scale1 = sin(t * omega) * invSin;
    } else { scale0 = 1.0 - t; scale1 = t; }
    Quaternion out;
    out.w = scale0 * a.w + scale1 * bcopy.w;
    out.x = scale0 * a.x + scale1 * bcopy.x;
    out.y = scale0 * a.y + scale1 * bcopy.y;
    out.z = scale0 * a.z + scale1 * bcopy.z;
    return normalizeQuaternion(out);
}

} // namespace AdasTools
