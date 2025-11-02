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

Point3 scalePosition(const Point3 &p, double scale)
{
    Point3 out;
    out.x = p.x * scale;
    out.y = p.y * scale;
    out.z = p.z * scale;
    return out;
}


// Helper: rotate a point by roll (X), pitch (Y), yaw (Z)
// Rotation applied as R = Rz(yaw) * Ry(pitch) * Rx(roll)
Point3 rotatePosition(const Point3 &p, double roll, double pitch, double yaw)
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

    Point3 out;
    out.x = r00 * p.x + r01 * p.y + r02 * p.z;
    out.y = r10 * p.x + r11 * p.y + r12 * p.z;
    out.z = r20 * p.x + r21 * p.y + r22 * p.z;
    return out;
}

Point3 translatePosition(const Point3 &p, double dx, double dy, double dz)
{
    Point3 out;
    out.x = p.x + dx;
    out.y = p.y + dy;
    out.z = p.z + dz;
    return out;
}


Pose globalToLocal(const Pose &globalPose, const Frame3D &frame)
{
    // Translate global so the frame origin is at the origin
    Pose t;
    t.x = globalPose.x - frame.x;
    t.y = globalPose.y - frame.y;
    t.z = globalPose.z - frame.z;

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
    Pose local;
    // Apply R^T * t (note: using transpose rows/cols)
    local.x = r00 * t.x + r10 * t.y + r20 * t.z;
    local.y = r01 * t.x + r11 * t.y + r21 * t.z;
    local.z = r02 * t.x + r12 * t.y + r22 * t.z;
    // preserve orientation (transforming positions only)
    local.roll = globalPose.roll; local.pitch = globalPose.pitch; local.yaw = globalPose.yaw;
    return local;
}

Pose localToGlobal(const Pose &localPos, const Frame3D &frame)
{
    // First rotate local position part by frame orientation (R * localPos.position)
    Point3 localPt{ localPos.x, localPos.y, localPos.z };
    Point3 rotated = rotatePosition(localPt, frame.roll, frame.pitch, frame.yaw);

    // Then translate by frame origin
    Pose global;
    global.x = rotated.x + frame.x;
    global.y = rotated.y + frame.y;
    global.z = rotated.z + frame.z;
    // preserve orientation fields from localPos
    global.roll = localPos.roll; global.pitch = localPos.pitch; global.yaw = localPos.yaw;
    return global;
}

void pose6ToMatrix(const double pose6[6], double outMat16[16])
{
    // pose6 = {x, y, z, roll, pitch, yaw}
    double x = pose6[0];
    double y = pose6[1];
    double z = pose6[2];
    double roll = pose6[3];
    double pitch = pose6[4];
    double yaw = pose6[5];

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    // R = Rz * Ry * Rx
    double r00 = cy*cp;
    double r01 = cy*sp*sr - sy*cr;
    double r02 = cy*sp*cr + sy*sr;

    double r10 = sy*cp;
    double r11 = sy*sp*sr + cy*cr;
    double r12 = sy*sp*cr - cy*sr;

    double r20 = -sp;
    double r21 = cp*sr;
    double r22 = cp*cr;

    // Row-major 4x4
    outMat16[0] = r00; outMat16[1] = r01; outMat16[2] = r02; outMat16[3] = x;
    outMat16[4] = r10; outMat16[5] = r11; outMat16[6] = r12; outMat16[7] = y;
    outMat16[8] = r20; outMat16[9] = r21; outMat16[10] = r22; outMat16[11] = z;
    outMat16[12] = 0.0; outMat16[13] = 0.0; outMat16[14] = 0.0; outMat16[15] = 1.0;
}

void poseToMatrix(const Pose &pose, double outMat16[16])
{
    double p[6];
    p[0] = pose.x; p[1] = pose.y; p[2] = pose.z;
    p[3] = pose.roll; p[4] = pose.pitch; p[5] = pose.yaw;
    pose6ToMatrix(p, outMat16);
}

Pose localToGlobalFromMatrix(const Pose &vehiclePose, const Pose &sensorPose)
{
    // Build 4x4 matrices for vehicle and sensor
    double Mv[16];
    double Ms[16];
    double pv[6] = { vehiclePose.x, vehiclePose.y, vehiclePose.z, vehiclePose.roll, vehiclePose.pitch, vehiclePose.yaw };
    double ps[6] = { sensorPose.x, sensorPose.y, sensorPose.z, sensorPose.roll, sensorPose.pitch, sensorPose.yaw };
    pose6ToMatrix(pv, Mv);
    pose6ToMatrix(ps, Ms);

    // Compute M = Mv * Ms (row-major multiply)
    double M[16];
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) {
                sum += Mv[r*4 + k] * Ms[k*4 + c];
            }
            M[r*4 + c] = sum;
        }
    }

    // Extract translation
    Pose out;
    out.x = M[3];
    out.y = M[7];
    out.z = M[11];

    // Extract R elements (row-major)
    double r00 = M[0], r01 = M[1];
    double r10 = M[4], r11 = M[5];
    double r20 = M[8], r21 = M[9], r22 = M[10];

    // Recover roll, pitch, yaw assuming R = Rz * Ry * Rx
    // From our earlier derivation:
    // r20 = -sin(pitch)
    double pitch = -asin(r20);
    double cp = cos(pitch);
    double roll = 0.0;
    double yaw = 0.0;
    if (fabs(cp) > 1e-8) {
        roll = atan2(r21 / cp, r22 / cp);
        yaw  = atan2(r10 / cp, r00 / cp);
    } else {
        // Gimbal lock: pitch ~= +-90 deg
        // yaw + roll = atan2(-r01, r11) (one degree of freedom lost)
        yaw = 0.0;
        roll = atan2(-r01, r11);
    }
    out.roll = roll; out.pitch = pitch; out.yaw = yaw;
    return out;
}

Pose globalToLocalFromMatrix(const Pose &vehiclePose, const Pose &sensorGlobalPose)
{
    // We want sensor_local = inverse(Mv) * Ms_global
    double Mv[16];
    double Mg[16];
    double pv[6] = { vehiclePose.x, vehiclePose.y, vehiclePose.z, vehiclePose.roll, vehiclePose.pitch, vehiclePose.yaw };
    double pg[6] = { sensorGlobalPose.x, sensorGlobalPose.y, sensorGlobalPose.z, sensorGlobalPose.roll, sensorGlobalPose.pitch, sensorGlobalPose.yaw };
    pose6ToMatrix(pv, Mv);
    pose6ToMatrix(pg, Mg);

    // Compute inverse of Mv (rigid body): inv(Mv) = [R^T, -R^T t; 0 1]
    double r00 = Mv[0], r01 = Mv[1], r02 = Mv[2];
    double r10 = Mv[4], r11 = Mv[5], r12 = Mv[6];
    double r20 = Mv[8], r21 = Mv[9], r22 = Mv[10];
    double tx  = Mv[3], ty = Mv[7], tz = Mv[11];

    double Rtr[9] = { r00, r10, r20, r01, r11, r21, r02, r12, r22 };
    double t_inv_x = -(Rtr[0]*tx + Rtr[1]*ty + Rtr[2]*tz);
    double t_inv_y = -(Rtr[3]*tx + Rtr[4]*ty + Rtr[5]*tz);
    double t_inv_z = -(Rtr[6]*tx + Rtr[7]*ty + Rtr[8]*tz);

    double Minv[16];
    Minv[0]=Rtr[0]; Minv[1]=Rtr[1]; Minv[2]=Rtr[2]; Minv[3]=t_inv_x;
    Minv[4]=Rtr[3]; Minv[5]=Rtr[4]; Minv[6]=Rtr[5]; Minv[7]=t_inv_y;
    Minv[8]=Rtr[6]; Minv[9]=Rtr[7]; Minv[10]=Rtr[8]; Minv[11]=t_inv_z;
    Minv[12]=0.0; Minv[13]=0.0; Minv[14]=0.0; Minv[15]=1.0;

    // sensor_local = Minv * Mg
    double Mloc[16];
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) sum += Minv[r*4 + k] * Mg[k*4 + c];
            Mloc[r*4 + c] = sum;
        }
    }

    // Extract pose from Mloc
    Pose out;
    out.x = Mloc[3]; out.y = Mloc[7]; out.z = Mloc[11];
    double rr00 = Mloc[0], rr01 = Mloc[1];
    double rr10 = Mloc[4], rr11 = Mloc[5];
    double rr20 = Mloc[8], rr21 = Mloc[9], rr22 = Mloc[10];
    double pitch = -asin(rr20);
    double cp = cos(pitch);
    double roll = 0.0; double yaw = 0.0;
    if (fabs(cp) > 1e-8) {
        roll = atan2(rr21 / cp, rr22 / cp);
        yaw  = atan2(rr10 / cp, rr00 / cp);
    } else {
        yaw = 0.0;
        roll = atan2(-rr01, rr11);
    }
    out.roll = roll; out.pitch = pitch; out.yaw = yaw;
    return out;
}

// Point3 overloads
Point3 localToGlobal(const Point3 &localPos, const Frame3D &frame)
{
    Point3 rotated = rotatePosition(localPos, frame.roll, frame.pitch, frame.yaw);
    Point3 out;
    out.x = rotated.x + frame.x;
    out.y = rotated.y + frame.y;
    out.z = rotated.z + frame.z;
    return out;
}

Point3 globalToLocal(const Point3 &globalPos, const Frame3D &frame)
{
    // Translate global so the frame origin is at the origin
    Point3 t;
    t.x = globalPos.x - frame.x;
    t.y = globalPos.y - frame.y;
    t.z = globalPos.z - frame.z;

    // Build R (same as in Pose path)
    double cr = cos(frame.roll);
    double sr = sin(frame.roll);
    double cp = cos(frame.pitch);
    double sp = sin(frame.pitch);
    double cy = cos(frame.yaw);
    double sy = sin(frame.yaw);

    double r00 = cy*cp;
    double r01 = cy*sp*sr - sy*cr;
    double r02 = cy*sp*cr + sy*sr;

    double r10 = sy*cp;
    double r11 = sy*sp*sr + cy*cr;
    double r12 = sy*sp*cr - cy*sr;

    double r20 = -sp;
    double r21 = cp*sr;
    double r22 = cp*cr;

    Point3 local;
    local.x = r00 * t.x + r10 * t.y + r20 * t.z;
    local.y = r01 * t.x + r11 * t.y + r21 * t.z;
    local.z = r02 * t.x + r12 * t.y + r22 * t.z;
    return local;
}

Point3 projectPointCamera(const Point3 &pointLocal, const double extrinsic[16], const double intrinsic[9])
{
    // Map local point to camera coordinates: p_cam = Extrinsic * [X Y Z 1]^T
    double X = pointLocal.x;
    double Y = pointLocal.y;
    double Z = pointLocal.z;

    double cx = extrinsic[3];
    double cy = extrinsic[7];
    double cz = extrinsic[11];

    double x_cam = extrinsic[0]*X + extrinsic[1]*Y + extrinsic[2]*Z + cx;
    double y_cam = extrinsic[4]*X + extrinsic[5]*Y + extrinsic[6]*Z + cy;
    double z_cam = extrinsic[8]*X + extrinsic[9]*Y + extrinsic[10]*Z + cz;

    Point3 out;
    if (z_cam == 0.0) {
        out.x = 0.0; out.y = 0.0; out.z = 0.0;
        return out;
    }

    double fx = intrinsic[0];
    double s  = intrinsic[1];
    double cx_i = intrinsic[2];
    double fy = intrinsic[4];
    double cy_i = intrinsic[5];

    double u = (fx * x_cam + s * y_cam) / z_cam + cx_i;
    double v = (fy * y_cam) / z_cam + cy_i;

    out.x = u; out.y = v; out.z = z_cam;
    return out;
}

Pose projectPointCamera(const Pose &pointLocal, const double extrinsic[16], const double intrinsic[9])
{
    // Map local point to camera coordinates: p_cam = Extrinsic * [X Y Z 1]^T
    double X = pointLocal.x;
    double Y = pointLocal.y;
    double Z = pointLocal.z;

    double cx = extrinsic[3];
    double cy = extrinsic[7];
    double cz = extrinsic[11];

    double x_cam = extrinsic[0]*X + extrinsic[1]*Y + extrinsic[2]*Z + cx;
    double y_cam = extrinsic[4]*X + extrinsic[5]*Y + extrinsic[6]*Z + cy;
    double z_cam = extrinsic[8]*X + extrinsic[9]*Y + extrinsic[10]*Z + cz;

    Pose out;
    if (z_cam == 0.0) {
        out.x = 0.0; out.y = 0.0; out.z = 0.0;
        return out;
    }

    // Intrinsic K (row-major): [fx s cx; 0 fy cy; 0 0 1]
    double fx = intrinsic[0];
    double s  = intrinsic[1];
    double cx_i = intrinsic[2];
    double fy = intrinsic[4];
    double cy_i = intrinsic[5];

    double u = (fx * x_cam + s * y_cam) / z_cam + cx_i;
    double v = (fy * y_cam) / z_cam + cy_i;

    out.x = u; out.y = v; out.z = z_cam;
    return out;
}

} // namespace AdasTools
