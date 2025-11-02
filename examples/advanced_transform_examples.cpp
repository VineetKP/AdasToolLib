#include <iostream>
#include "helpers.hpp"
#include "transformers.hpp"

using namespace AdasTools;

int main()
{
    // Sequence: vehiclePose, lidarPose (deg->rad), extrinsics, local/global transforms

    // 1) vehicle pose in world
    Pose vehiclePose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // 2) lidar pose relative to vehicle (angles in degrees provided)
    Pose lidarPoseDeg = {2.0, 0.1, 0.5, 0.572958, -1.14592, 45.0}; // roll,pitch in deg (approx), yaw in deg

    // 3) convert lidar angles to radians (poseRadiansToDegrees exists; add inverse here inline)
    auto deg2rad = [](double d){ return d * 3.14159265358979323846 / 180.0; };
    Pose lidarPose = lidarPoseDeg;
    lidarPose.roll = deg2rad(lidarPoseDeg.roll);
    lidarPose.pitch = deg2rad(lidarPoseDeg.pitch);
    lidarPose.yaw = deg2rad(lidarPoseDeg.yaw);

    // 4) create extrinsic matrices
    double Mv[16], Ms[16];
    double pv[6] = { vehiclePose.x, vehiclePose.y, vehiclePose.z, vehiclePose.roll, vehiclePose.pitch, vehiclePose.yaw };
    double ps[6] = { lidarPose.x, lidarPose.y, lidarPose.z, lidarPose.roll, lidarPose.pitch, lidarPose.yaw };
    pose6ToMatrix(pv, Mv);
    pose6ToMatrix(ps, Ms);

    // Compose M = Mv * Ms (vehicle * sensor-local) so we can transform points
    double M[16];
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
        double s = 0.0;
        for (int k = 0; k < 4; ++k) s += Mv[r*4 + k] * Ms[k*4 + c];
        M[r*4 + c] = s;
    }

    // call local->global (vehicle, sensor-local)
    Pose lidarGlobal = localToGlobalFromMatrix(vehiclePose, lidarPose);
    std::cout << "Lidar global pose: (" << lidarGlobal.x << ", " << lidarGlobal.y << ", " << lidarGlobal.z << ")\n";

    // call global->local (vehicle, sensor-global) -- should return lidarPose (or close)
    Pose lidarBack = globalToLocalFromMatrix(vehiclePose, lidarGlobal);
    Pose lidarBackDeg = poseRadiansToDegrees(lidarBack);
    std::cout << "Lidar back in vehicle frame (deg): roll=" << lidarBackDeg.roll << " pitch=" << lidarBackDeg.pitch << " yaw=" << lidarBackDeg.yaw << "\n";

    // Demonstrate point transform: apply previously composed M to a point

    // Local point in lidar frame (as Pose: position + zero orientation)
    Pose p_local = {1.0, 0.5, 0.2, 0.0, 0.0, 0.0};
    // Apply M to point
    double X = p_local.x, Y = p_local.y, Z = p_local.z;
    double gx = M[0]*X + M[1]*Y + M[2]*Z + M[3];
    double gy = M[4]*X + M[5]*Y + M[6]*Z + M[7];
    double gz = M[8]*X + M[9]*Y + M[10]*Z + M[11];
    std::cout << "Lidar point global: (" << gx << ", " << gy << ", " << gz << ")\n";

    // --- Camera sensor init and projection sequence ---
    // camera sensor is mounted on vehicle at some pose (angles given in degrees)
    Pose cameraDeg = {1.8, -0.05, 0.4, 0.0, 0.0, 0.0};
    Pose cameraPose = cameraDeg;
    cameraPose.roll = deg2rad(cameraDeg.roll);
    cameraPose.pitch = deg2rad(cameraDeg.pitch);
    cameraPose.yaw = deg2rad(cameraDeg.yaw);

    // camera intrinsics
    double K[9] = {800.0, 0.0, 320.0,
                   0.0, 800.0, 240.0,
                   0.0, 0.0, 1.0};

    // camera extrinsic (vehicle->camera)
    double Mc[16];
    double pc[6] = { cameraPose.x, cameraPose.y, cameraPose.z, cameraPose.roll, cameraPose.pitch, cameraPose.yaw };
    pose6ToMatrix(pv, Mv); // vehicle (already computed but safe to refresh)
    pose6ToMatrix(pc, Mc); // camera (vehicle->camera)

    // Compute camera global matrix: M_camera_global = Mv * Mc
    double Mcam_global[16];
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
        double s = 0.0;
        for (int k = 0; k < 4; ++k) s += Mv[r*4 + k] * Mc[k*4 + c];
        Mcam_global[r*4 + c] = s;
    }

    // Compute inverse of Mcam_global (rigid transform): inv = [R^T, -R^T t; 0 1]
    double r00 = Mcam_global[0], r01 = Mcam_global[1], r02 = Mcam_global[2];
    double r10 = Mcam_global[4], r11 = Mcam_global[5], r12 = Mcam_global[6];
    double r20 = Mcam_global[8], r21 = Mcam_global[9], r22 = Mcam_global[10];
    double tx  = Mcam_global[3], ty = Mcam_global[7], tz = Mcam_global[11];
    double Rtr[9] = { r00, r10, r20, r01, r11, r21, r02, r12, r22 };
    double t_inv_x = -(Rtr[0]*tx + Rtr[1]*ty + Rtr[2]*tz);
    double t_inv_y = -(Rtr[3]*tx + Rtr[4]*ty + Rtr[5]*tz);
    double t_inv_z = -(Rtr[6]*tx + Rtr[7]*ty + Rtr[8]*tz);
    double Minv_cam[16];
    Minv_cam[0]=Rtr[0]; Minv_cam[1]=Rtr[1]; Minv_cam[2]=Rtr[2]; Minv_cam[3]=t_inv_x;
    Minv_cam[4]=Rtr[3]; Minv_cam[5]=Rtr[4]; Minv_cam[6]=Rtr[5]; Minv_cam[7]=t_inv_y;
    Minv_cam[8]=Rtr[6]; Minv_cam[9]=Rtr[7]; Minv_cam[10]=Rtr[8]; Minv_cam[11]=t_inv_z;
    Minv_cam[12]=0.0; Minv_cam[13]=0.0; Minv_cam[14]=0.0; Minv_cam[15]=1.0;

    // extrinsic mapping from lidar_local -> camera = Minv_cam * M_lidar_global (M)
    double extrinsic_lidar_to_camera[16];
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            double s = 0.0;
            for (int k = 0; k < 4; ++k) s += Minv_cam[r*4 + k] * M[k*4 + c];
            extrinsic_lidar_to_camera[r*4 + c] = s;
        }
    }

    // Project the lidar local point into the camera image
    Pose pix = projectPointCamera(p_local, extrinsic_lidar_to_camera, K);
    std::cout << "Projected pixel (u,v,depth): (" << pix.x << ", " << pix.y << ", " << pix.z << ")\n";

    // camera global->local: given camera global pose, compute camera pose in vehicle frame
    Pose cameraGlobal = localToGlobalFromMatrix(vehiclePose, cameraPose);
    Pose cameraInVehicle = globalToLocalFromMatrix(vehiclePose, cameraGlobal);
    Pose camDegOut = poseRadiansToDegrees(cameraInVehicle);
    std::cout << "Camera in vehicle frame (deg): x=" << cameraInVehicle.x << " y=" << cameraInVehicle.y << " z=" << cameraInVehicle.z << " yaw=" << camDegOut.yaw << "\n";

    return 0;
}
