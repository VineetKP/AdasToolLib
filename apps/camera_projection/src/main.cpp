#include <iostream>
#include "helpers.hpp"
#include "transformers.hpp"

using namespace AdasTools;

int main()
{
    auto deg2rad = [](double d){ return d * 3.14159265358979323846 / 180.0; };
    // Pose vehiclePose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    Pose cameraPose = {-2.0, 0.0, 1.5, 0.0, deg2rad(90), 0.0};
    
    // Point in vehicle frame
    Pose p_local = {10.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    
    // Intrinsics
    double K[9] = {
        800.0, 0.0, 320.0,
        0.0, 800.0, 240.0,
        0.0, 0.0, 1.0
    };
    
    
    double M_cam_in_vehicle[16];
    double pc[6] = {cameraPose.x, cameraPose.y, cameraPose.z, 
                     cameraPose.roll, cameraPose.pitch, cameraPose.yaw};
    pose6ToMatrix(pc, M_cam_in_vehicle);
    
    double extrinsic[16];
    double r00 = M_cam_in_vehicle[0], r01 = M_cam_in_vehicle[1], r02 = M_cam_in_vehicle[2];
    double r10 = M_cam_in_vehicle[4], r11 = M_cam_in_vehicle[5], r12 = M_cam_in_vehicle[6];
    double r20 = M_cam_in_vehicle[8], r21 = M_cam_in_vehicle[9], r22 = M_cam_in_vehicle[10];
    double tx = M_cam_in_vehicle[3], ty = M_cam_in_vehicle[7], tz = M_cam_in_vehicle[11];
    
    double Rtr[9] = {r00, r10, r20, r01, r11, r21, r02, r12, r22};
    double t_inv_x = -(Rtr[0]*tx + Rtr[1]*ty + Rtr[2]*tz);
    double t_inv_y = -(Rtr[3]*tx + Rtr[4]*ty + Rtr[5]*tz);
    double t_inv_z = -(Rtr[6]*tx + Rtr[7]*ty + Rtr[8]*tz);
    
    extrinsic[0]=Rtr[0]; extrinsic[1]=Rtr[1]; extrinsic[2]=Rtr[2];   extrinsic[3]=t_inv_x;
    extrinsic[4]=Rtr[3]; extrinsic[5]=Rtr[4]; extrinsic[6]=Rtr[5];   extrinsic[7]=t_inv_y;
    extrinsic[8]=Rtr[6]; extrinsic[9]=Rtr[7]; extrinsic[10]=Rtr[8];  extrinsic[11]=t_inv_z;
    extrinsic[12]=0;     extrinsic[13]=0;     extrinsic[14]=0;       extrinsic[15]=1;
    
    Pose pix = projectPointCamera(p_local, extrinsic, K);
    
    if (pix.z > 0) {
        std::cout << "Point is IN FRONT of camera\n";
        if (pix.x >= 0 && pix.x < 640 && pix.y >= 0 && pix.y < 480) {
            std::cout << "Point is VISIBLE in image!\n";
        } else {
            std::cout << "Point is outside image bounds (640x480)\n";
        }
    } else {
        std::cout << "Point is BEHIND camera\n";
    }
    std::cout << "Projected pixel (u,v,depth): (" << pix.x << ", " << pix.y << ", " << pix.z << ")\n";
    
    return 0;
}

