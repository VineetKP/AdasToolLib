#include <iostream>
#include "helpers.hpp"
#include "transformers.hpp"

using namespace AdasTools;

int main()
{
    Pose vehiclePose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Pose lidarPoseDeg = {2.0, 0.1, 0.5, 0.572958, -1.14592, 45.0};
    auto deg2rad = [](double d){ return d * 3.14159265358979323846 / 180.0; };
    Pose lidarPose = lidarPoseDeg;
    lidarPose.roll = deg2rad(lidarPoseDeg.roll);
    lidarPose.pitch = deg2rad(lidarPoseDeg.pitch);
    lidarPose.yaw = deg2rad(lidarPoseDeg.yaw);

    double Mv[16], Ms[16];
    double pv[6] = { vehiclePose.x, vehiclePose.y, vehiclePose.z, vehiclePose.roll, vehiclePose.pitch, vehiclePose.yaw };
    double ps[6] = { lidarPose.x, lidarPose.y, lidarPose.z, lidarPose.roll, lidarPose.pitch, lidarPose.yaw };
    pose6ToMatrix(pv, Mv);
    pose6ToMatrix(ps, Ms);

    double M[16];
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
        double s = 0.0;
        for (int k = 0; k < 4; ++k) s += Mv[r*4 + k] * Ms[k*4 + c];
        M[r*4 + c] = s;
    }

    Pose lidarGlobal = localToGlobalFromMatrix(vehiclePose, lidarPose);
    std::cout << "Lidar global pose: (" << lidarGlobal.x << ", " << lidarGlobal.y << ", " << lidarGlobal.z << ")\n";

    // Demonstrate using the composed matrix M to transform a local point
    double X = 1.0, Y = 0.5, Z = 0.2;
    double gx = M[0]*X + M[1]*Y + M[2]*Z + M[3];
    double gy = M[4]*X + M[5]*Y + M[6]*Z + M[7];
    double gz = M[8]*X + M[9]*Y + M[10]*Z + M[11];
    std::cout << "Transformed point (using M): (" << gx << ", " << gy << ", " << gz << ")\n";

    return 0;
}
