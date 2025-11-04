#include <iostream>
#include <iomanip>
#include "helpers.hpp"
#include "transformers.hpp"

using namespace AdasTools;

static void printMat16(const double M[16]){
    std::cout<<std::fixed<<std::setprecision(6);
    for(int r=0;r<4;++r){
        for(int c=0;c<4;++c) std::cout<<M[r*4+c]<<" ";
        std::cout<<"\n";
    }
}

int main()
{
    auto deg2rad = [](double d){ return d * 3.14159265358979323846 / 180.0; };
    auto rad2deg = [](double r){ return r * 180.0 / 3.14159265358979323846; };

    // User test case (angles in degrees)
    Pose vehiclePose = {0.0, 0.0, 0.0, 0.0, 10.0, 0.0};
    Pose lidarPoseDeg = {1.0, 0.0, 1.0, 0.0, 5.0, 0.0};

    // Convert to radians where needed
    Pose v = vehiclePose;
    v.roll = deg2rad(v.roll); v.pitch = deg2rad(v.pitch); v.yaw = deg2rad(v.yaw);
    Pose s = lidarPoseDeg;
    s.roll = deg2rad(s.roll); s.pitch = deg2rad(s.pitch); s.yaw = deg2rad(s.yaw);

    double Mv[16], Ms[16], M[16];
    poseToMatrix(v, Mv);
    poseToMatrix(s, Ms);

    // M = Mv * Ms (row-major multiply)
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c){
        double sum = 0.0;
        for (int k = 0; k < 4; ++k) sum += Mv[r*4 + k] * Ms[k*4 + c];
        M[r*4 + c] = sum;
    }

    std::cout<<"Mv (vehicle->world):\n"; printMat16(Mv);
    std::cout<<"Ms (sensor->vehicle):\n"; printMat16(Ms);
    std::cout<<"M = Mv * Ms (sensor->world):\n"; printMat16(M);

    std::cout<<std::setprecision(9);
    std::cout<<"Composed translation (M[3],M[7],M[11]) = ("<<M[3]<<", "<<M[7]<<", "<<M[11]<<")\n";

    Pose out = localToGlobalFromMatrix(v, s);
    out.roll = rad2deg(out.roll); out.pitch = rad2deg(out.pitch); out.yaw = rad2deg(out.yaw);
    std::cout<<"localToGlobalFromMatrix result: ("<<out.x<<", "<<out.y<<", "<<out.z<<", "<<out.roll<<", "<<out.pitch<<", "<<out.yaw<<")\n";

    return 0;
}
