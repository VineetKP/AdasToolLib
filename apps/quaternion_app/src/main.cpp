#include <iostream>
#include <cstdlib>
#include "quaternion.hpp"

using namespace AdasTools;

int main(int argc, char **argv)
{
    double yaw_deg = 90.0;
    int steps = 10;
    if (argc >= 2) yaw_deg = std::atof(argv[1]);
    if (argc >= 3) steps = std::atoi(argv[2]);

    double yaw = yaw_deg * 3.14159265358979323846 / 180.0;
    Quaternion q1 = quaternionFromRPY(0.0, 0.0, 0.0);
    Quaternion q2 = quaternionFromRPY(0.0, 0.0, yaw);

    Pose p = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i <= steps; ++i) {
        double t = (double)i / (double)steps;
        Quaternion q = slerp(q1, q2, t);
        Pose rp = rotateByQuaternion(q, p);
        std::cout << "t=" << t << " rp=(" << rp.x << "," << rp.y << "," << rp.z << ")\n";
    }
    return 0;
}
