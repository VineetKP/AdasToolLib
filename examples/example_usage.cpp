/* *******************************************************************************
 * File: examples/example_usage.cpp
 * Description: Small example demonstrating transformers and quaternion utils.
 * *******************************************************************************/

#include <iostream>
#include "helpers.hpp"
#include "transformers.hpp"
#include "quaternion.hpp"

int main()
{
    using namespace AdasTools;
    Point3 p_local = {1.0, 0.5, 0.2};

    Frame3D sensorFrame;
    sensorFrame.x = 2.0;
    sensorFrame.y = 0.1;
    sensorFrame.z = 0.5;
    sensorFrame.roll  = 0.0;
    sensorFrame.pitch = 0.0;
    sensorFrame.yaw   = 0.785398; // 45 degrees

    Point3 p_global = localToGlobal(p_local, sensorFrame);

    // Use quaternion to rotate the local point instead (should match rotatePosition)
    Quaternion q = quaternionFromRPY(sensorFrame.roll, sensorFrame.pitch, sensorFrame.yaw);
    q = normalizeQuaternion(q);
    Point3 p_rot = rotateByQuaternion(q, p_local);

    // Translate rotated point to global
    Point3 p_global_q;
    p_global_q.x = p_rot.x + sensorFrame.x;
    p_global_q.y = p_rot.y + sensorFrame.y;
    p_global_q.z = p_rot.z + sensorFrame.z;

    std::cout << "p_global (matrix) = (" << p_global.x << ", " << p_global.y << ", " << p_global.z << ")\n";
    std::cout << "p_global (quat)   = (" << p_global_q.x << ", " << p_global_q.y << ", " << p_global_q.z << ")\n";

    // Round-trip
    Point3 back = globalToLocal(p_global, sensorFrame);
    std::cout << "round-trip equals local: (" << back.x << ", " << back.y << ", " << back.z << ")\n";

    return 0;
}
