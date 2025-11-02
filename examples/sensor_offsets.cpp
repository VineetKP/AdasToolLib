/* *******************************************************************************
 * File: examples/sensor_offsets.cpp
 * Description: Demonstrate sensor mounting offsets for camera, lidar, and radar.
 *              Converts a sample point from each sensor frame into vehicle/global
 *              coordinates and then into another sensor frame.
 * *******************************************************************************/

#include <iostream>
#include "helpers.hpp"
#include "transformers.hpp"
#include "quaternion.hpp"

using namespace AdasTools;

int main()
{
    // Define a sample point in each sensor's local frame
    Point3 p_camera = {0.5, 0.0, 1.2}; // in front of the camera
    Point3 p_lidar  = {1.0, -0.2, 0.3};
    Point3 p_radar  = {2.5, 0.1, 0.2};

    // Sensor frames relative to vehicle (example values)
    Frame3D cameraFrame = {1.2, 0.3, 1.0, 0.0, 0.0, 0.05}; // small yaw
    Frame3D lidarFrame  = {0.8, -0.2, 0.6, 0.01, -0.02, -0.03};
    Frame3D radarFrame  = {0.5, 0.0, 0.4, 0.0, 0.0, 0.1};

    // Convert camera point to global and then to lidar frame
    Point3 cam_global = localToGlobal(p_camera, cameraFrame);
    Point3 cam_in_lidar = globalToLocal(cam_global, lidarFrame);

    std::cout << "Camera point in global: (" << cam_global.x << ", " << cam_global.y << ", " << cam_global.z << ")\n";
    std::cout << "Camera point in lidar frame: (" << cam_in_lidar.x << ", " << cam_in_lidar.y << ", " << cam_in_lidar.z << ")\n";

    // Demonstrate using quaternion to rotate point from radar to vehicle
    Quaternion qrad = quaternionFromRPY(radarFrame.roll, radarFrame.pitch, radarFrame.yaw);
    qrad = normalizeQuaternion(qrad);
    Point3 radar_rot = rotateByQuaternion(qrad, p_radar);
    Point3 radar_global;
    radar_global.x = radar_rot.x + radarFrame.x;
    radar_global.y = radar_rot.y + radarFrame.y;
    radar_global.z = radar_rot.z + radarFrame.z;

    std::cout << "Radar point in global (quat): (" << radar_global.x << ", " << radar_global.y << ", " << radar_global.z << ")\n";

    return 0;
}
