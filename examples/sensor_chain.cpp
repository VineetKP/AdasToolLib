/* *******************************************************************************
 * File: examples/sensor_chain.cpp
 * Description: Chain a point through multiple sensor frames (camera -> global -> radar -> lidar)
 * *******************************************************************************/

#include <iostream>
#include "helpers.hpp"
#include "transformers.hpp"
#include "quaternion.hpp"

using namespace AdasTools;

int main()
{
    Point3 p_cam = {0.3, 0.0, 1.0};
    Frame3D cam = {1.0, 0.2, 1.2, 0.0, 0.0, 0.1};
    Frame3D radar = {0.5, -0.1, 0.6, 0.0, 0.0, -0.05};
    Frame3D lidar = {0.8, 0.0, 0.7, 0.02, 0.01, 0.02};

    // camera -> global
    Point3 cam_g = localToGlobal(p_cam, cam);
    // global -> radar local
    Point3 cam_in_radar = globalToLocal(cam_g, radar);
    // radar local -> global
    Point3 radar_g = localToGlobal(cam_in_radar, radar);
    // radar global -> lidar local
    Point3 in_lidar = globalToLocal(radar_g, lidar);

    std::cout << "Camera in global: ("<<cam_g.x<<","<<cam_g.y<<","<<cam_g.z<<")\n";
    std::cout << "Camera in radar local: ("<<cam_in_radar.x<<","<<cam_in_radar.y<<","<<cam_in_radar.z<<")\n";
    std::cout << "Camera in lidar local via radar: ("<<in_lidar.x<<","<<in_lidar.y<<","<<in_lidar.z<<")\n";

    return 0;
}
