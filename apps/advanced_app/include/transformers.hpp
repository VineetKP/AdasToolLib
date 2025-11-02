#pragma once
#include "helpers.hpp"

namespace AdasTools {

struct Point3;
struct Pose;
struct Frame3D;

Point3 rotatePosition(const Point3 &p, double roll, double pitch, double yaw);
Point3 translatePosition(const Point3 &p, double dx, double dy, double dz);

Pose localToGlobalFromMatrix(const Pose &vehiclePose, const Pose &sensorPose);
Pose globalToLocalFromMatrix(const Pose &vehiclePose, const Pose &sensorGlobalPose);

void pose6ToMatrix(const double pose6[6], double outMat16[16]);
void poseToMatrix(const Pose &pose, double outMat16[16]);

Pose localToGlobal(const Pose &localPos, const Frame3D &frame);

} // namespace AdasTools
