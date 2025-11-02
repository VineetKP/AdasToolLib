#pragma once
#include "helpers.hpp"
namespace AdasTools {
void pose6ToMatrix(const double pose6[6], double outMat16[16]);
void poseToMatrix(const Pose &pose, double outMat16[16]);
Point3 projectPointCamera(const Point3 &pointLocal, const double extrinsic[16], const double intrinsic[9]);
Pose projectPointCamera(const Pose &pointLocal, const double extrinsic[16], const double intrinsic[9]);
}
