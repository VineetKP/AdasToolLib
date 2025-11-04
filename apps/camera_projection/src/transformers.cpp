#include "transformers.hpp"
#include <math.h>
namespace AdasTools {


void pose6ToMatrix(const double pose6[6], double outMat16[16])
{
    double x = pose6[0]; double y = pose6[1]; double z = pose6[2];
    double roll = pose6[3]; double pitch = pose6[4]; double yaw = pose6[5];
    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw); double sy = sin(yaw);
    double r00 = cy*cp; double r01 = cy*sp*sr - sy*cr; double r02 = cy*sp*cr + sy*sr;
    double r10 = sy*cp; double r11 = sy*sp*sr + cy*cr; double r12 = sy*sp*cr - cy*sr;
    double r20 = -sp; double r21 = cp*sr; double r22 = cp*cr;
    outMat16[0]=r00; outMat16[1]=r01; outMat16[2]=r02; outMat16[3]=x;
    outMat16[4]=r10; outMat16[5]=r11; outMat16[6]=r12; outMat16[7]=y;
    outMat16[8]=r20; outMat16[9]=r21; outMat16[10]=r22; outMat16[11]=z;
    outMat16[12]=0.0; outMat16[13]=0.0; outMat16[14]=0.0; outMat16[15]=1.0;
}

Point3 projectPointCamera(const Point3 &pointLocal, const double extrinsic[16], const double intrinsic[9])
{
    double X = pointLocal.x; double Y = pointLocal.y; double Z = pointLocal.z;
    double cx = extrinsic[3]; double cy = extrinsic[7]; double cz = extrinsic[11];
    double x_cam = extrinsic[0]*X + extrinsic[1]*Y + extrinsic[2]*Z + cx;
    double y_cam = extrinsic[4]*X + extrinsic[5]*Y + extrinsic[6]*Z + cy;
    double z_cam = extrinsic[8]*X + extrinsic[9]*Y + extrinsic[10]*Z + cz;
    Point3 out;
    if (z_cam == 0.0) { out.x=0; out.y=0; out.z=0; return out; }
    double fx = intrinsic[0]; double s = intrinsic[1]; double cx_i = intrinsic[2];
    double fy = intrinsic[4]; double cy_i = intrinsic[5];
    double u = (fx * x_cam + s * y_cam) / z_cam + cx_i;
    double v = (fy * y_cam) / z_cam + cy_i;
    out.x=u; 
    out.y=v; 
    out.z=z_cam; 
    return out;
}

Pose projectPointCamera(const Pose &pointLocal, const double extrinsic[16], const double intrinsic[9])
{
    Point3 p{pointLocal.x, pointLocal.y, pointLocal.z};
    Point3 pix = projectPointCamera(p, extrinsic, intrinsic);
    Pose out; 
    out.x = pix.x; 
    out.y = pix.y; 
    out.z = pix.z; 
    return out;
}

bool isPointInFrontOfCamera(double z_cam) {
    return z_cam > 0.0;
}

bool isPixelInImage(double u, double v, int width, int height) {
    return u >= 0 && u < width && v >= 0 && v < height;
}

} // namespace AdasTools
