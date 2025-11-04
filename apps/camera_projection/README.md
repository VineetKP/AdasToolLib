# camera_projection

Build and run from repo root:

cmake -S . -B build
cmake --build build --target camera_projection -- -j
./build/apps/camera_projection/camera_projection
camera_projection
==================

Standalone app that computes extrinsics and projects a sample lidar pose into camera image coordinates.

Build
-----

```
cmake -S . -B build
cmake --build build --target camera_projection -- -j 4
```

Run
---

```
./build/apps/camera_projection/camera_projection
```
# ============Test case Calculation=================

Pose cameraPose = {-2.0, 0.0, 1.5, 0.0, deg2rad(90), 0.0};

// Point in vehicle frame
Pose p_local = {10.0, 1.0, 0.0, 0.0, 0.0, 0.0};

// Intrinsics
double K[9] = {
    800.0, 0.0, 320.0,
    0.0, 800.0, 240.0,
    0.0, 0.0, 1.0
};

1: Compute Rotation Matrix
roll = 0, pitch = 90° = π/2, yaw = 0

cr = cos(0) = 1,  sr = sin(0) = 0
cp = cos(90°) = 0, sp = sin(90°) = 1
cy = cos(0) = 1,  sy = sin(0) = 0

Using your formulas:
r00 = cy*cp = 1*0 = 0
r01 = cy*sp*sr - sy*cr = 1*1*0 - 0*1 = 0
r02 = cy*sp*cr + sy*sr = 1*1*1 + 0*0 = 1
r10 = sy*cp = 0*0 = 0
r11 = sy*sp*sr + cy*cr = 0*1*0 + 1*1 = 1
r12 = sy*sp*cr - cy*sr = 0*1*1 - 1*0 = 0
r20 = -sp = -1
r21 = cp*sr = 0*0 = 0
r22 = cp*cr = 0*1 = 0

M_cam_in_vehicle:
[ 0  0  1  -2.0 ]
[ 0  1  0   0.0 ]
[-1  0  0   1.5 ]
[ 0  0  0   1.0 ]

2: compute Extrinsic(Inverse)
R = [ 0  0  1 ]
    [ 0  1  0 ]
    [-1  0  0 ]

R^T = [ 0  0 -1 ]
      [ 0  1  0 ]
      [ 1  0  0 ]

t = [-2.0, 0.0, 1.5]

-R^T * t:
  x: -(0*(-2.0) + 0*0.0 + (-1)*1.5) = -(-1.5) = 1.5
  y: -(0*(-2.0) + 1*0.0 + 0*1.5) = 0.0
  z: -(1*(-2.0) + 0*0.0 + 0*1.5) = -(-2.0) = 2.0

Extrinsic:
[ 0  0 -1   1.5 ]
[ 0  1  0   0.0 ]
[ 1  0  0   2.0 ]
[ 0  0  0   1.0 ]

3: Transform Point to Camera Frame
P_vehicle = [10.0, 1.0, 0.0, 1.0]^T

x_cam = 0*10.0 + 0*1.0 + (-1)*0.0 + 1.5 = 1.5  ← CHANGED!
y_cam = 0*10.0 + 1*1.0 + 0*0.0 + 0.0 = 1.0
z_cam = 1*10.0 + 0*1.0 + 0*0.0 + 2.0 = 12.0

P_camera = [1.5, 1.0, 12.0]

4: Project to Image Plane

u = (fx * x_cam + s * y_cam) / z_cam + cx
  = (800 * 1.5 + 0 * 1.0) / 12.0 + 320
  = 1200 / 12.0 + 320
  = 100 + 320
  = 420 ✓

v = (fy * y_cam) / z_cam + cy
  = (800 * 1.0) / 12.0 + 240
  = 800 / 12.0 + 240
  = 66.667 + 240
  = 306.667 ✓

depth = 12.0 ✓
```