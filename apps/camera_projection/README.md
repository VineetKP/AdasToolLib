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
