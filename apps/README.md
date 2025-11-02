# Apps

This folder contains three minimal, self-contained demo apps you can copy out of the repo:

- advanced_app: demonstrates Pose -> matrix transforms and local->global conversions.
- camera_projection: shows projecting a 3D point into camera image coordinates.
- quaternion_app: demonstrates quaternion SLERP and rotating a Pose (includes a unit test).

Build (from repo root):

cmake -S . -B build
cmake --build build --target advanced_app -- -j
cmake --build build --target camera_projection -- -j
cmake --build build --target quaternion_app -- -j
Apps
====

This folder contains several minimal, standalone demo apps that exercise parts of the
ADAS tools library. Each app is self-contained under `apps/<appname>` with its own
`include/` and `src/` so you can copy a folder out of the repo and build it independently.

Available apps:
- advanced_app - pose compose/point transform demo
- camera_projection - camera extrinsic/intrinsic projection demo
- quaternion_app - quaternion SLERP + Pose-rotation demo

Build all apps:

```
cmake -S . -B build
cmake --build build --target all -- -j 4
```

Or build individual apps by target name, for example:

```
cmake --build build --target quaternion_app -- -j 4
```
