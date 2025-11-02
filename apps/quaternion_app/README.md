# quaternion_app

Small demo showing quaternion SLERP and rotating a Pose. Build and run:

cmake -S . -B build
cmake --build build --target quaternion_app -- -j
./build/quaternion_app 45 20

Run tests:
cmake --build build --target test_quaternion -- -j
ctest --test-dir build -V

quaternion_app
=============

This small demo builds a self-contained quaternion example that demonstrates
spherical linear interpolation (SLERP) between two orientations.

Build
-----
From the repository root:

```
cmake -S . -B build
cmake --build build --target quaternion_app -- -j 4
```

Run
---

```
./build/apps/quaternion_app/quaternion_app
```

Usage: you can pass yaw (degrees) and steps, for example:

```
./build/apps/quaternion_app/quaternion_app 45 20
```


What it does
------------
- Constructs two quaternions (identity and 90° yaw)
- Interpolates between them with `slerp` for t in [0,1]
- Prints the quaternion components for each t

Notes
-----
- The app is standalone: all headers and sources required live under
  `apps/quaternion_app` so you can copy the folder out of the repo.
- If you want a different demo (rotate a 3D point or write output), tell me
  and I can update the main accordingly.
