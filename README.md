# AdasToolLib

ADAS Tools Library — a compact, header-first C++ library intended to collect
# AdasToolLib

ADAS Tools Library — a compact, header-first C++ library intended to collect
utility functions and helpers for Advanced Driver Assistance Systems (ADAS).

This repository provides a small, dependency-free foundation (no STL in core
math modules) for coordinate transforms, quaternion math, and sensor frame
helpers. It includes examples and a minimal test runner.

## Goals
- Provide a focused namespace `AdasTools` for ADAS helper routines
- Keep public headers small and documented (Doxygen-ready)
- Offer small example programs and a self-contained test runner

## Layout
- `include/` — public headers (e.g. `helpers.hpp`, `transformers.hpp`)
- `src/` — library sources (transformers, quaternion)
- `examples/` — example programs (example_usage, main_app, sensor offsets)
- `tests/` — self-contained test runner
- `build/` — out-of-tree CMake build directory

## Build (Linux/macOS)
This project uses CMake. From the project root:

```bash
cmake -S . -B build -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON
cmake --build build -j
```

## Example binaries
- `adas_tools_app` — simple app that prints a short message (examples/main_app.cpp)
- `example_usage` — demo comparing matrix-based vs quaternion-based transforms

Run the examples:

```bash
./build/adas_tools_app
./build/example_usage
```

## Tests
Build and run the minimal test runner:

```bash
cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build --target tests -j
./build/tests
```

The test runner prints `All tests passed` on success.

## Transformer module — mathematical overview

The Transformers module implements 3D coordinate frame math used across ADAS
sensors (camera, radar, lidar, ultrasonic). The library provides plain-old-data
types `AdasTools::Position` and `AdasTools::Frame3D` and functions to scale,
rotate, translate and convert coordinates between local and global frames.

Notation
- Position p = $(x, y, z)^T$
- Frame origin: $o = (x_0, y_0, z_0)^T$
- Orientation: roll $(\phi)$, pitch $(\theta)$, yaw $(\psi)$ in radians

Rotation matrices
We use the intrinsic rotation sequence $R = R_z(\psi) R_y(\theta) R_x(\phi)$ where

$$
R_x(\phi)=\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi & \cos\phi
\end{bmatrix},
\quad
R_y(\theta)=\begin{bmatrix}
\cos\theta & 0 & \sin\theta \\
0 & 1 & 0 \\
-\sin\theta & 0 & \cos\theta
\end{bmatrix},
\quad
R_z(\psi)=\begin{bmatrix}
\cos\psi & -\sin\psi & 0 \\
\sin\psi & \cos\psi & 0 \\
0 & 0 & 1
\end{bmatrix}.
$$

The combined rotation $R = R_z(\psi) R_y(\theta) R_x(\phi)$ maps a local
vector to the global frame by

$$
v_{global} = R \, v_{local}
$$

Frame transforms
- Local to Global: $v_{global} = R \, v_{local} + o$
- Global to Local: $v_{local} = R^T (v_{global} - o)$ (since $R^{-1}=R^T$)

Functions (short)
- `scalePosition(p, s)` — uniform scaling by `s`
- `rotatePosition(p, roll, pitch, yaw)` — rotate the position about origin
- `translatePosition(p, dx, dy, dz)` — translate by (dx,dy,dz)
- `localToGlobal(localPos, frame)` — rotate then translate
- `globalToLocal(globalPos, frame)` — translate then apply inverse rotation

C++ example (matrix-based round-trip)

```cpp
#include "helpers.hpp"
#include "transformers.hpp"
using namespace AdasTools;

int main() {
  Position p_local = {1.0, 0.5, 0.2};
  Frame3D frame = {2.0, 0.1, 0.5, 0.0, 0.0, 0.785398};
  Position p_global = localToGlobal(p_local, frame);
  Position p_back = globalToLocal(p_global, frame);
  // p_back ~= p_local
}
```

## Quaternion utilities

Quaternions provide a compact, numerically stable representation for 3D
rotations and are useful for composition and smooth interpolation (slerp).

Representation
- $q = w + x i + y j + z k$ where $w$ is scalar and $(x,y,z)$ is the vector part.
  Unit quaternions ($\lVert q \rVert = 1$) represent rotations.

Conversion from roll-pitch-yaw $(\phi,\theta,\psi)$ to quaternion:

$$
q = \begin{bmatrix} w \\ x \\ y \\ z \end{bmatrix} =
\begin{bmatrix}
\cos(\tfrac{\phi}{2})\cos(\tfrac{\theta}{2})\cos(\tfrac{\psi}{2}) + \sin(\tfrac{\phi}{2})\sin(\tfrac{\theta}{2})\sin(\tfrac{\psi}{2})\\
\sin(\tfrac{\phi}{2})\cos(\tfrac{\theta}{2})\cos(\tfrac{\psi}{2}) - \cos(\tfrac{\phi}{2})\sin(\tfrac{\theta}{2})\sin(\tfrac{\psi}{2})\\
\cos(\tfrac{\phi}{2})\sin(\tfrac{\theta}{2})\cos(\tfrac{\psi}{2}) + \sin(\tfrac{\phi}{2})\cos(\tfrac{\theta}{2})\sin(\tfrac{\psi}{2})\\
\cos(\tfrac{\phi}{2})\cos(\tfrac{\theta}{2})\sin(\tfrac{\psi}{2}) - \sin(\tfrac{\phi}{2})\sin(\tfrac{\theta}{2})\cos(\tfrac{\psi}{2})
\end{bmatrix}
$$

API (short)
- `quaternionFromRPY(roll,pitch,yaw)` — create quaternion from RPY
- `normalizeQuaternion(q)` — normalize to unit quaternion
- `rotateByQuaternion(q,p)` — apply quaternion rotation to a `Position`
- `multiplyQuaternion(q1,q2)` — compose rotations
- `slerp(a,b,t)` — spherical linear interpolation between two quaternions

Example: compare matrix vs quaternion rotation (see `examples/example_usage.cpp`)

## Next steps and notes
- Consider adding quaternion slerp tests and edge-case checks for near-identical
  quaternions (already handled in the implementation with a LERP fallback).
- If you want richer examples (camera projection, lidar point clouds), I can
  add them in the `examples/` folder.
		sensorFrame.z = 0.5;    // 0.5 m above ground

		sensorFrame.roll  = 0.0;

