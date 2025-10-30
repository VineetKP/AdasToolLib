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
 # AdasToolLib

A small, dependency-minimal C++ library of ADAS helper routines: 3D transforms,
quaternion utilities, and sensor-frame helpers. The core math modules avoid the
STL and are designed to be embedded in embedded/RTOS projects.

This repository contains a production-ready library plus examples and tests.

## Quick start — build and run (Linux / macOS)

Configure, build, and run examples and tests (out-of-tree build):

```bash
cmake -S . -B build -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON
cmake --build build -j

# Run example apps
./build/adas_tools_app
./build/example_usage

# Run tests
./build/tests
```

If you only want examples and not tests, set `-DBUILD_TESTS=OFF` when configuring.

## Project layout
- `include/` — public headers (POD types and function declarations)
- `src/` — implementation sources
- `examples/` — small example programs demonstrating usage
- `tests/` — unit tests and micro-benchmarks
- `Documentation/` — extended guides and visualizations
- `build/` — out-of-tree CMake build directory (ignored by `.gitignore`)

## Important functions (short reference)
These are the core helpers provided by the library and useful entry points.

Transformers (coordinate frame helpers)
- `AdasTools::Position` — POD { x,y,z }
- `AdasTools::Frame3D` — POD { x,y,z, roll, pitch, yaw }
- `AdasTools::scalePosition(const Position &p, double s)` — uniform scale
- `AdasTools::rotatePosition(const Position &p, double roll, double pitch, double yaw)` — rotate about origin using R = Rz * Ry * Rx
- `AdasTools::translatePosition(const Position &p, double dx, double dy, double dz)` — translate
- `AdasTools::localToGlobal(const Position &localPos, const Frame3D &frame)` — rotate then translate
- `AdasTools::globalToLocal(const Position &globalPos, const Frame3D &frame)` — invert transform (translate then apply R^T)

Quaternion utilities
- `AdasTools::Quaternion` — POD { w,x,y,z }
- `AdasTools::quaternionFromRPY(double roll, double pitch, double yaw)` — construct quaternion
- `AdasTools::normalizeQuaternion(const Quaternion &q)` — normalize to unit length
- `AdasTools::rotateByQuaternion(const Quaternion &q, const Position &p)` — rotate vector
- `AdasTools::multiplyQuaternion(const Quaternion &a, const Quaternion &b)` — compose rotations
- `AdasTools::slerp(const Quaternion &a, const Quaternion &b, double t)` — spherical linear interpolation (LERP fallback when angle is small)

Examples (useful targets)
- `adas_tools_app` — tiny app that prints a short message (`examples/main_app.cpp`)
- `example_usage` — demonstrates matrix vs quaternion rotation (`examples/example_usage.cpp`)
- `sensor_offsets`, `sensor_chain`, `quaternion_walk` — additional demos

## Documentation
Detailed mathematical explanations, derivations, and visualizations are
kept in `Documentation/Transformers/README.md` (includes SLERP, gimbal-lock,
code samples and SVG visuals). This keeps the root README concise.

## Contributing & tests
- Tests are simple self-contained executables in `tests/`.
- Add new public headers to `include/` and implementations to `src/`.
- When adding API, include Doxygen-style comments and a short example.

## License
See `LICENSE` (if present) or add your preferred license file.

## Examples — Quick run
Run these one-liners after building (see Quick start above). They are
designed for quick verification and to show expected short outputs.

- Build and run a tiny app

```bash
cmake -S . -B build -DBUILD_EXAMPLES=ON -DBUILD_TESTS=OFF
cmake --build build --target adas_tools_app -j
./build/adas_tools_app
# Expected: "My Own Library"
```

- Run the matrix vs quaternion example

```bash
cmake -S . -B build -DBUILD_EXAMPLES=ON -DBUILD_TESTS=OFF
cmake --build build --target example_usage -j
./build/example_usage
# Expected snippet (numeric values may vary slightly):
# p_global (matrix) = (2.35355, 1.16066, 0.7)
# p_global (quat)   = (2.35355, 1.16066, 0.7)
# round-trip equals local: (1, 0.5, 0.2)
```

- Run the tests (all tests)

```bash
cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build --target tests -j
./build/tests
# Expected: "All tests passed"
```

If you want a full rebuild of everything, use:

```bash
cmake -S . -B build -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON
cmake --build build -j
```


