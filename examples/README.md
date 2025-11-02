# Examples

This folder contains small example programs that demonstrate the core
capabilities of the AdasToolLib library.

Files and purpose

- `advanced_transform_examples`
  - End-to-end demo showing pose→matrix conversion, matrix composition,
    computing extrinsics between sensors, and projecting a lidar point into
    a camera image using intrinsics/extrinsics. This is the most comprehensive
    example and is good for validating sensor calibration pipelines.

- `example_usage`
  - Demonstrates matrix vs quaternion rotation equivalence and round-trip
    transforms (matrix-based and quaternion-based rotation produce the same
    point when applied consistently).

- `sensor_offsets`
  - Shows converting points between camera, lidar, and radar frames and
    how small mounting offsets affect coordinates.

- `sensor_chain`
  - Chains a point through multiple sensor frames to test multi-sensor
    chaining scenarios.

- `quaternion_walk`
  - Demonstrates quaternion slerp interpolation and rotating a point by
    interpolated orientations.

How to run the advanced example

From the project root:

```bash
cmake -S . -B build -DBUILD_EXAMPLES=ON -DBUILD_TESTS=OFF
cmake --build build --target advanced_transform_examples -j
./build/advanced_transform_examples
```

Notes

- The examples use simple hard-coded numbers for brevity. Replace intrinsics
  and extrinsics with real calibration data for realistic results.
