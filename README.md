# Motion Distortion Correction

This project is a small C++ simulation of LiDAR motion distortion correction using IMU-based pose estimation. It generates a synthetic robot trajectory, simulates a distorted sphere point cloud as seen by a moving LiDAR sensor, and applies a simple correction pipeline to recover a corrected point cloud.

## Overview

The application demonstrates the effect of motion distortion in LiDAR measurements when the sensor is mounted on a moving robot. The workflow is:

1. Generate a helical robot trajectory.
2. Generate a distorted spherical point cloud that appears to move because the sensor is moving.
3. Extract IMU-like measurements from the trajectory.
4. Use those measurements to estimate and compensate for the robot motion.
5. Write the resulting point clouds to disk for further analysis and plotting.

## What the code does

- Simulates a robot moving along a helical trajectory.
- Produces synthetic LiDAR measurements for a sphere.
- Applies a distortion model based on the robot pose.
- Estimates pose updates from IMU-like accelerometer and gyroscope data.
- Writes the generated point sets to `.dat` files in the project root.

## Project structure

- [CMakeLists.txt](CMakeLists.txt) — CMake build configuration.
- [src/main.cpp](src/main.cpp) — entry point of the application.
- [src/Constants.hpp](src/Constants.hpp) — simulation constants such as trajectory and sphere parameters.
- [src/TrajectoryGenerator.cpp](src/TrajectoryGenerator.cpp) — generates the helical trajectory.
- [src/IMUMeasurementsExtractor.cpp](src/IMUMeasurementsExtractor.cpp) — derives IMU-like measurements from the trajectory.
- [src/LiDARMeasurementsGenerator.cpp](src/LiDARMeasurementsGenerator.cpp) — generates the distorted sphere measurements.
- [src/MotionDistortionRemover.cpp](src/MotionDistortionRemover.cpp) — applies the correction using IMU data.
- [src/PoseEstimator.cpp](src/PoseEstimator.cpp) — estimates linear position, velocity, and orientation.
- [plot*.gp](plotSpherePoints.gp) — GNUplot scripts for visualization.

## Requirements

- C++17 compiler
- CMake
- Eigen3

On Ubuntu/Debian, Eigen can usually be installed with:

```bash
sudo apt-get install libeigen3-dev
```

## Build

From the project root, run:

```bash
cmake -S . -B build
cmake --build build
```

## Run

Run the built executable:

```bash
./build/MotionDistortionCorrection
```

On Windows, the executable will typically be located at:

```powershell
build\MotionDistortionCorrection.exe
```

## Output files

Running the program writes these files to the project root:

- `generatedSpherePoints.dat`
- `distortedSphereTrajectory.dat`
- `correctedSphereTrajectory.dat`

These files contain point coordinates in XYZ format, one point per line.

## Visualization

The repository includes several GNUplot scripts such as:

- `plotSpherePoints.gp`
- `plotSphereTrajectory.gp`
- `plotDistortedAndCorrectedSphereTrajectory.gp`
- `plotRobotAndDistortedCorrectedSphereTrajectory.gp`

You can plot them with GNUplot, for example:

```bash
gnuplot plotDistortedAndCorrectedSphereTrajectory.gp
```

## Notes

This project is intended as a compact educational example rather than a production-ready LiDAR processing pipeline. It uses simplified motion models and synthetic data to illustrate how motion distortion can be estimated and corrected.
