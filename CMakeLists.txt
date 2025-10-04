cmake_minimum_required(VERSION 3.10)
project(MotionDistortionCorrection)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Eigen
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(src)

# Add executable with source files from src directory
add_executable(MotionDistortionCorrection
    src/main.cpp
    src/Estimators/PoseEstimator.cpp
    src/MotionDistortionRemovers/MotionDistortionRemover.cpp
    src/Readers/FileReader.cpp
    src/Sensors/LiDAR/LiDARMeasurementsGenerator.cpp
    src/Sensors/IMU/IMUMeasurementsExtractor.cpp
    src/TrajectoryTools/TrajectoryGenerator.cpp
    src/TrajectoryTools/TrajectoryExtractor.cpp
)
