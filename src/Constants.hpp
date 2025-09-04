#pragma once
#include <Eigen/Dense>

namespace Constants {
    
    constexpr double GRAVITY_MAGNITUDE = 9.81;

    inline const Eigen::Vector3d GRAVITY_VECTOR{0.0, 0.0, -GRAVITY_MAGNITUDE};
    // Common Trajectory variables
    const double trajectoryDeltaT = 0.1;  // seconds
    const double trajectoryDuration = 10; // seconds

    // Helical Trajectory
    const double helicalRadius = 1;       // meter
    const double helicalHeight = 2;       // meters
    const Eigen::Vector3d positionOffset(-helicalRadius, 0.0, 0.0);

}
