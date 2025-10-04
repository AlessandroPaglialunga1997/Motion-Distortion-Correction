#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace TrajectoriesConstants{
    // Common Constants
    constexpr double GRAVITY_MAGNITUDE = 9.81;
    inline const Eigen::Vector3d GRAVITY_VECTOR{0.0, 0.0, -GRAVITY_MAGNITUDE};
    const double trajectoryDeltaT = 0.1;    // seconds
    const double trajectoryDuration = 10;   // seconds

    // Helical Trajectory
    const double helicalRadius = 3.2;       // meters
    const double helicalHeight = 4.8;       // meters
    const Eigen::Vector3d helicalPositionOffset(-helicalRadius, 0.0, 0.0); // meters

    // Sphere
    const double sphereRadius = 0.5;                      // meters
    const double sphereAngularStep = (M_PI / 90) * 30;    // radians
    const Eigen::Vector3d spherePositionOffset(0.0, 0.0, 0.0);;
}
