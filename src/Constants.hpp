#pragma once
#include <Eigen/Dense>

namespace Constants {
    
    constexpr double GRAVITY_MAGNITUDE = 9.81;

    inline const Eigen::Vector3d GRAVITY_VECTOR{0.0, 0.0, -GRAVITY_MAGNITUDE};
    double const helicalRadius = 1;       // 1 meter 
    double const helicalHeight = 2;       //  2 meters
    double const trajectoryDuration = 10; // 10 seconds
    double const trajectoryDeltaT = 0.1;  // 100 milli-seconds
}
