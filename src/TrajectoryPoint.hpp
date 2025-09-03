#pragma once
#include <Eigen/Dense>

struct TrajectoryPoint {
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Matrix3d orientation; 
};
