#pragma once
#include <Eigen/Dense>

struct LiDARMeasurement {
    int id;
    double timestamp;
    Eigen::Vector3d position; 
};