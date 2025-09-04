#pragma once
#include <Eigen/Dense>

struct IMUMeasurment {
    double timestamp;
    Eigen::Vector3d accelerometer; // it can be seen as the linear acceleration of the robot
    Eigen::Vector3d gyroscope;     // it can be seen as the angular velocity of the robot
};
