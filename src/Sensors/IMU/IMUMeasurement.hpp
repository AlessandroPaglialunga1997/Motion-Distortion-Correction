#pragma once
#include <Eigen/Dense>

struct IMUMeasurment {
    double timestamp;
    // following acceleration must be applied with (timestamp) orientation
    Eigen::Vector3d accelerometer; // constant linear acceleration from timestamp to timestamp+deltaT that will bring the robot to the (timestamp+deltaT) position
    Eigen::Vector3d gyroscope;     // constant angula acceleration from timestamp to timestamp+deltaT that will bring the robot to the (timestamp+deltaT) orientation
};
