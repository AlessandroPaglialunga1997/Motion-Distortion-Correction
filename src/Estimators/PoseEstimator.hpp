#pragma once
#include "Sensors/IMU/IMUMeasurement.hpp"
#include "TrajectoryTools/TrajectoryPoint.hpp"
#include <Eigen/Dense>

class PoseEstimator{

    public:
        Eigen::Matrix3d getNextOrientation(Eigen::Matrix3d currOrientation, 
                                           double deltaT,
                                           Eigen::Vector3d gyroscope);
        
        Eigen::Vector3d getNextLinearPosition(Eigen::Vector3d currLinearPosition, 
                                              Eigen::Vector3d currLinearVelocity, 
                                              Eigen::Matrix3d currOrientation,
                                              double deltaT,
                                              Eigen::Vector3d accelerometer);

        Eigen::Vector3d getNextLinearVelocity(Eigen::Vector3d currLinearVelocity, 
                                              Eigen::Matrix3d currOrientation,
                                              double deltaT,
                                              Eigen::Vector3d accelerometer);
};