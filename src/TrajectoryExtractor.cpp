#include "TrajectoryExtractor.hpp"
#include <iostream>

std::vector<TrajectoryPoint> TrajectoryExtractor::extractTrajectoryPoints(std::vector<IMUMeasurment> imuMeasurements){
    std::vector<TrajectoryPoint> trajectory;
    
    if (imuMeasurements.empty()) 
    return trajectory;

    Eigen::Vector3d currLinPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d currLinVelocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d currOrientation = Eigen::Matrix3d::Identity();; 

    for (auto it = imuMeasurements.begin(); std::next(it) != imuMeasurements.end(); ++it) {
        const IMUMeasurment& currIMUMeasurement = *it;
        const IMUMeasurment& nextIMUMeasurement = *std::next(it);
        double deltaT = nextIMUMeasurement.timestamp - currIMUMeasurement.timestamp;
        trajectory.push_back({currIMUMeasurement.timestamp, currLinPosition, currOrientation});

        // Update Position for next iteration
        Eigen::Vector3d currLinAcceleration = currOrientation * currIMUMeasurement.accelerometer;
        currLinVelocity += currLinAcceleration * deltaT;
        currLinPosition += currLinVelocity * deltaT;

        // Update Orientation for next iteration
        Eigen::Matrix3d omega_skew;
        omega_skew <<                      0, -currIMUMeasurement.gyroscope.z(),  currIMUMeasurement.gyroscope.y(),
                      currIMUMeasurement.gyroscope.z(),                       0, -currIMUMeasurement.gyroscope.x(),
                     -currIMUMeasurement.gyroscope.y(),  currIMUMeasurement.gyroscope.x(),                       0;

        currOrientation = currOrientation * (Eigen::Matrix3d::Identity() + omega_skew * deltaT);
    }

    return trajectory;
}