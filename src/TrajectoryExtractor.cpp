#include "TrajectoryExtractor.hpp"
#include <iostream>

std::vector<TrajectoryPoint> TrajectoryExtractor::extractTrajectoryPoints(std::vector<IMUMeasurment> imuMeasurements){
    std::vector<TrajectoryPoint> trajectory;

    if (imuMeasurements.empty())
        return trajectory;

    Eigen::Vector3d currLinPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d currLinVelocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d currOrientation = Eigen::Matrix3d::Identity();
    double deltaT;
    for (auto it = imuMeasurements.begin(); it != imuMeasurements.end(); ++it) {
        const IMUMeasurment& currIMUMeasurement = *it;
        if (std::next(it) != imuMeasurements.end()){
            const IMUMeasurment& nextIMUMeasurement = *std::next(it);
            deltaT = nextIMUMeasurement.timestamp - currIMUMeasurement.timestamp;
        }

        trajectory.push_back({currIMUMeasurement.timestamp, currLinPosition, currOrientation});

        // Runge-Kutta 4th order integration
        Eigen::Vector3d a1 = currOrientation * currIMUMeasurement.accelerometer;
        Eigen::Vector3d v1 = currLinVelocity;

        Eigen::Vector3d a2 = currOrientation * currIMUMeasurement.accelerometer;
        Eigen::Vector3d v2 = currLinVelocity + deltaT * a1;

        Eigen::Vector3d a3 = currOrientation * currIMUMeasurement.accelerometer;
        Eigen::Vector3d v3 = currLinVelocity + deltaT * a2;

        Eigen::Vector3d a4 = currOrientation * currIMUMeasurement.accelerometer;
        Eigen::Vector3d v4 = currLinVelocity + deltaT * a3;

        currLinVelocity += (deltaT / 6.0) * (a1 + 2.0 * a2 + 2.0 * a3 + a4);
        currLinPosition += (deltaT / 6.0) * (v1 + 2.0 * v2 + 2.0 * v3 + v4);

        // Orientation update using exponential map
        Eigen::Vector3d omega = currIMUMeasurement.gyroscope;
        double omega_norm = omega.norm();
        Eigen::Matrix3d omega_skew;
        omega_skew << 0, -omega.z(), omega.y(),
                      omega.z(), 0, -omega.x(),
                      -omega.y(), omega.x(), 0;

        Eigen::Matrix3d R_inc = Eigen::Matrix3d::Identity();
        if (omega_norm > 1e-5) {
            double theta = omega_norm * deltaT;
            Eigen::Matrix3d omega_skew_norm = omega_skew / omega_norm;
            R_inc = Eigen::Matrix3d::Identity()
                    + sin(theta) * omega_skew_norm
                    + (1 - cos(theta)) * omega_skew_norm * omega_skew_norm;
        }

        currOrientation = currOrientation * R_inc;
    }

    return trajectory;
}
