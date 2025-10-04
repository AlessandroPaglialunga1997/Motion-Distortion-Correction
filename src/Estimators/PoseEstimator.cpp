#include "PoseEstimator.hpp"

struct Acceleration4ThOrder {
    Eigen::Vector3d a1;
    Eigen::Vector3d a2;
    Eigen::Vector3d a3;
    Eigen::Vector3d a4;
};

struct Valocity4ThOrder {
    Eigen::Vector3d a1;
    Eigen::Vector3d a2;
    Eigen::Vector3d a3;
    Eigen::Vector3d a4;
};

Eigen::Vector3d PoseEstimator::getNextLinearVelocity(Eigen::Vector3d currLinearVelocity, 
                                                     Eigen::Matrix3d currOrientation,
                                                     double deltaT,
                                                     Eigen::Vector3d accelerometer){
    Eigen::Vector3d nextLinearVelocity;
    // Runge-Kutta 4th order integration
    Eigen::Vector3d a1 = currOrientation * accelerometer;
    Eigen::Vector3d a2 = currOrientation * accelerometer;
    Eigen::Vector3d a3 = currOrientation * accelerometer;
    Eigen::Vector3d a4 = currOrientation * accelerometer;
    
    nextLinearVelocity = currLinearVelocity + (deltaT / 6.0) * (a1 + 2.0 * a2 + 2.0 * a3 + a4);
    return nextLinearVelocity;
}

Eigen::Vector3d PoseEstimator::getNextLinearPosition(Eigen::Vector3d currLinearPosition, 
                                                     Eigen::Vector3d currLinearVelocity, 
                                                     Eigen::Matrix3d currOrientation,
                                                     double deltaT,
                                                     Eigen::Vector3d accelerometer){
    Eigen::Vector3d nextLinearPosition;
    // Runge-Kutta 4th order integration
    Eigen::Vector3d a1 = currOrientation * accelerometer;
    Eigen::Vector3d v1 = currLinearVelocity;

    Eigen::Vector3d a2 = currOrientation * accelerometer;
    Eigen::Vector3d v2 = currLinearVelocity + deltaT * a1;

    Eigen::Vector3d a3 = currOrientation * accelerometer;
    Eigen::Vector3d v3 = currLinearVelocity + deltaT * a2;

    Eigen::Vector3d a4 = currOrientation * accelerometer;
    Eigen::Vector3d v4 = currLinearVelocity + deltaT * a3;

    nextLinearPosition = currLinearPosition + (deltaT / 6.0) * (v1 + 2.0 * v2 + 2.0 * v3 + v4);;
    return nextLinearPosition;
}

Eigen::Matrix3d PoseEstimator::getNextOrientation(Eigen::Matrix3d currOrientation,
                                                  double deltaT,
                                                  Eigen::Vector3d gyroscope){
    Eigen::Matrix3d nextOrientation;
    // Orientation update using exponential map
    Eigen::Vector3d omega = gyroscope;
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

    nextOrientation = currOrientation * R_inc;
    return nextOrientation;
}
