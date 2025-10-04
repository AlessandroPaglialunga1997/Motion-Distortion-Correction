#include "TrajectoryExtractor.hpp"

std::vector<TrajectoryPoint> TrajectoryExtractor::extractTrajectoryPoints(std::vector<IMUMeasurment> imuMeasurements){
    PoseEstimator poseEstimator;
    std::vector<TrajectoryPoint> trajectory;

    if (imuMeasurements.empty())
        return trajectory;

    Eigen::Vector3d currLinearPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d currLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d currOrientation = Eigen::Matrix3d::Identity();
    TrajectoryPoint currPose;
    currPose.position = currLinearPosition;
    currPose.orientation = currOrientation;
    double deltaT;
    for (auto it = imuMeasurements.begin(); it != imuMeasurements.end(); ++it) {
        const IMUMeasurment& currIMUMeasurement = *it;
        if (std::next(it) != imuMeasurements.end()){
            const IMUMeasurment& nextIMUMeasurement = *std::next(it);
            deltaT = nextIMUMeasurement.timestamp - currIMUMeasurement.timestamp;
        }
        currPose.timestamp = currIMUMeasurement.timestamp;

        trajectory.push_back(currPose);

        Eigen::Vector3d nextLinPosition = poseEstimator.getNextLinearPosition(currLinearPosition, 
                                                                              currLinearVelocity, 
                                                                              currOrientation,
                                                                              deltaT,
                                                                              currIMUMeasurement.accelerometer);
        currPose.position = nextLinPosition;
        currLinearPosition = nextLinPosition;

        Eigen::Vector3d nextLinVelocity = poseEstimator.getNextLinearVelocity(currLinearVelocity, 
                                                                              currOrientation,
                                                                              deltaT,
                                                                              currIMUMeasurement.accelerometer);

        currLinearVelocity = nextLinVelocity;

        Eigen::Matrix3d nextOrientation = poseEstimator.getNextOrientation(currOrientation,
                                                                           deltaT,
                                                                           currIMUMeasurement.gyroscope);
        currPose.orientation = nextOrientation;
        currOrientation = nextOrientation;
    }

    return trajectory;
}

