#include "IMUMeasurementsExtractor.hpp"
#include <iostream>

std::vector<IMUMeasurment> IMUMeasurementsExtractor::extractIMUMeasurments(std::vector<TrajectoryPoint> trajectory){
    std::vector<IMUMeasurment> imuMeasurements;
    IMUMeasurment currIMUMeasurement;

    TrajectoryPoint firstPose = *trajectory.begin();
    TrajectoryPoint secondPose = *std::next(trajectory.begin());
    currIMUMeasurement.timestamp = firstPose.timestamp;
    currIMUMeasurement.gyroscope = estimateAngularVelocity(firstPose, secondPose);
    double deltaT = secondPose.timestamp - firstPose.timestamp;
    Eigen::Vector3d firstWorldLinearAcc = (secondPose.position - firstPose.position) / (deltaT * deltaT);
    currIMUMeasurement.accelerometer = firstWorldLinearAcc;
    imuMeasurements.push_back(currIMUMeasurement);

    for (auto it = std::next(trajectory.begin()); std::next(it) != trajectory.end(); ++it) {
        const TrajectoryPoint& prevPose = *std::prev(it);
        const TrajectoryPoint& currPose = *it;
        const TrajectoryPoint& nextPose = *std::next(it);

        currIMUMeasurement.timestamp = currPose.timestamp;
        currIMUMeasurement.gyroscope = estimateAngularVelocity(currPose, nextPose);
        currIMUMeasurement.accelerometer = estimateLinearAcceleration(prevPose, currPose, nextPose);
        imuMeasurements.push_back(currIMUMeasurement);
    }

    TrajectoryPoint lastPose = *(--trajectory.end());
    currIMUMeasurement.timestamp = lastPose.timestamp;
    imuMeasurements.push_back(currIMUMeasurement);

    return imuMeasurements;
}

Eigen::Vector3d IMUMeasurementsExtractor::estimateAngularVelocity(TrajectoryPoint currPose, TrajectoryPoint nextPose){
    double deltaT = nextPose.timestamp - currPose.timestamp;
    Eigen::Matrix3d R_rel = currPose.orientation.transpose() * nextPose.orientation;
    Eigen::AngleAxisd aa(R_rel);
    Eigen::Vector3d omega = aa.axis() * aa.angle() / deltaT;
    return omega;
}

Eigen::Vector3d IMUMeasurementsExtractor::estimateLinearAcceleration(TrajectoryPoint prevPose,
                                                                    TrajectoryPoint currPose,
                                                                    TrajectoryPoint nextPose){
    double prevDeltaT = currPose.timestamp - prevPose.timestamp;
    double currDeltaT = nextPose.timestamp - currPose.timestamp;
    Eigen::Vector3d prevVelocity = (currPose.position - prevPose.position) / prevDeltaT;
    Eigen::Vector3d currVelocity = (nextPose.position - currPose.position) / currDeltaT;
    Eigen::Vector3d worldLinearAcc = (currVelocity - prevVelocity) / currDeltaT;
    Eigen::Vector3d bodyLinearAcc = currPose.orientation.transpose() * worldLinearAcc;
    return bodyLinearAcc;
}
