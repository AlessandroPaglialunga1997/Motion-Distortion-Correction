#include "IMUMeasurementsExtractor.hpp"

// Useful comments:
// 1. to undestrand the used indecies remember
    //                          [       0, -omega_z,  omega_y
    //  skew_symmetric(omega) =   omega_z,        0, -omega_x
    //                           -omega_y,  omega_x,        0]

std::vector<IMUMeasurment> IMUMeasurmentsExtractor::extractIMUMeasurments(std::vector<TrajectoryPoint> trajectory){
    std::vector<IMUMeasurment> imuMeasurements;
    
    IMUMeasurment currIMUMeasurement;
    
    // first mesurement
    TrajectoryPoint virtualBeforeFirstPose = {-1.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity()};
    TrajectoryPoint firstPose = *trajectory.begin();
    TrajectoryPoint secondPose = *std::next(trajectory.begin());
    currIMUMeasurement.timestamp = firstPose.timestamp;
    currIMUMeasurement.gyroscope = estimateAngularVelocity(firstPose, secondPose);
    currIMUMeasurement.accelerometer = estimateLinearAcceleration(virtualBeforeFirstPose, firstPose, secondPose);
    imuMeasurements.push_back(currIMUMeasurement);

    // middle mesurements
    for (auto it = std::next(trajectory.begin()); std::next(it) != trajectory.end(); ++it) {
        const TrajectoryPoint& prevPose = *std::prev(it);
        const TrajectoryPoint& currPose = *it;
        const TrajectoryPoint& nextPose = *std::next(it);

        currIMUMeasurement.timestamp = currPose.timestamp;
        currIMUMeasurement.gyroscope = estimateAngularVelocity(currPose, nextPose);
        currIMUMeasurement.accelerometer = estimateLinearAcceleration(prevPose, currPose, nextPose);

        imuMeasurements.push_back(currIMUMeasurement);
    }
    // last mesurement: copy of last currIMUMeasurement except for the time stamp
    TrajectoryPoint lastPose = *(--trajectory.end());
    currIMUMeasurement.timestamp = lastPose.timestamp;
    imuMeasurements.push_back(currIMUMeasurement);

    return imuMeasurements;
}

Eigen::Vector3d IMUMeasurmentsExtractor::estimateAngularVelocity(TrajectoryPoint currPose, TrajectoryPoint nextPose){
    double currDeltaT = nextPose.timestamp - currPose.timestamp;
    Eigen::Matrix3d R_dot = (nextPose.orientation - currPose.orientation) / currDeltaT;
    Eigen::Matrix3d skewGyroscope = currPose.orientation.transpose() * R_dot;
    Eigen::Vector3d angularVelocity( skewGyroscope(2, 1), skewGyroscope(0, 2), skewGyroscope(1, 0) ); // Check useful copmment 1.
    return angularVelocity;
}

Eigen::Vector3d IMUMeasurmentsExtractor::estimateLinearAcceleration(TrajectoryPoint prevPose, 
                                           TrajectoryPoint currPose, 
                                           TrajectoryPoint nextPose){
    double prevDeltaT = currPose.timestamp - prevPose.timestamp;
    double currDeltaT = nextPose.timestamp - currPose.timestamp;
    Eigen::Vector3d prevVelocity = (currPose.position - prevPose.position) / prevDeltaT;
    Eigen::Vector3d currVelocity = (nextPose.position - currPose.position) / currDeltaT;
    Eigen::Vector3d worldLinearAcc = (currVelocity - prevVelocity) / currDeltaT;
    Eigen::Vector3d bodyLinearAcc = currPose.orientation.transpose() * (worldLinearAcc - Constants::GRAVITY_VECTOR);
    return bodyLinearAcc;
}