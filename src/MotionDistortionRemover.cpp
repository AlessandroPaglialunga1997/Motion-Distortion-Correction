#include "MotionDistortionRemover.hpp"

std::vector<LiDARMeasurement> MotionDistortionRemover::removeDistortionUsingIMU(std::vector<LiDARMeasurement> distortedLiDARMeasurements,
                                                                                std::vector<IMUMeasurment> imuMeasurements){
    std::vector<LiDARMeasurement> correctedLiDARMeasurements;                                                                                    
    for (auto imuPointer = imuMeasurements.begin(); imuPointer != imuMeasurements.end(); ++imuPointer){
        const IMUMeasurment& currIMUMeasure = *imuPointer;
        const double currTimestamp = currIMUMeasure.timestamp;
        Eigen::Vector3d currAccelerometer = currIMUMeasure.accelerometer;
        Eigen::Vector3d currGyroscope = currIMUMeasure.gyroscope;
        // obtain curr robot position
        Eigen::Vector3d currRobotLinPosition = Eigen::Vector3d::Zero();
        // obtain curr robot orientation
        Eigen::Matrix3d currRobotOrientation = Eigen::Matrix3d::Identity();
        for (auto lidarPointer = distortedLiDARMeasurements.begin(); lidarPointer != distortedLiDARMeasurements.end(); ++lidarPointer){
            const LiDARMeasurement& currLiDARMeasure = *lidarPointer;
            if(currLiDARMeasure.timestamp > currTimestamp)
                break;
            LiDARMeasurement correctedCurrLiDARMeasure;
            correctedCurrLiDARMeasure.id = currLiDARMeasure.id;
            correctedCurrLiDARMeasure.timestamp = currLiDARMeasure.timestamp;
            correctedCurrLiDARMeasure.position = currRobotOrientation * currLiDARMeasure.position - currRobotLinPosition;
            correctedLiDARMeasurements.push_back(correctedCurrLiDARMeasure);
        }
    }
    return correctedLiDARMeasurements;
}