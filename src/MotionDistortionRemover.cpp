#include "MotionDistortionRemover.hpp"
#include <iostream>
#include <fstream>

std::vector<LiDARMeasurement> MotionDistortionRemover::removeDistortionUsingIMU(std::vector<LiDARMeasurement> distortedLiDARMeasurements,
                                                                                std::vector<IMUMeasurment> imuMeasurements){
    std::vector<LiDARMeasurement> correctedLiDARMeasurements; 
    Eigen::Vector3d currRobotLinPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d currRobotLinVelocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d currRobotOrientation = Eigen::Matrix3d::Identity();
    PoseEstimator poseEstimator;
    auto lidarPointer = distortedLiDARMeasurements.begin();
    auto imuPointer = imuMeasurements.begin();
    double deltaT;                                                                             
    for (; imuPointer != imuMeasurements.end(); ++imuPointer){
        const IMUMeasurment& currIMUMeasure = *imuPointer;
        if (std::next(imuPointer) != imuMeasurements.end()){
            const IMUMeasurment& nextIMUMeasure = *std::next(imuPointer);
            deltaT = nextIMUMeasure.timestamp - currIMUMeasure.timestamp;
        }
        std::cout << "currIMUMeasure.timestamp: " << currIMUMeasure.timestamp << std::endl;
        for (; lidarPointer != distortedLiDARMeasurements.end(); ++lidarPointer){
            const LiDARMeasurement& currLiDARMeasure = *lidarPointer;
            if(currLiDARMeasure.timestamp > currIMUMeasure.timestamp)
                break;
            LiDARMeasurement correctedCurrLiDARMeasure;
            correctedCurrLiDARMeasure.id = currLiDARMeasure.id;
            correctedCurrLiDARMeasure.timestamp = currLiDARMeasure.timestamp;
            correctedCurrLiDARMeasure.position = currLiDARMeasure.position + currRobotLinPosition;
            
            std::cout << "distorted.timestamp: " << currLiDARMeasure.timestamp << ", "
                  << "distorted.Id = " <<  currLiDARMeasure.id << ", "
                  << "distorted.Position = " << currLiDARMeasure.position << std::endl;

            std::cout << "corrected.timestamp: " << correctedCurrLiDARMeasure.timestamp << ", "
                  << "corrected.Id = " <<  correctedCurrLiDARMeasure.id << ", "
                  << "corrected.Position = " << correctedCurrLiDARMeasure.position << std::endl;
    
            std::cout << "robot.position: " << currRobotLinPosition << std::endl;
            
            correctedLiDARMeasurements.push_back(correctedCurrLiDARMeasure);
        }
        // obtain next robot position/velocity and next robot orientation
        Eigen::Vector3d nextRobotLinPosition = poseEstimator.getNextLinearPosition(currRobotLinPosition, 
                                                                                   currRobotLinVelocity, 
                                                                                   currRobotOrientation,
                                                                                   deltaT,
                                                                                   currIMUMeasure.accelerometer);
        Eigen::Vector3d nextRobotLinVelocity = poseEstimator.getNextLinearVelocity(currRobotLinVelocity, 
                                                                                   currRobotOrientation,
                                                                                   deltaT,
                                                                                   currIMUMeasure.accelerometer);
        Eigen::Matrix3d nextRobotOrientation = poseEstimator.getNextOrientation(currRobotOrientation,
                                                                                deltaT,
                                                                                currIMUMeasure.gyroscope);
        // update current robot position and next robot orientation for next iteration
        currRobotLinPosition = nextRobotLinPosition;
        currRobotLinVelocity = nextRobotLinVelocity;
        currRobotOrientation = nextRobotOrientation;
    }
    return correctedLiDARMeasurements;
}