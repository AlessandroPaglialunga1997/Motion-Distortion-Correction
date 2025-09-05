#pragma once
#include <vector>
#include "TrajectoryPoint.hpp"
#include "IMUMeasurement.hpp"
#include "Constants.hpp"

class IMUMeasurementsExtractor{
        
    private:
        Eigen::Vector3d estimateAngularVelocity(TrajectoryPoint currPose, 
                                               TrajectoryPoint nextPose);
        Eigen::Vector3d estimateLinearAcceleration(TrajectoryPoint prevPose, 
                                                   TrajectoryPoint currPose, 
                                                   TrajectoryPoint nextPose);
    public:
        std::vector<IMUMeasurment> extractIMUMeasurments(std::vector<TrajectoryPoint> trajectory);
};