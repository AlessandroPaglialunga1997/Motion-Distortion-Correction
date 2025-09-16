#pragma once
#include <vector>
#include "LiDARMeasurement.hpp"
#include "IMUMeasurement.hpp"
#include "TrajectoryPoint.hpp"
#include "PoseEstimator.hpp"

class MotionDistortionRemover {
    
    public:
        std::vector<LiDARMeasurement> removeDistortionUsingIMU(std::vector<LiDARMeasurement> distortedLiDARMeasurements,
                                                               std::vector<IMUMeasurment> imuMeasurements);
};