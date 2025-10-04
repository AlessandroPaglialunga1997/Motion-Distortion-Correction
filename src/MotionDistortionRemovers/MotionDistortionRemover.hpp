#pragma once
#include <vector>
#include "Sensors/LiDAR/LiDARMeasurement.hpp"
#include "Sensors/IMU/IMUMeasurement.hpp"
#include "TrajectoryTools/TrajectoryPoint.hpp"
#include "Estimators/PoseEstimator.hpp"

class MotionDistortionRemover {
    
    public:
        std::vector<LiDARMeasurement> removeDistortionUsingIMU(std::vector<LiDARMeasurement> distortedLiDARMeasurements,
                                                               std::vector<IMUMeasurment> imuMeasurements);
};