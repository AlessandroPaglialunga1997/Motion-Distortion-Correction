#pragma once
#include <vector>
#include "Constants/TrajectoriesConstants.hpp"
#include "Sensors/IMU/IMUMeasurement.hpp"
#include "TrajectoryTools/TrajectoryPoint.hpp"
#include "Estimators/PoseEstimator.hpp"

class TrajectoryExtractor{

    public:
        std::vector<TrajectoryPoint> extractTrajectoryPoints(std::vector<IMUMeasurment> imuMeasurements);
};