#pragma once
#include <vector>
#include "Constants.hpp"
#include "IMUMeasurement.hpp"
#include "TrajectoryPoint.hpp"

class TrajectoryExtractor{

    public:
        std::vector<TrajectoryPoint> extractTrajectoryPoints(std::vector<IMUMeasurment> imuMeasurements);
};