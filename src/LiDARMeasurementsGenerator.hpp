#pragma once
#include <vector>
#include "LiDARMeasurement.hpp"
#include "TrajectoryPoint.hpp"
#include <cmath>

class LiDARMeasurementsGenerator {
    
    public:
        std::vector<LiDARMeasurement> getSphere(double timestamp,
                                                double radius,
                                                double angulaStep,
                                                Eigen::Vector3d positionOffset,
                                                Eigen::Vector3d positionDistortion,
                                                Eigen::Matrix3d orientationDistortion);
        
        std::vector<LiDARMeasurement> getSphereTrajectory(std::vector<TrajectoryPoint> trajectory,
                                                          double radius,
                                                          double angularStep,
                                                          Eigen::Vector3d positionOffset);
};
