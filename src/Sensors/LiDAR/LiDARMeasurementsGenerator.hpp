#pragma once
#include <vector>
#include "Sensors/LiDAR/LiDARMeasurement.hpp"
#include "TrajectoryTools/TrajectoryPoint.hpp"
#include <cmath>

class LiDARMeasurementsGenerator {
    
    public:
        std::vector<LiDARMeasurement> getDistortedSpherePoints(double timestamp,
                                                double radius,
                                                double angulaStep,
                                                Eigen::Vector3d positionOffset,
                                                Eigen::Vector3d positionDistortion,
                                                Eigen::Matrix3d orientationDistortion);
        
        std::vector<LiDARMeasurement> getDistortedSphereTrajectory(std::vector<TrajectoryPoint> trajectory,
                                                          double radius,
                                                          double angularStep,
                                                          Eigen::Vector3d positionOffset);
};
