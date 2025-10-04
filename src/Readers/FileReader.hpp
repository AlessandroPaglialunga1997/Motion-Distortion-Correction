#pragma once
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include "TrajectoryTools/TrajectoryPoint.hpp"
#include "Sensors/IMU/IMUMeasurement.hpp"

class FileReader {

    public:
        std::vector<TrajectoryPointWithQuaternion> getTrajectoryFromCSV(const std::string& filename, 
                                                                 const std::string& delimiter,
                                                                 bool hasHeader);
        
        std::vector<IMUMeasurment> getIMUMeasurementsFromBag(const std::string& filename, 
                                                             const std::string& topicName);
};