#pragma once
#include <vector>
#include <string>
#include "TrajectoryTools/TrajectoryPoint.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>

class FileReader {

    public:
        std::vector<TrajectoryPointWithQuaternion> readTrajectoryFile(const std::string& filename, 
                                                                      const std::string& delimiter,
                                                                      bool hasHeader);
};