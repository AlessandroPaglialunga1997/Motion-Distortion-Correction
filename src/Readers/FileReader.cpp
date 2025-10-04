#include "FileReader.hpp"

std::vector<TrajectoryPointWithQuaternion> FileReader::readTrajectoryFile(const std::string& filename, 
                                                                      const std::string& delimiter,
                                                                      bool hasHeader){
    std::vector<TrajectoryPointWithQuaternion> trajectory;
    std::ifstream file(filename);
    std::string line;
    if (!file.is_open()) 
        throw std::runtime_error("Could not open file: " + filename);

    if (hasHeader)
        std::getline(file, line); // Skip header line which is #sec,nsec,x,y,z,qx,qy,qz,qw

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        TrajectoryPointWithQuaternion point;

        // Read timestamp
        std::getline(ss, token, delimiter[0]);
        double sec = std::stod(token);
        std::getline(ss, token, delimiter[0]);
        double nsec = std::stod(token);
        point.timestamp = sec + nsec * 1e-9;

        // Read position
        for (int i = 0; i < 3; ++i) {
            std::getline(ss, token, delimiter[0]);
            point.position[i] = std::stod(token);
        }

        // Read orientation as quaternion
        Eigen::Quaterniond q;
        for (int i = 0; i < 4; ++i) {
            std::getline(ss, token, delimiter[0]);
                 if (i == 0) q.x() = std::stod(token);
            else if (i == 1) q.y() = std::stod(token);
            else if (i == 2) q.z() = std::stod(token);
            else if (i == 3) q.w() = std::stod(token);
        }
        point.orientation = q;

        trajectory.push_back(point);
    }
    file.close();
    return trajectory;
}