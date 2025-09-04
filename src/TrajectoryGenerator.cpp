#include "TrajectoryGenerator.hpp"
#include <cmath>

std::vector<TrajectoryPoint> TrajectoryGenerator::getHelicalTrajectory(double radius, 
                                                                       double height, 
                                                                       double duration, 
                                                                       double deltaT) {
    std::vector<TrajectoryPoint> trajectory;
    double omega = 2.0 * M_PI / duration; // constant angular velocity

    for (double timestamp = 0.0; timestamp <= duration; timestamp += deltaT) {
        TrajectoryPoint currPoint;
        currPoint.timestamp = timestamp;
        currPoint.position = Eigen::Vector3d(radius * cos(omega * timestamp), 
                                             radius * sin(omega * timestamp), 
                                             height * timestamp / duration);
        double currYaw = omega * timestamp;
        currPoint.orientation <<    cos(currYaw), -sin(currYaw), 0,
                                    sin(currYaw),  cos(currYaw), 0,
                                               0,             0, 1;

        trajectory.push_back(currPoint);
    }

    return trajectory;
}
