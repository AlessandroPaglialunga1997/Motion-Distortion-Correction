#pragma once
#include <vector>
#include "TrajectoryPoint.hpp"

class TrajectoryGenerator {
public:
    std::vector<TrajectoryPoint> getHelicalTrajectory(double radius, 
                                                      double step, 
                                                      double duration, 
                                                      double deltaT);
};
