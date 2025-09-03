#include <iostream>
#include <fstream>
#include "TrajectoryGenerator.hpp"

int main() {
    TrajectoryGenerator trajectoryGenerator;
    auto helicalTrajectory = trajectoryGenerator.getHelicalTrajectory(1.0, 2.0, 10.0, 0.1);

    std::ofstream file("helicalTrajectory.dat");
    for (const auto& currPoint : helicalTrajectory) {
        file << currPoint.position(0) << " "    //x
             << currPoint.position(1) << " "    //y
             << currPoint.position(2) << "\n";  //z
    }
    file.close();

    return 0;
}
