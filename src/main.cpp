#include <iostream>
#include <fstream>
#include "TrajectoryGenerator.hpp"
#include "IMUMeasurementsExtractor.hpp"
#include "Constants.hpp"

void printIMUMeasurements(const std::vector<IMUMeasurment>& measurements) {
    for (const auto& meas : measurements) {
        std::cout << "Timestamp: " << meas.timestamp << "\n";
        std::cout << "Accelerometer: [" 
                  << meas.accelerometer.x() << ", "
                  << meas.accelerometer.y() << ", "
                  << meas.accelerometer.z() << "]\n";
        std::cout << "Gyroscope: [" 
                  << meas.gyroscope.x() << ", "
                  << meas.gyroscope.y() << ", "
                  << meas.gyroscope.z() << "]\n";
        std::cout << "-----------------------------\n";
    }
}


int main() {
    TrajectoryGenerator trajectoryGenerator;
    std::vector<TrajectoryPoint> helicalTrajectory = 
                                 trajectoryGenerator.getHelicalTrajectory(Constants::helicalRadius,
                                                                          Constants::helicalRadius, 
                                                                          Constants::trajectoryDuration, 
                                                                          Constants::trajectoryDeltaT);
    std::ofstream file("helicalTrajectory.dat");
    for (const TrajectoryPoint& currPoint : helicalTrajectory) {
        file << currPoint.position(0) << " "    //x
             << currPoint.position(1) << " "    //y
             << currPoint.position(2) << "\n";  //z
    }
    file.close();

    IMUMeasurmentsExtractor imuMeasurmentsExtractor;
    std::vector<IMUMeasurment> imuMeasurements = imuMeasurmentsExtractor.extractIMUMeasurments(helicalTrajectory);
    printIMUMeasurements(imuMeasurements);

    return 0;
}
