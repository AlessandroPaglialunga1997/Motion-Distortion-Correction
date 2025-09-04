#include <iostream>
#include <fstream>
#include "TrajectoryGenerator.hpp"
#include "IMUMeasurementsExtractor.hpp"
#include "Constants.hpp"
#include "TrajectoryExtractor.hpp"

/*void printIMUMeasurements(const std::vector<IMUMeasurment>& measurements) {
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
}*/


int main() {
    IMUMeasurmentsExtractor imuMeasurmentsExtractor;
    TrajectoryGenerator trajectoryGenerator;
    TrajectoryExtractor trajectoryExtractor;

    std::vector<TrajectoryPoint> generatedHelicalTrajectory = 
                                 trajectoryGenerator.getHelicalTrajectory(Constants::helicalRadius,
                                                                          Constants::helicalRadius, 
                                                                          Constants::trajectoryDuration, 
                                                                          Constants::trajectoryDeltaT,
                                                                          Constants::positionOffset);

    std::vector<IMUMeasurment> imuMeasurements = imuMeasurmentsExtractor.extractIMUMeasurments(generatedHelicalTrajectory);

    std::vector<TrajectoryPoint> extractedHelicalTrajectory =
                                 trajectoryExtractor.extractTrajectoryPoints(imuMeasurements);
    

    std::ofstream genFile("generatedHelicalTrajectory.dat");
    for (const TrajectoryPoint& currPoint : generatedHelicalTrajectory) {
        genFile << currPoint.position(0) << " "    //x
             << currPoint.position(1) << " "    //y
             << currPoint.position(2) << "\n";  //z
    }
    genFile.close();

    std::ofstream imuFile("imuHelicalTrajectory.dat");
    for (const IMUMeasurment& currMeasurement : imuMeasurements) {
        imuFile << "Time: " << currMeasurement.timestamp << " "
                << "Accelerometer: [" 
                << currMeasurement.accelerometer.x() << ", "
                << currMeasurement.accelerometer.y() << ", "
                << currMeasurement.accelerometer.z() << "] "
                << "Gyroscope: [" 
                << currMeasurement.gyroscope.x() << ", "
                << currMeasurement.gyroscope.y() << ", "
                << currMeasurement.gyroscope.z() << "]\n";
    }
    imuFile.close();

    std::ofstream extFile("extractedHelicalTrajectory.dat");
    for (const TrajectoryPoint& currPoint : extractedHelicalTrajectory) {
        extFile << currPoint.position(0) << " "
                << currPoint.position(1) << " "
                << currPoint.position(2) << "\n";
    }
    extFile.close();


    return 0;
}
