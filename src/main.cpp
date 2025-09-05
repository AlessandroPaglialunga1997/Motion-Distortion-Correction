#include <iostream>
#include <fstream>
#include "TrajectoryGenerator.hpp"
#include "IMUMeasurementsExtractor.hpp"
#include "Constants.hpp"
#include "TrajectoryExtractor.hpp"


void printMaxTrajectoryError(const std::vector<TrajectoryPoint>& expectedTraj,
                             const std::vector<TrajectoryPoint>& actualTraj) {
    double maxPosError = 0.0;
    double maxOriError = 0.0;

    for (size_t i = 0; i < expectedTraj.size(); ++i) {
        const auto& expectedPose = expectedTraj[i];
        const auto& actualPose = actualTraj[i];

        // Errore di posizione
        double posError = (expectedPose.position - actualPose.position).norm();
        maxPosError = std::max(maxPosError, posError);

        // Errore di orientamento
        Eigen::Matrix3d R_rel = expectedPose.orientation.transpose() * actualPose.orientation;
        Eigen::AngleAxisd angleAxis(R_rel);
        double oriError = std::abs(angleAxis.angle());
        maxOriError = std::max(maxOriError, oriError);
    }

    std::cout << "Errore massimo di posizione: " << maxPosError << " metri" << std::endl;
    std::cout << "Errore massimo di orientamento: " << maxOriError << " radianti" << std::endl;
}


int main() {
    IMUMeasurementsExtractor imuMeasurementsExtractor;
    TrajectoryGenerator trajectoryGenerator;
    TrajectoryExtractor trajectoryExtractor;

    std::vector<TrajectoryPoint> generatedHelicalTrajectory = 
                                 trajectoryGenerator.getHelicalTrajectory(Constants::helicalRadius,
                                                                          Constants::helicalRadius, 
                                                                          Constants::trajectoryDuration, 
                                                                          Constants::trajectoryDeltaT,
                                                                          Constants::positionOffset);

    std::vector<IMUMeasurment> imuMeasurements = imuMeasurementsExtractor.extractIMUMeasurments(generatedHelicalTrajectory);

    std::vector<TrajectoryPoint> extractedHelicalTrajectory =
                                 trajectoryExtractor.extractTrajectoryPoints(imuMeasurements);
    
    printMaxTrajectoryError(generatedHelicalTrajectory, extractedHelicalTrajectory);

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
