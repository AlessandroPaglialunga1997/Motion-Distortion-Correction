#include <iostream>
#include <fstream>
#include "Constants/TrajectoriesConstants.hpp"
#include "MotionDistortionRemovers/MotionDistortionRemover.hpp"
#include "Readers/FileReader.hpp"
#include "Sensors/LiDAR/LiDARMeasurementsGenerator.hpp"
#include "Sensors/IMU/IMUMeasurementsExtractor.hpp"
#include "TrajectoryTools/TrajectoryGenerator.hpp"
#include "TrajectoryTools/TrajectoryExtractor.hpp"
#include "TrajectoryTools/TrajectoryPoint.hpp"

int main(){
    FileReader fileReader;
    std::vector<TrajectoryPointWithQuaternion> quadHardTrueTrajectory = fileReader.readTrajectoryFile("../input_data/gt-nc-quad-hard.csv", ",", true);
    std::ofstream genFile("../output_data/quadHardTrueTrajectory.dat");
    for (const TrajectoryPointWithQuaternion& currPoint : quadHardTrueTrajectory) {
        genFile << currPoint.position(0) << " "    //x
                << currPoint.position(1) << " "    //y
                << currPoint.position(2) << "\n";  //z
    }
    return 0;
}

/*int main() {
    IMUMeasurementsExtractor imuMeasurementsExtractor;
    TrajectoryGenerator trajectoryGenerator;
    TrajectoryExtractor trajectoryExtractor;

    std::vector<TrajectoryPoint> generatedHelicalTrajectory =  trajectoryGenerator.getHelicalTrajectory(Constants::helicalRadius,
                                                                                                        Constants::helicalHeight, 
                                                                                                        Constants::trajectoryDuration, 
                                                                                                        Constants::trajectoryDeltaT,
                                                                                                        Constants::helicalPositionOffset);

    std::vector<IMUMeasurment> imuMeasurements = imuMeasurementsExtractor.extractIMUMeasurments(generatedHelicalTrajectory);

    std::vector<TrajectoryPoint> extractedHelicalTrajectory = trajectoryExtractor.extractTrajectoryPoints(imuMeasurements);
    
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
}*/

/*
int main(){
    LiDARMeasurementsGenerator lidarMeasurementsGenerator;
    TrajectoryGenerator trajectoryGenerator;
    MotionDistortionRemover motionDistortionRemover;
    IMUMeasurementsExtractor imuMeasurementsExtractor;
    
    // 1. Generate the point cloud 
    //      to simulate LiDAR measurements
    std::vector<LiDARMeasurement> generatedSphereLiDARMeasurements = lidarMeasurementsGenerator.getDistortedSpherePoints(0.0,
                                                                                                                           Constants::sphereRadius,
                                                                                                                           Constants::sphereAngularStep,
                                                                                                                           Constants::spherePositionOffset,
                                                                                                                           Eigen::Vector3d::Zero(),
                                                                                                                           Eigen::Matrix3d::Identity());
    // 2. Visualize the point cloud
    std::ofstream genFile("generatedSpherePoints.dat");
    for (const LiDARMeasurement& currPoint : generatedSphereLiDARMeasurements) {
        genFile << currPoint.position(0) << " "    //x
                << currPoint.position(1) << " "    //y
                << currPoint.position(2) << "\n";  //z
    }
    genFile.close();

    // 3. Generate the Motion Distortion 
    //      since LiDAR is mounted on the robot, after some steps the point cloud is moved because the LiDAR measurments are based from current robot reference frame
    std::vector<TrajectoryPoint> generatedHelicalTrajectory = trajectoryGenerator.getHelicalTrajectory(Constants::helicalRadius,
                                                                                                       Constants::helicalHeight, 
                                                                                                       Constants::trajectoryDuration, 
                                                                                                       Constants::trajectoryDeltaT,
                                                                                                       Constants::helicalPositionOffset);
    std::vector<TrajectoryPoint> partialTrajectory;
    partialTrajectory.insert(partialTrajectory.end(), generatedHelicalTrajectory.begin(), generatedHelicalTrajectory.begin() + 5);
    std::vector<LiDARMeasurement> distortedSphereLiDARMeasurementsTrajectory = lidarMeasurementsGenerator.getDistortedSphereTrajectory(generatedHelicalTrajectory,
                                                                                                                                       Constants::sphereRadius,
                                                                                                                                       Constants::sphereAngularStep,
                                                                                                                                       Constants::spherePositionOffset);
    std::ofstream distortedSphereFile("distortedSphereTrajectory.dat");
    for (const LiDARMeasurement& currPoint : distortedSphereLiDARMeasurementsTrajectory) {
        distortedSphereFile << currPoint.position(0) << " "    //x
                << currPoint.position(1) << " "    //y
                << currPoint.position(2) << "\n";  //z
    }
    distortedSphereFile.close();

    // 4. Remove the Motion Distortion using IMU measurments
    std::vector<IMUMeasurment> imuMeasurements = imuMeasurementsExtractor.extractIMUMeasurments(generatedHelicalTrajectory);
    std::vector<LiDARMeasurement>  correctedSphereLiDARMeasurementsTrajectory = motionDistortionRemover.removeDistortionUsingIMU(distortedSphereLiDARMeasurementsTrajectory,
                                                                                                                                 imuMeasurements);
    std::ofstream correctedSphereFile("correctedSphereTrajectory.dat");
    for (const LiDARMeasurement& currPoint : correctedSphereLiDARMeasurementsTrajectory) {
        correctedSphereFile << currPoint.position(0) << " "    //x
                << currPoint.position(1) << " "    //y
                << currPoint.position(2) << "\n";  //z
    }
    correctedSphereFile.close();

    std::cout << "distortedSphereLiDARMeasurementsTrajectory size: " << distortedSphereLiDARMeasurementsTrajectory.size() << std::endl;
    std::cout << "distortedSphereLiDARMeasurementsTrajectory size: " << imuMeasurements.size() << std::endl;
    std::cout << "distortedSphereLiDARMeasurementsTrajectory size: " << correctedSphereLiDARMeasurementsTrajectory.size() << std::endl;

    // 5. Check the error between the point cloud obtained at 1. and the one obtained at 4.

    return 0;
}*/