#include "LiDARMeasurementsGenerator.hpp"

std::vector<LiDARMeasurement> LiDARMeasurementsGenerator::getDistortedSpherePoints(double timestamp,
                                                                                   double radius,
                                                                                   double angularStep,
                                                                                   Eigen::Vector3d positionOffset,
                                                                                   Eigen::Vector3d positionDistortion,
                                                                                   Eigen::Matrix3d orientation){
    std::vector<LiDARMeasurement> sphere;
    int id = 0;
    for(double angleX_Y = 0; angleX_Y <= 2*M_PI; angleX_Y += angularStep){
        for(double angleX_Z = 0; angleX_Z <= 2*M_PI; angleX_Z += angularStep){
            LiDARMeasurement currPoint;
            currPoint.id = id++;
            currPoint.timestamp = timestamp;
            Eigen::Vector3d worldPoint = positionOffset + Eigen::Vector3d(radius * cos(angleX_Y) * cos(angleX_Z), 
                                                                          radius * sin(angleX_Y) * cos(angleX_Z), 
                                                                          radius * sin(angleX_Z));
            currPoint.position = orientation * worldPoint + positionDistortion;

            sphere.push_back(currPoint);
        }
    }

    return sphere;
}

std::vector<LiDARMeasurement> LiDARMeasurementsGenerator::getDistortedSphereTrajectory(std::vector<TrajectoryPoint> trajectory,
                                                                              double radius,
                                                                              double angularStep,
                                                                              Eigen::Vector3d positionOffset){
    std::vector<LiDARMeasurement> sphereTrajectory;
    for (auto it = trajectory.begin(); std::next(it) != trajectory.end(); ++it){
        const TrajectoryPoint& currPose = *it;
        Eigen::Vector3d positionDistortion = -1 * currPose.position;
        Eigen::Matrix3d orientation = currPose.orientation.transpose();
        std::vector<LiDARMeasurement> localCurrSphere = getDistortedSpherePoints(currPose.timestamp,
                                                                                 radius,
                                                                                 angularStep,
                                                                                 positionOffset,
                                                                                 positionDistortion,
                                                                                 orientation);
        sphereTrajectory.insert(sphereTrajectory.end(), localCurrSphere.begin(), localCurrSphere.end());                                                         
    }

    return sphereTrajectory;
}