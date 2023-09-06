//
// Created by tim on 26.03.21.
//

#include <ros/ros.h>
#include "graphSlamSaveStructure.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/Path.h"

#include <random>
#include "scanRegistrationClass.h"

#ifndef SIMULATION_BLUEROV_SLAMTOOLSROS_H
#define SIMULATION_BLUEROV_SLAMTOOLSROS_H

struct intensityValues {
    Eigen::Matrix4d transformation;
    intensityMeasurement intensity;
};


class slamToolsRos {

public:

    static void visualizeCurrentPoseGraph(graphSlamSaveStructure &graphSaved, ros::Publisher &publisherPath,
                                          ros::Publisher &publisherMarker, double sigmaScaling,
                                          ros::Publisher &publisherPoseSlam, ros::Publisher &publisherLoopClosures);




//    static std::vector<double> linspace(double start_in, double end_in, int num_in);




    static edge
    calculatePoseDiffByTimeDepOnEKF(double startTimetoAdd, double endTimeToAdd, std::deque<double> &timeVector,
                                    std::deque<double> &xPositionVector, std::deque<double> &yPositionVector,
                                    std::deque<double> &zPositionVector, std::deque<Eigen::Quaterniond> &rotationVector,
                                    std::mutex &stateEstimationMutex);

    static double angleBetweenLastKeyframeAndNow(graphSlamSaveStructure &graphSaved);

    static int getLastIntensityKeyframe(graphSlamSaveStructure &graphSaved);

    static double getDatasetFromGraphForMap(std::vector<intensityValues> &dataSet, graphSlamSaveStructure &graphSaved,
                                            std::mutex &graphSlamMutex);

    static void clearSavingsOfPoses(double upToTime, std::deque<double> &timeVector,
                                    std::deque<double> &xPositionVector, std::deque<double> &yPositionVector,
                                    std::deque<double> &zPositionVector, std::deque<Eigen::Quaterniond> &rotationVector,
                                    std::mutex &stateEstimationMutex);


    static double createVoxelOfGraphStartEndPoint(double voxelData[], int indexStart, int indexEnd,
                                                  int numberOfPoints, graphSlamSaveStructure &usedGraph,
                                                  double ignoreDistanceToRobot, double dimensionOfVoxelData,
                                                  Eigen::Matrix4d transformationInTheEndOfCalculation);

    static bool calculateStartAndEndIndexForVoxelCreation(int indexMiddle, int &indexStart, int &indexEnd,
                                                          graphSlamSaveStructure &usedGraph);

    static Eigen::Matrix4d registrationOfTwoVoxelsFast(double voxelData1Input[],
                                                   double voxelData2Input[],
                                                   Eigen::Matrix4d initialGuess,
                                                   Eigen::Matrix3d &covarianceMatrix,
                                                   bool useInitialAngle,
                                                   bool useInitialTranslation,
                                                   double cellSize,
                                                   bool useGauss,
                                                   scanRegistrationClass &scanRegistrationObject,
                                                   bool debug, double potentialNecessaryForPeak = 0.1, bool multipleRadii = false,
                                                   bool useClahe = true,
                                                   bool useHamming = true);



    static bool
    loopDetectionByClosestPath(graphSlamSaveStructure &graphSaved, scanRegistrationClass &scanRegistrationObject,
                               int dimensionOfVoxelData,
                               double ignoreDistanceToRobot, double distanceOfVoxelDataLengthSI,
                               bool debugRegistration, bool useInitialTranslation,
                               double potentialNecessaryForPeak = 0.1, double maxLoopClosure = 100);


    static bool
    calculateEndIndexForVoxelCreationByStartIndex(int indexStart, int &indexEnd, graphSlamSaveStructure &usedGraph);

    };

#endif //SIMULATION_BLUEROV_VISUALIZESLAMINROS_H
