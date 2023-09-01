//
// Created by Tim Hansen on 30.08.23.
//


#include "geometry_msgs/PoseStamped.h"
#include "commonbluerovmsg/SonarEcho2.h"
#include "generalHelpfulTools.h"
#include "slamToolsRos.h"
#include "commonbluerovmsg/saveGraph.h"
#include "nav_msgs/OccupancyGrid.h"
//#include "scanRegistrationClass.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/SetBoolRequest.h"
#include <std_srvs/SetBool.h>
#include "commonbluerovmsg/staterobotforevaluation.h"



#define NAME_OF_CURRENT_METHOD "ourTest"
//#define SAVE_CALCULATION_FOLDER "/home/ubuntu/timeMeasurements/exp1TimesSaved.csv"

class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_, int dimensionOfRegistration, double radiusOfScanSize, int NSizeMap,
                double mapDimension, double howOftenRegistrationPerFullScan, double loopClosureDistance,
                bool reverseScanDirection, double rotationSonar, double removeDistanceToRobot) : graphSaved(3,
                                                                                                            INTENSITY_BASED_GRAPH),
                                                                                                 scanRegistrationObject(
                                                                                                         dimensionOfRegistration,
                                                                                                         dimensionOfRegistration /
                                                                                                         2,
                                                                                                         dimensionOfRegistration /
                                                                                                         2,
                                                                                                         dimensionOfRegistration /
                                                                                                         2 -
                                                                                                         1) {

        this->subscriberEKF = n_.subscribe("publisherPoseEkf", 10000, &rosClassEKF::stateEstimationCallback, this);
        ros::Duration(2).sleep();
        this->subscriberIntensitySonar = n_.subscribe("sonar/intensity", 10000, &rosClassEKF::scanCallback, this);
        this->serviceSaveGraph = n_.advertiseService("saveGraphOfSLAM", &rosClassEKF::saveGraph, this);


        publisherPathOverTime = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
        publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);
        publisherMarkerArrayLoopClosures = n_.advertise<visualization_msgs::MarkerArray>("loopClosures", 10);
        publisherOccupancyMap = n_.advertise<nav_msgs::OccupancyGrid>("occupancyMap", 10);

        publisherPoseSLAM = n_.advertise<geometry_msgs::PoseStamped>("slamEndPose", 10);


        this->sigmaScaling = 0.1;// was 1.0
        this->firstSonarInput = true;
        this->firstCompleteSonarScan = true;
        this->numberOfTimesFirstScan = 0;

        map.info.height = NSizeMap;
        map.info.width = NSizeMap;
        map.info.resolution = mapDimension / NSizeMap;
        map.info.origin.position.x = -mapDimension / 2;
        map.info.origin.position.y = -mapDimension / 2;
        map.info.origin.position.z = +0.5;
        for (int i = 0; i < NSizeMap; i++) {
            for (int j = 0; j < NSizeMap; j++) {
                //determine color:
                map.data.push_back(50);
            }
        }
        this->computationTime = 0;

        this->dimensionOfRegistration = dimensionOfRegistration;
        this->radiusOfScanSize = radiusOfScanSize;
        this->NSizeMap = NSizeMap;
        this->mapDimension = mapDimension;
        this->howOftenRegistrationPerFullScan = howOftenRegistrationPerFullScan;
        this->loopClosureDistance = loopClosureDistance;
        this->reverseScanDirection = reverseScanDirection;
        this->rotationSonar = rotationSonar;
        this->removeDistanceToRobot = removeDistanceToRobot;

    }


private:
    nav_msgs::OccupancyGrid map;


    ros::Subscriber subscriberEKF, subscriberIntensitySonar, subscriberPositionGT, subscriberPositionGTGantry;
    ros::Publisher publisherPoseSLAM, publisherOccupancyMap;
    ros::ServiceServer serviceSaveGraph;
    std::mutex stateEstimationMutex;
    std::mutex graphSlamMutex;

    //GraphSlam things
    ros::Publisher  publisherPathOverTime, publisherMarkerArray, publisherMarkerArrayLoopClosures;

    //Matrices:
    Eigen::Matrix4d currentEstimatedTransformation;
    Eigen::Matrix4d initialGuessTransformation;


    //EKF savings
//    std::deque<edge> posDiffOverTimeEdges;
    std::deque<double> xPositionVector, yPositionVector, zPositionVector, timeVector;
    std::deque<Eigen::Quaterniond> rotationVector;

    double sigmaScaling;

    graphSlamSaveStructure graphSaved;
    scanRegistrationClass scanRegistrationObject;
    bool firstSonarInput, firstCompleteSonarScan;// saveGraphStructure;
    int numberOfTimesFirstScan;
    double computationTime;


    // config settings
    int dimensionOfRegistration;
    double radiusOfScanSize;
    int NSizeMap;
    double mapDimension;
    double howOftenRegistrationPerFullScan;
    double loopClosureDistance;
    bool reverseScanDirection;
    double rotationSonar;
    double removeDistanceToRobot;

    void scanCallback(const commonbluerovmsg::SonarEcho2::ConstPtr &msg) {
        //used for locking and to make asynchron map creations possible
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        intensityMeasurement intensityTMP;
        // for the sonar on the bottom of the robot vs top of robot
        if (this->reverseScanDirection) {
            intensityTMP.angle = std::fmod(-msg->angle / 400.0 * M_PI * 2.0 + this->rotationSonar,
                                           M_PI * 2);// TEST TRYING OUT -
        } else {
            intensityTMP.angle = std::fmod(msg->angle / 400.0 * M_PI * 2.0 + this->rotationSonar,
                                           M_PI * 2);// TEST TRYING OUT -
        }
        intensityTMP.time = msg->header.stamp.toSec();
        intensityTMP.range = msg->range;
        intensityTMP.increment = msg->step_size;
        std::vector<double> intensitiesVector;
        for (int i = 0; i < msg->intensities.size(); i++) {
            intensitiesVector.push_back(msg->intensities[i]);
        }
        intensityTMP.intensities = intensitiesVector;
        // first scan line, where we dont need an edge yet.
        if (firstSonarInput) {

            this->graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                                       Eigen::Matrix3d::Zero(), intensityTMP, msg->header.stamp.toSec(),
                                       FIRST_ENTRY);
            firstSonarInput = false;
            return;
        }

        // wait until enough EKF messages are there to calculate the position of the scan line
        bool waitingForMessages = waitForEKFMessagesToArrive2(0.1, msg->header.stamp.toSec());
        if (!waitingForMessages) {
            std::cout << "return no message found: " << msg->header.stamp.toSec() << "    " << ros::Time::now().toSec()
                      << std::endl;
            return;
        }

        // calc difference between vertexes by EKF. edge used as container
        edge differenceOfEdge = slamToolsRos::calculatePoseDiffByTimeDepOnEKF(
                this->graphSaved.getVertexList()->back().getTimeStamp(), msg->header.stamp.toSec(), this->timeVector,
                this->xPositionVector, this->yPositionVector,
                this->zPositionVector, this->rotationVector, this->stateEstimationMutex);
        slamToolsRos::clearSavingsOfPoses(this->graphSaved.getVertexList()->back().getTimeStamp() - 2, this->timeVector,
                                          this->xPositionVector, this->yPositionVector,
                                          this->zPositionVector, this->rotationVector, this->stateEstimationMutex);

        Eigen::Matrix4d tmpTransformation = this->graphSaved.getVertexList()->back().getTransformation();
        tmpTransformation = tmpTransformation * differenceOfEdge.getTransformation();
        Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
        Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
        Eigen::Quaterniond rot(rotM);

        //add a new edge and vertex to the graph defined by EKF and Intensity Measurement
        this->graphSaved.addVertex(this->graphSaved.getVertexList()->back().getKey() + 1, pos, rot,
                                   this->graphSaved.getVertexList()->back().getCovarianceMatrix(),
                                   intensityTMP,
                                   msg->header.stamp.toSec(),
                                   INTENSITY_SAVED);

        // covariance values fixed for now. Could be changed
        Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
        covarianceMatrix(0, 0) = 0.03;//x
        covarianceMatrix(1, 1) = 0.03;//y
        covarianceMatrix(2, 2) = 0.03;//theta
        this->graphSaved.addEdge(this->graphSaved.getVertexList()->back().getKey() - 1,
                                 this->graphSaved.getVertexList()->back().getKey(),
                                 differenceOfEdge.getPositionDifference(), differenceOfEdge.getRotationDifference(),
                                 covarianceMatrix, INTEGRATED_POSE);


        double angleDiff = slamToolsRos::angleBetweenLastKeyframeAndNow(this->graphSaved);

        // look if angle is high enough for doing a registration

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0;
        this->computationTime += timeToCalculate;
        if (abs(angleDiff) > 2 * M_PI / this->howOftenRegistrationPerFullScan) {


            this->graphSaved.getVertexList()->back().setTypeOfVertex(INTENSITY_SAVED_AND_KEYFRAME);
            // first complete scan created, afterwords SLAM can start.
            if (firstCompleteSonarScan) {
                numberOfTimesFirstScan++;
                if (numberOfTimesFirstScan > 2 * this->howOftenRegistrationPerFullScan - 1) {
                    firstCompleteSonarScan = false;
                }
                return;
            }

            begin = std::chrono::steady_clock::now();

            // calculate indices, where in the graph the two scans are created
            int indexStart1, indexEnd1, indexStart2, indexEnd2;
            slamToolsRos::calculateStartAndEndIndexForVoxelCreation(
                    this->graphSaved.getVertexList()->back().getKey() - 5, indexStart1, indexEnd1, this->graphSaved);
            indexStart2 = indexEnd1;
            slamToolsRos::calculateEndIndexForVoxelCreationByStartIndex(indexStart2, indexEnd2, this->graphSaved);


            std::cout << "scanAcusitionTime: " << this->graphSaved.getVertexList()->at(indexStart2).getTimeStamp() -
                                                  this->graphSaved.getVertexList()->at(indexEnd2).getTimeStamp()
                      << std::endl;

            this->initialGuessTransformation =
                    (this->graphSaved.getVertexList()->at(indexStart2).getTransformation().inverse() *
                     this->graphSaved.getVertexList()->at(indexStart1).getTransformation());



            double initialGuessAngle = std::atan2(this->initialGuessTransformation(1, 0),
                                                  this->initialGuessTransformation(0, 0));

            double *voxelData1;
            double *voxelData2;
            voxelData1 = (double *) malloc(
                    sizeof(double) * this->dimensionOfRegistration * this->dimensionOfRegistration);
            voxelData2 = (double *) malloc(
                    sizeof(double) * this->dimensionOfRegistration * this->dimensionOfRegistration);

            double maximumVoxel1 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData1, indexStart1, indexEnd1,
                                                                                 this->dimensionOfRegistration,
                                                                                 this->graphSaved,
                                                                                 this->removeDistanceToRobot,
                                                                                 this->radiusOfScanSize,
                                                                                 Eigen::Matrix4d::Identity());//get voxel


            double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData2, indexStart2, indexEnd2,
                                                                                 this->dimensionOfRegistration,
                                                                                 this->graphSaved,
                                                                                 this->removeDistanceToRobot,
                                                                                 this->radiusOfScanSize,
                                                                                 Eigen::Matrix4d::Identity());//get voxel

            Eigen::Matrix3d covarianceEstimation = Eigen::Matrix3d::Zero();
            std::cout << "consecutive registration: " << std::endl;
            // result is matrix to transform scan 1 to scan 2 therefore later inversed + initial guess inversed


            this->currentEstimatedTransformation = slamToolsRos::registrationOfTwoVoxelsFast(voxelData1, voxelData2,
                                                                                             this->initialGuessTransformation,
                                                                                             covarianceEstimation, true,
                                                                                             true,
                                                                                             (double) this->radiusOfScanSize /
                                                                                             (double) this->dimensionOfRegistration,
                                                                                             false,
                                                                                             this->scanRegistrationObject,
                                                                                             false,
                                                                                             0.1);

            slamToolsRos::saveResultingRegistrationTMPCOPY(indexStart1, indexEnd1, indexStart2, indexEnd2,
                                                           this->graphSaved, this->dimensionOfRegistration,
                                                           this->removeDistanceToRobot,
                                                           this->radiusOfScanSize,
                                                           false, this->currentEstimatedTransformation,
                                                           initialGuessTransformation);

            double differenceAngleBeforeAfter = generalHelpfulTools::angleDiff(
                    std::atan2(this->currentEstimatedTransformation(1, 0), this->currentEstimatedTransformation(0, 0)),
                    initialGuessAngle);




            //only if angle diff is smaller than 40 degree its ok. Small safety measurement
            if (abs(differenceAngleBeforeAfter) < 40.0 / 180.0 * M_PI) {
                //inverse the transformation because we want the robot transformation, not the scan transformation
                Eigen::Matrix4d transformationEstimationRobot1_2 = this->currentEstimatedTransformation;
                Eigen::Quaterniond qTMP(transformationEstimationRobot1_2.block<3, 3>(0, 0));

                graphSaved.addEdge(indexStart2,
                                   indexStart1,
                                   transformationEstimationRobot1_2.block<3, 1>(0, 3), qTMP,
                                   covarianceEstimation,
                                   LOOP_CLOSURE);

            } else {
                std::cout << "we just skipped that registration" << std::endl;
            }
            std::cout << "loopClosure: " << std::endl;
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0;
            double typeALCTime = timeToCalculate;
            std::cout << "Type A LC: " << typeALCTime << std::endl;

            begin = std::chrono::steady_clock::now();

            ////////////// look for loop closure  //////////////
            slamToolsRos::loopDetectionByClosestPath(this->graphSaved, this->scanRegistrationObject,
                                                     this->dimensionOfRegistration, this->removeDistanceToRobot,
                                                     this->radiusOfScanSize, false,
                                                     true,
                                                     0.1, this->loopClosureDistance);
            end = std::chrono::steady_clock::now();

            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0;
            double typeBLCTime = timeToCalculate;
            std::cout << "Type B LC: " << typeBLCTime << std::endl;
            begin = std::chrono::steady_clock::now();
            //optimization of graph
            this->graphSaved.isam2OptimizeGraph(true, 1);
            end = std::chrono::steady_clock::now();

            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0;
            double optimizationTimeCalculation = timeToCalculate;
            std::cout << "Optimization: " << optimizationTimeCalculation << std::endl;
            slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherPathOverTime,
                                                    this->publisherMarkerArray, this->sigmaScaling,
                                                    this->publisherPoseSLAM, this->publisherMarkerArrayLoopClosures);
            std::cout << "other accumulated Time: " << this->computationTime << std::endl;


            this->computationTime = 0;
            std::cout << "next: " << std::endl;

        }
//        this->graphSaved.isam2OptimizeGraph(true,1);
        slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherPathOverTime,
                                                this->publisherMarkerArray, this->sigmaScaling,
                                                this->publisherPoseSLAM, this->publisherMarkerArrayLoopClosures);
//        std::cout << "huhu3" << std::endl;
    }

    void stateEstimationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(this->stateEstimationMutex);
        double currentTimeStamp = msg->header.stamp.toSec();
        // calculate where to put the current new message
        int i = 0;
        if (!this->timeVector.empty()) {
            i = this->timeVector.size();
            while (this->timeVector[i - 1] > currentTimeStamp) {
                i--;
            }
        }

        if (i == this->timeVector.size() || i == 0) {
            Eigen::Quaterniond tmpQuad(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
            this->rotationVector.push_back(tmpQuad);
            this->timeVector.push_back(msg->header.stamp.toSec());
            this->xPositionVector.push_back(msg->pose.pose.position.x);
            this->yPositionVector.push_back(msg->pose.pose.position.y);
            this->zPositionVector.push_back(msg->pose.pose.position.z);
        } else {
            std::cout << "we test it" << std::endl;
            exit(0);
        }
    }

    bool saveGraph(commonbluerovmsg::saveGraph::Request &req, commonbluerovmsg::saveGraph::Response &res) {
        this->createMapAndSaveToFile();
        //create image without motion compensation
        //create image with motion compensation(saved images)
        //create image with SLAM compensation
        std::cout << "test for saving1" << std::endl;
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
        std::cout << "test for saving2" << std::endl;

        std::ofstream myFile1;
        myFile1.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/positionEstimationOverTime" +
                std::string(NAME_OF_CURRENT_METHOD) + ".csv");


        for (int k = 0; k < this->graphSaved.getVertexList()->size(); k++) {

            Eigen::Matrix4d tmpMatrix1 = this->graphSaved.getVertexList()->at(k).getTransformation();
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    myFile1 << tmpMatrix1(i, j) << " ";//number of possible rotations
                }
                myFile1 << "\n";
            }


        }

        myFile1.close();

        res.saved = true;
        return true;
    }

    bool waitForEKFMessagesToArrive2(double timeDiffWait, double endStampOfNewMessage) {

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//        std::cout << "tmp1" << std::endl;
        while (this->timeVector.empty() || this->timeVector.back() < endStampOfNewMessage) {
            ros::Duration(0.002).sleep();
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//            std::cout << timeToCalculate << std::endl;
            if (timeToCalculate > timeDiffWait * 1000) {
                return false;
            }
        }
//        std::cout << timeToCalculate << std::endl;
        return true;
    }




    void createMapAndSaveToFile() {

        std::vector<intensityValues> dataSet;
        double maximumIntensity = slamToolsRos::getDatasetFromGraphForMap(dataSet, this->graphSaved,
                                                                          this->graphSlamMutex);
        //homePosition is 0 0
        //size of mapData is defined in this->NSizeMap

        int *voxelDataIndex;
        voxelDataIndex = (int *) malloc(sizeof(int) * this->NSizeMap * this->NSizeMap);
        double *mapData;
        mapData = (double *) malloc(sizeof(double) * this->NSizeMap * this->NSizeMap);
        //set zero voxel and index
        for (int i = 0; i < this->NSizeMap * this->NSizeMap; i++) {
            voxelDataIndex[i] = 0;
            mapData[i] = 0;
        }

        for (int currentPosition = 0;
             currentPosition < dataSet.size(); currentPosition++) {
            //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.
            //was 90 yaw and 180 roll

            Eigen::Matrix4d transformationOfIntensityRay =
                    generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 0.0 / 180.0 * M_PI) *
                    generalHelpfulTools::getTransformationMatrixFromRPY(0.0 / 180.0 * M_PI, 0, 0) *
                    dataSet[currentPosition].transformation;
            //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             dataSet[currentPosition].intensity.angle);

            int ignoreDistance = (int) (this->removeDistanceToRobot / (dataSet[currentPosition].intensity.range /
                                                                       ((double) dataSet[currentPosition].intensity.intensities.size())));


            for (int j = ignoreDistance;
                 j <
                 dataSet[currentPosition].intensity.intensities.size(); j++) {
                double distanceOfIntensity =
                        j / ((double) dataSet[currentPosition].intensity.intensities.size()) *
                        ((double) dataSet[currentPosition].intensity.range);

                int incrementOfScan = dataSet[currentPosition].intensity.increment;
                for (int l = -incrementOfScan - 5; l <= incrementOfScan + 5; l++) {
                    Eigen::Vector4d positionOfIntensity(
                            distanceOfIntensity,
                            0,
                            0,
                            1);
                    double rotationOfPoint = l / 400.0;
                    Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(0,
                                                                                                                0,
                                                                                                                rotationOfPoint);
                    positionOfIntensity = rotationForBetterView * positionOfIntensity;

                    positionOfIntensity =
                            transformationOfIntensityRay * rotationOfSonarAngleMatrix * positionOfIntensity;
                    //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
                    int indexX =
                            (int) (positionOfIntensity.x() / (this->mapDimension / 2) * this->NSizeMap /
                                   2) +
                            this->NSizeMap / 2;
                    int indexY =
                            (int) (positionOfIntensity.y() / (this->mapDimension / 2) * this->NSizeMap /
                                   2) +
                            this->NSizeMap / 2;


                    if (indexX < this->NSizeMap && indexY < this->NSizeMap && indexY >= 0 &&
                        indexX >= 0) {
                        //                    std::cout << indexX << " " << indexY << std::endl;
                        //if index fits inside of our data, add that data. Else Ignore
                        voxelDataIndex[indexX + this->NSizeMap * indexY] =
                                voxelDataIndex[indexX + this->NSizeMap * indexY] + 1;
                        //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                        mapData[indexX + this->NSizeMap * indexY] =
                                mapData[indexX + this->NSizeMap * indexY] +
                                dataSet[currentPosition].intensity.intensities[j];
                        //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                        //                    std::cout << "random: " << std::endl;
                    }
                }
            }

        }


        //make sure next iteration the correct registrationis calculated
        //TO THE END
        //NOW: TO THE BEGINNING


        double maximumOfVoxelData = 0;
        double minimumOfVoxelData = INFINITY;

        for (int i = 0; i < this->NSizeMap * this->NSizeMap; i++) {
            if (voxelDataIndex[i] > 0) {
                mapData[i] = mapData[i] / voxelDataIndex[i];
                if (maximumOfVoxelData < mapData[i]) {
                    maximumOfVoxelData = mapData[i];
                }
                if (minimumOfVoxelData > mapData[i]) {
                    minimumOfVoxelData = mapData[i];
                }
                //std::cout << voxelData[i] << std::endl;
            }
        }


        for (int i = 0; i < this->NSizeMap * this->NSizeMap; i++) {

            mapData[i] = (mapData[i] - minimumOfVoxelData) / (maximumOfVoxelData - minimumOfVoxelData) * 250;
        }


        std::ofstream myFile1;
        myFile1.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/currentMap" +
                std::string(NAME_OF_CURRENT_METHOD) + ".csv");
        for (int j = 0; j < this->NSizeMap; j++) {
            for (int i = 0; i < this->NSizeMap; i++) {
                myFile1 << mapData[j + this->NSizeMap * i] << std::endl;//number of possible rotations
            }
        }

        myFile1.close();


    }


public:

    void createImageOfAllScans() {

        std::vector<intensityValues> dataSet;
        double maximumIntensity = slamToolsRos::getDatasetFromGraphForMap(dataSet, this->graphSaved,
                                                                          this->graphSlamMutex);
        //homePosition is 0 0
        //size of mapData is defined in this->NSizeMap

        int *voxelDataIndex;
        voxelDataIndex = (int *) malloc(sizeof(int) * this->NSizeMap * this->NSizeMap);
        double *mapData;
        mapData = (double *) malloc(sizeof(double) * this->NSizeMap * this->NSizeMap);
        //set zero voxel and index
        for (int i = 0; i < this->NSizeMap * this->NSizeMap; i++) {
            voxelDataIndex[i] = 0;
            mapData[i] = 0;
        }

        for (int currentPosition = 0;
             currentPosition < dataSet.size(); currentPosition++) {
            //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.
            //was 90 yaw and 180 roll

            Eigen::Matrix4d transformationOfIntensityRay =
                    generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 0.0 / 180.0 * M_PI) *
                    generalHelpfulTools::getTransformationMatrixFromRPY(0.0 / 180.0 * M_PI, 0, 0) *
                    dataSet[currentPosition].transformation;
            //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             dataSet[currentPosition].intensity.angle);

            int ignoreDistance = (int) (this->removeDistanceToRobot / (dataSet[currentPosition].intensity.range /
                                                                       ((double) dataSet[currentPosition].intensity.intensities.size())));


            for (int j = ignoreDistance;
                 j <
                 dataSet[currentPosition].intensity.intensities.size(); j++) {
                double distanceOfIntensity =
                        j / ((double) dataSet[currentPosition].intensity.intensities.size()) *
                        ((double) dataSet[currentPosition].intensity.range);

                int incrementOfScan = dataSet[currentPosition].intensity.increment;
                for (int l = -incrementOfScan - 5; l <= incrementOfScan + 5; l++) {
                    Eigen::Vector4d positionOfIntensity(
                            distanceOfIntensity,
                            0,
                            0,
                            1);
                    double rotationOfPoint = l / 400.0;
                    Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(0,
                                                                                                                0,
                                                                                                                rotationOfPoint);
                    positionOfIntensity = rotationForBetterView * positionOfIntensity;

                    positionOfIntensity =
                            transformationOfIntensityRay * rotationOfSonarAngleMatrix * positionOfIntensity;
                    //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
                    int indexX =
                            (int) (positionOfIntensity.x() / (this->mapDimension / 2) * this->NSizeMap /
                                   2) +
                            this->NSizeMap / 2;
                    int indexY =
                            (int) (positionOfIntensity.y() / (this->mapDimension / 2) * this->NSizeMap /
                                   2) +
                            this->NSizeMap / 2;


                    if (indexX < this->NSizeMap && indexY < this->NSizeMap && indexY >= 0 &&
                        indexX >= 0) {
                        //                    std::cout << indexX << " " << indexY << std::endl;
                        //if index fits inside of our data, add that data. Else Ignore
                        voxelDataIndex[indexX + this->NSizeMap * indexY] =
                                voxelDataIndex[indexX + this->NSizeMap * indexY] + 1;
                        //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                        mapData[indexX + this->NSizeMap * indexY] =
                                mapData[indexX + this->NSizeMap * indexY] +
                                dataSet[currentPosition].intensity.intensities[j];
                        //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                        //                    std::cout << "random: " << std::endl;
                    }
                }
            }

        }


        //make sure next iteration the correct registrationis calculated
        //TO THE END
        //NOW: TO THE BEGINNING


        double maximumOfVoxelData = 0;
        double minimumOfVoxelData = INFINITY;

        for (int i = 0; i < this->NSizeMap * this->NSizeMap; i++) {
            if (voxelDataIndex[i] > 0) {
                mapData[i] = mapData[i] / voxelDataIndex[i];
                if (maximumOfVoxelData < mapData[i]) {
                    maximumOfVoxelData = mapData[i];
                }
                if (minimumOfVoxelData > mapData[i]) {
                    minimumOfVoxelData = mapData[i];
                }
                //std::cout << voxelData[i] << std::endl;
            }
        }


        for (int i = 0; i < this->NSizeMap * this->NSizeMap; i++) {

            mapData[i] = (mapData[i] - minimumOfVoxelData) / (maximumOfVoxelData - minimumOfVoxelData) * 250;
        }


        nav_msgs::OccupancyGrid occupanyMap;
        occupanyMap.header.frame_id = "map_ned";
        occupanyMap.info.height = this->NSizeMap;
        occupanyMap.info.width = this->NSizeMap;
        occupanyMap.info.resolution = this->mapDimension / this->NSizeMap;
        occupanyMap.info.origin.position.x = -this->mapDimension / 2;
        occupanyMap.info.origin.position.y = -this->mapDimension / 2;
        occupanyMap.info.origin.position.z = +0.5;
        for (int i = 0; i < this->NSizeMap; i++) {
            for (int j = 0; j < this->NSizeMap; j++) {
                //determine color:
                occupanyMap.data.push_back((int) (mapData[j + this->NSizeMap * i]));
            }
        }
        this->publisherOccupancyMap.publish(occupanyMap);
        free(voxelDataIndex);
        free(mapData);
    }


};


int main(int argc, char **argv) {


    ros::init(argc, argv, "slamalgorithm");


    ros::V_string nodes;
    int i = 0;
    std::string stringForRosClass;


    ros::start();
    ros::NodeHandle n_;

    int dimensionOfRegistration;
    n_.getParam("rosslamexp1/dimensionOfRegistration", dimensionOfRegistration);
    double radiusOfScanSize;
    n_.getParam("rosslamexp1/radiusOfScanSize", radiusOfScanSize);
    int NSizeMap;
    n_.getParam("rosslamexp1/NSizeMap", NSizeMap);
    double mapDimension;
    n_.getParam("rosslamexp1/mapDimension", mapDimension);
    double howOftenRegistrationPerFullScan;
    n_.getParam("rosslamexp1/howOftenRegistrationPerFullScan", howOftenRegistrationPerFullScan);
    double loopClosureDistance;
    n_.getParam("rosslamexp1/loopClosureDistance", loopClosureDistance);
    bool reverseScanDirection;
    n_.getParam("rosslamexp1/reverseScanDirection", reverseScanDirection);
    double rotationSonar;
    n_.getParam("rosslamexp1/rotationSonar", rotationSonar);
    double removeDistanceToRobot;
    n_.getParam("rosslamexp1/removeDistanceToRobot", removeDistanceToRobot);



    rosClassEKF rosClassForTests(n_, dimensionOfRegistration, radiusOfScanSize, NSizeMap, mapDimension,
                                 howOftenRegistrationPerFullScan, loopClosureDistance, reverseScanDirection,
                                 rotationSonar, removeDistanceToRobot);


//    ros::spin();


    ros::Rate loop_rate(0.1);
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::Duration(10).sleep();

    while (ros::ok()) {
//        ros::spinOnce();

        //rosClassForTests.updateHilbertMap();
//        rosClassForTests.updateMap();
        rosClassForTests.createImageOfAllScans();

        loop_rate.sleep();

        //std::cout << ros::Time::now() << std::endl;
    }


    return (0);
}
