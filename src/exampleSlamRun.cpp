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

//#define this->dimensionOfRegistration 128
//#define this->radiusOfScanSize 10 // was 50 //tuhh tank 6 // 10 in OL tank
//#define NUMBER_OF_POINTS_MAP 512//was 512
// 80 simulation 300 valentin 45.0 for Keller 10.0 TUHH TANK 15.0 Ocean
//#define DIMENSION_OF_MAP 15.0

//#define IGNORE_DISTANCE_TO_ROBOT 0.3 // was 1.0 // TUHH 0.2 // 0.3 ocean
//#define DEBUG_REGISTRATION false

//#define ROTATION_SONAR M_PI // sonar on robot M_PI // simulation 0
//#define SHOULD_USE_ROSBAG true
//#define FACTOR_OF_MATCHING 1.0 //1.5
//#define THRESHOLD_FOR_TRANSLATION_MATCHING 0.1 // standard is 0.1, 0.05 und 0.01  // 0.05 for valentin Oben

//#define INTEGRATED_NOISE_XY 0.03 // was 0.03  // TUHH 0.005
//#define INTEGRATED_NOISE_YAW 0.03 // was 0.03 // TUHH 0.005

//#define USE_INITIAL_TRANSLATION_LOOP_CLOSURE true
//#define MAXIMUM_LOOP_CLOSURE_DISTANCE 0.4 // 0.2 TUHH 2.0 valentin Keller 4.0 Valentin Oben // 2.0 simulation // 0.4 OceanLab

//#define TUHH_SYSTEM false
//#define SIMULATION_SYSTEM false


//#define NAME_OF_CURRENT_METHOD "oceanLabExp1"
//#define SAVE_CALCULATION_FOLDER "/home/ubuntu/timeMeasurements/exp1TimesSaved.csv"

//#define NAME_OF_CURRENT_METHOD "_circle_dead_reckoning_"
//#define NAME_OF_CURRENT_METHOD "_circle_dead_reckoning_wsm_"
//#define NAME_OF_CURRENT_METHOD "_circle_dynamic_slam_1_0_"
//#define NAME_OF_CURRENT_METHOD "_circle_dynamic_slam_4_0_"
//#define NAME_OF_CURRENT_METHOD "_video_4_0_"

//#define NAME_OF_CURRENT_METHOD "_TEST2_classical_slam_"

//occupancyMap(256, this->dimensionOfRegistration, 70, hilbertMap::HINGED_FEATURES)
class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_,int dimensionOfRegistration,double radiusOfScanSize,int NSizeMap, double mapDimension,double howOftenRegistrationPerFullScan, double loopClosureDistance,bool reverseScanDirection,double rotationSonar,double removeDistanceToRobot) : graphSaved(3, INTENSITY_BASED_GRAPH),
                                                                               scanRegistrationObject(
                                                                                       dimensionOfRegistration,
                                                                                       dimensionOfRegistration / 2,
                                                                                       dimensionOfRegistration / 2,
                                                                                       dimensionOfRegistration / 2 -
                                                                                       1) {

        this->subscriberEKF = n_.subscribe("publisherPoseEkf", 10000, &rosClassEKF::stateEstimationCallback, this);
        ros::Duration(2).sleep();
        this->subscriberIntensitySonar = n_.subscribe("sonar/intensity", 10000, &rosClassEKF::scanCallback, this);
        this->serviceSaveGraph = n_.advertiseService("saveGraphOfSLAM", &rosClassEKF::saveGraph, this);
        this->subscriberPositionGT = n_.subscribe("positionGT", 100000, &rosClassEKF::groundTruthEvaluationCallback,
                                                  this);
        this->subscriberPositionGTGantry = n_.subscribe("gantry/path_follower/current_position", 100000, &rosClassEKF::groundTruthEvaluationTUHHCallback,
                                                        this);

        publisherPathOverTime = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
        publisherPathOverTimeGT = n_.advertise<nav_msgs::Path>("positionOverTimeGT", 10);
        publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);
        publisherMarkerArrayLoopClosures = n_.advertise<visualization_msgs::MarkerArray>("loopClosures", 10);
        publisherOccupancyMap = n_.advertise<nav_msgs::OccupancyGrid>("occupancyHilbertMap", 10);

        publisherPoseSLAM = n_.advertise<geometry_msgs::PoseStamped>("slamEndPose", 10);



        this->sigmaScaling = 0.1;// was 1.0
        this->firstSonarInput = true;
        this->firstCompleteSonarScan = true;
        this->numberOfTimesFirstScan = 0;
        this->saveGraphStructure = false;
//        this->maxTimeOptimization = 1.0;
        this->currentGTPosition = Eigen::Matrix4d::Identity();

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

        this->dimensionOfRegistration=dimensionOfRegistration;
        this->radiusOfScanSize=radiusOfScanSize;
        this->NSizeMap=NSizeMap;
        this->mapDimension=mapDimension;
        this->howOftenRegistrationPerFullScan=howOftenRegistrationPerFullScan;
        this->loopClosureDistance=loopClosureDistance;
        this->reverseScanDirection=reverseScanDirection;
        this->rotationSonar=rotationSonar;
        this->removeDistanceToRobot=removeDistanceToRobot;
        
    }


private:
    nav_msgs::OccupancyGrid map;


    ros::Subscriber subscriberEKF, subscriberIntensitySonar, subscriberPositionGT,subscriberPositionGTGantry;
    ros::Publisher publisherPoseSLAM, publisherOccupancyMap;
    ros::ServiceServer serviceSaveGraph;
    ros::ServiceClient pauseRosbag;
    std::mutex stateEstimationMutex;
    std::mutex groundTruthMutex;
    std::mutex graphSlamMutex;

    //GraphSlam things
    ros::Publisher publisherKeyFrameClouds, publisherPathOverTime, publisherMarkerArray, publisherPathOverTimeGT, publisherMarkerArrayLoopClosures, publisherLastPCL, publisherRegistrationPCL, publisherBeforeCorrection, publisherAfterCorrection;

    //Matrices:
    Eigen::Matrix4d currentEstimatedTransformation;
    Eigen::Matrix4d initialGuessTransformation;


    //EKF savings
    std::deque<edge> posDiffOverTimeEdges;
    std::deque<double> xPositionVector, yPositionVector, zPositionVector, timeVector;//,yawAngleVector,pitchAngleVector,rollAngleVector;
    std::deque<Eigen::Quaterniond> rotationVector;

    // GT savings
    std::deque<transformationStamped> currentPositionGTDeque;
    Eigen::Matrix4d currentGTPosition;

    double sigmaScaling;

    graphSlamSaveStructure graphSaved;
    scanRegistrationClass scanRegistrationObject;
    bool firstSonarInput, firstCompleteSonarScan, saveGraphStructure;
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
//        std::cout << "huhu1" << std::endl;
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
//        std::cout << "huhu2" << std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        intensityMeasurement intensityTMP;
        if(this->reverseScanDirection){
            intensityTMP.angle = std::fmod(-msg->angle / 400.0 * M_PI * 2.0 + this->rotationSonar, M_PI * 2);// TEST TRYING OUT -
        }else{
            intensityTMP.angle = std::fmod(msg->angle / 400.0 * M_PI * 2.0 + this->rotationSonar, M_PI * 2);// TEST TRYING OUT -
        }

        intensityTMP.time = msg->header.stamp.toSec();
        intensityTMP.range = msg->range;
        intensityTMP.increment = msg->step_size;
        std::vector<double> intensitiesVector;
        for (int i = 0; i < msg->intensities.size(); i++) {
            intensitiesVector.push_back(msg->intensities[i]);
        }
        intensityTMP.intensities = intensitiesVector;

        if (firstSonarInput) {

            this->graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                                       Eigen::Matrix3d::Zero(), intensityTMP, msg->header.stamp.toSec(),
                                       FIRST_ENTRY);
            firstSonarInput = false;
            return;
        }
        //add a new edge and vertex to the graph defined by EKF and Intensity Measurement

        bool waitingForMessages = waitForEKFMessagesToArrive2(0.1,msg->header.stamp.toSec());
        if(!waitingForMessages){
            std::cout << "return no message found: " << msg->header.stamp.toSec() << "    " << ros::Time::now().toSec()<< std::endl;
            return;
        }
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


        this->graphSaved.addVertex(this->graphSaved.getVertexList()->back().getKey() + 1, pos, rot,
                                   this->graphSaved.getVertexList()->back().getCovarianceMatrix(),
                                   intensityTMP,
                                   msg->header.stamp.toSec(),
                                   INTENSITY_SAVED);


        Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
        covarianceMatrix(0, 0) = 0.03;
        covarianceMatrix(1, 1) = 0.03;
        covarianceMatrix(2, 2) = 0.03;
        this->graphSaved.addEdge(this->graphSaved.getVertexList()->back().getKey() - 1,
                                 this->graphSaved.getVertexList()->back().getKey(),
                                 differenceOfEdge.getPositionDifference(), differenceOfEdge.getRotationDifference(),
                                 covarianceMatrix, INTEGRATED_POSE);


        int indexOfLastKeyframe;
        double angleDiff = slamToolsRos::angleBetweenLastKeyframeAndNow(this->graphSaved);// i think this is always true
//        std::cout << angleDiff << std::endl;
        // best would be scan matching between this angle and transformation based last angle( i think this is currently done)

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000.0;
        this->computationTime += timeToCalculate;
        if (abs(angleDiff) > 2 * M_PI / this->howOftenRegistrationPerFullScan) {


            this->graphSaved.getVertexList()->back().setTypeOfVertex(INTENSITY_SAVED_AND_KEYFRAME);
            if (firstCompleteSonarScan) {
                numberOfTimesFirstScan++;
                if (numberOfTimesFirstScan > 2 * this->howOftenRegistrationPerFullScan - 1) {
                    firstCompleteSonarScan = false;
                }
                return;
            }
            
            begin = std::chrono::steady_clock::now();

            //angleDiff = angleBetweenLastKeyframeAndNow(false);
//            indexOfLastKeyframe = slamToolsRos::getLastIntensityKeyframe(this->graphSaved);
            int indexStart1, indexEnd1, indexStart2, indexEnd2;
            slamToolsRos::calculateStartAndEndIndexForVoxelCreation(
                    this->graphSaved.getVertexList()->back().getKey() - 5, indexStart1, indexEnd1, this->graphSaved);
            indexStart2 = indexEnd1;
            slamToolsRos::calculateEndIndexForVoxelCreationByStartIndex(indexStart2, indexEnd2, this->graphSaved);


            std::cout << "scanAcusitionTime: " << this->graphSaved.getVertexList()->at(indexStart2).getTimeStamp()-this->graphSaved.getVertexList()->at(indexEnd2).getTimeStamp() << std::endl;

            //we inverse the initial guess, because the registration creates a T from scan 1 to scan 2.
            // But the graph creates a transformation from 1 -> 2 by the robot, therefore inverse.
            this->initialGuessTransformation =
                    (this->graphSaved.getVertexList()->at(indexStart2).getTransformation().inverse() *
                     this->graphSaved.getVertexList()->at(indexStart1).getTransformation());

//            std::cout << "this->initialGuessTransformation.inverse()" << std::endl;
//            std::cout << this->initialGuessTransformation.inverse() << std::endl;

            double initialGuessAngle = std::atan2(this->initialGuessTransformation(1, 0),
                                                  this->initialGuessTransformation(0, 0));

            double *voxelData1;
            double *voxelData2;
            voxelData1 = (double *) malloc(sizeof(double) * this->dimensionOfRegistration * this->dimensionOfRegistration);
            voxelData2 = (double *) malloc(sizeof(double) * this->dimensionOfRegistration * this->dimensionOfRegistration);

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
            std::cout << "direct matching consecutive: " << std::endl;
            // result is matrix to transform scan 1 to scan 2 therefore later inversed + initial guess inversed


            this->currentEstimatedTransformation = slamToolsRos::registrationOfTwoVoxelsFast(voxelData1, voxelData2,
                                                                                             this->initialGuessTransformation,
                                                                                             covarianceEstimation, true,
                                                                                             true,
                                                                                             (double) this->radiusOfScanSize /
                                                                                             (double) this->dimensionOfRegistration,
                                                                                             false, scanRegistrationObject,
                                                                                             false,
                                                                                             THRESHOLD_FOR_TRANSLATION_MATCHING);

            slamToolsRos::saveResultingRegistrationTMPCOPY(indexStart1, indexEnd1, indexStart2, indexEnd2,
                                                           this->graphSaved, this->dimensionOfRegistration,
                                                           IGNORE_DISTANCE_TO_ROBOT,
                                                           this->radiusOfScanSize,
                                                           DEBUG_REGISTRATION, this->currentEstimatedTransformation,
                                                           initialGuessTransformation);
//            slamToolsRos::saveResultingRegistration(indexStart1, indexStart2,
//                                                    this->graphSaved, this->dimensionOfRegistration,
//                                                    IGNORE_DISTANCE_TO_ROBOT, this->radiusOfScanSize,
//                                                    DEBUG_REGISTRATION, this->currentTransformation);


            double differenceAngleBeforeAfter = generalHelpfulTools::angleDiff(
                    std::atan2(this->currentEstimatedTransformation(1, 0), this->currentEstimatedTransformation(0, 0)),
                    initialGuessAngle);


//            std::cout << "currentTransformation:" << std::endl;
//            std::cout << this->currentTransformation << std::endl;
//            std::cout << "initial Guess Transformation:" << std::endl;
//            std::cout << this->initialGuessTransformation << std::endl;
//
//            std::cout << "Initial guess angle: "
//                      << initialGuessAngle * 180 / M_PI
//                      << std::endl;
//            std::cout << "Registration angle: "
//                      << std::atan2(this->currentEstimatedTransformation(1, 0),
//                                    this->currentEstimatedTransformation(0, 0)) * 180 / M_PI
//                      << std::endl;
//            std::cout << "difference of angle after Registration: " << differenceAngleBeforeAfter * 180 / M_PI
//                      << std::endl;
////
//            std::cout << "Input In Graph:" << std::endl;
//            std::cout << this->currentEstimatedTransformation << std::endl;
//            std::cout << "Initial Guess From Graph:" << std::endl;
//            std::cout << this->graphSaved.getVertexList()->at(indexStart2).getTransformation().inverse() *
//                         this->graphSaved.getVertexList()->at(indexStart1).getTransformation()
//                      << std::endl;
//            std::cout << covarianceEstimation << std::endl;

            //only if angle diff is smaller than 40 degreece its ok
            if (abs(differenceAngleBeforeAfter) < 40.0 / 180.0 * M_PI) {
                //inverse the transformation because we want the robot transformation, not the scan transformation
                Eigen::Matrix4d transformationEstimationRobot1_2 = this->currentEstimatedTransformation;
                Eigen::Quaterniond qTMP(transformationEstimationRobot1_2.block<3, 3>(0, 0));

                graphSaved.addEdge(indexStart2,
                                   indexStart1,
                                   transformationEstimationRobot1_2.block<3, 1>(0, 3), qTMP,
                                   covarianceEstimation,
                                   LOOP_CLOSURE);//@TODO still not sure about size

            } else {
                std::cout << "we just skipped that registration" << std::endl;
            }
            std::cout << "loopClosure: " << std::endl;
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000.0;
            double typeALCTime = timeToCalculate;
            std::cout << "Type A LC: " << typeALCTime << std::endl;

            begin = std::chrono::steady_clock::now();

            ////////////// look for loop closure  //////////////
            slamToolsRos::loopDetectionByClosestPath(this->graphSaved, this->scanRegistrationObject,
                                                     this->dimensionOfRegistration, IGNORE_DISTANCE_TO_ROBOT,
                                                     this->radiusOfScanSize, DEBUG_REGISTRATION,
                                                     USE_INITIAL_TRANSLATION_LOOP_CLOSURE,
                                                     THRESHOLD_FOR_TRANSLATION_MATCHING, MAXIMUM_LOOP_CLOSURE_DISTANCE);
            end = std::chrono::steady_clock::now();

            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000.0;
            double typeBLCTime = timeToCalculate;
            std::cout << "Type B LC: " << typeBLCTime << std::endl;
            begin = std::chrono::steady_clock::now();

            this->graphSaved.isam2OptimizeGraph(true, 2);
            end = std::chrono::steady_clock::now();

            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000.0;
            double optimizationTimeCalculation = timeToCalculate;
            std::cout << "Optimization: " << optimizationTimeCalculation << std::endl;
            slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherPathOverTime,
                                                    this->publisherMarkerArray, this->sigmaScaling,
                                                    this->publisherPoseSLAM, this->publisherMarkerArrayLoopClosures);
            //            this->graphSaved.classicalOptimizeGraph(true);
            std::cout << "other accumulated Time: " << this->computationTime << std::endl;


            std::ofstream saveCalculationTimes;
            saveCalculationTimes.open(std::string(SAVE_CALCULATION_FOLDER), std::ios_base::app);
//            saveCalculationTimes.open(std::string("/home/tim-external/dataSets/computationTimes.csv"), std::ios_base::app);
            saveCalculationTimes << optimizationTimeCalculation<<","<< typeALCTime<< ","<< typeBLCTime<< "," << this->computationTime<< "," << this->graphSaved.getVertexList()->back().getTimeStamp()<<'\n';



            saveCalculationTimes.close();



            this->computationTime = 0;
            std::cout << "next: " << std::endl;

            if (SHOULD_USE_ROSBAG) {
                std_srvs::SetBool srv;
                srv.request.data = false;
                pauseRosbag.call(srv);
            }

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

        std::ofstream myFile1, myFile2;
        myFile1.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/positionEstimationOverTime" + std::string(NAME_OF_CURRENT_METHOD)+ ".csv");
        myFile2.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/groundTruthOverTime" + std::string(NAME_OF_CURRENT_METHOD)+ ".csv");


        for (int k = 0; k < this->graphSaved.getVertexList()->size(); k++) {

            Eigen::Matrix4d tmpMatrix1 = this->graphSaved.getVertexList()->at(k).getTransformation();
            Eigen::Matrix4d tmpMatrix2 = this->graphSaved.getVertexList()->at(k).getGroundTruthTransformation();
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    myFile1 << tmpMatrix1(i, j) << " ";//number of possible rotations
                }
                myFile1 << "\n";
            }


            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    myFile2 << tmpMatrix2(i, j) << " ";//number of possible rotations
                }
                myFile2 << "\n";
            }

        }


        myFile1.close();
        myFile2.close();

        res.saved = true;
        return true;
    }
    bool waitForEKFMessagesToArrive2(double timeDiffWait,double endStampOfNewMessage) {

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//        std::cout << "tmp1" << std::endl;
        while (this->timeVector.empty() || this->timeVector.back()<endStampOfNewMessage) {
            ros::Duration(0.002).sleep();
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//            std::cout << timeToCalculate << std::endl;
            if(timeToCalculate > timeDiffWait*1000){
                return false;
            }
        }
//        std::cout << timeToCalculate << std::endl;
        return true;
    }

    bool waitForEKFMessagesToArrive(double timeUntilWait) {

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//        std::cout << "tmp1" << std::endl;
        while (this->timeVector.empty() && timeToCalculate<10) {
            ros::Duration(0.002).sleep();
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        }
//        std::cout << "tmp2" << std::endl;
        while (timeUntilWait > timeVector[timeVector.size() - 1] ) {
            ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf",ros::Duration(10));
            ros::Duration(0.001).sleep();

            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//            std::cout << timeToCalculate << std::endl;
            double timeToWait = 10;
            if(SIMULATION_SYSTEM){
                timeToWait = 20;
            }
            if(timeToCalculate>timeToWait){
                break;
            }
        }
//        std::cout << "tmp3" << std::endl;
//        ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf");
        ros::Duration(0.002).sleep();
//        std::cout << "tmp4" << std::endl;
        end = std::chrono::steady_clock::now();
        timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//        std::cout << timeToCalculate << std::endl;
        double timeToWait = 8;
        if(SIMULATION_SYSTEM){
            timeToWait = 40;
        }
        if(timeToCalculate>timeToWait){
            return false;
        }else{
            return true;
        }
    }

    void groundTruthEvaluationCallback(const commonbluerovmsg::staterobotforevaluation::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        //first time? calc current Position
        Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(msg->roll, msg->pitch,
                                                                                        msg->yaw);
        tmpMatrix(0, 3) = msg->xPosition;
        tmpMatrix(1, 3) = msg->yPosition;
        tmpMatrix(2, 3) = msg->zPosition;
        transformationStamped tmpValue;
        tmpValue.transformation = tmpMatrix;
        tmpValue.timeStamp = msg->header.stamp.toSec();
        this->currentPositionGTDeque.push_back(tmpValue);
    }

    void groundTruthEvaluationTUHHCallback(const geometry_msgs::Point::ConstPtr &msg) {


//        std::cout << "test" << std::endl;
        //first time? calc current Position
        Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0,0,0);
        tmpMatrix(0, 3) = msg->x;
        tmpMatrix(1, 3) = msg->y;
        tmpMatrix(2, 3) = msg->z;
//        std:: cout << tmpMatrix << std::endl;
        transformationStamped tmpValue;
        tmpValue.transformation = tmpMatrix;
        tmpValue.timeStamp = ros::Time::now().toSec();

        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        this->currentGTPosition = tmpMatrix;

//        this->currentPositionGTDeque.push_back(tmpValue);
    }

    Eigen::Matrix4d getCurrentGTPosition(){
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        return this->currentGTPosition;
    }

    void saveCurrentGTPosition() {
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        while (!this->currentPositionGTDeque.empty()) {
            double currentTimeStampOfInterest = this->currentPositionGTDeque[0].timeStamp;
//            std::cout << currentTimeStampOfInterest << std::endl;
            int i = this->graphSaved.getVertexList()->size() - 1;
            while (this->graphSaved.getVertexList()->at(i).getTimeStamp() >= currentTimeStampOfInterest) {
                i--;
                if (i == -1) {
                    break;
                }
            }
            i++;
            if(i == this->graphSaved.getVertexList()->size()){
                break;
            }
//            if (i == 0) {
//                break;
//            }

//            std::cout << this->graphSaved.getVertexList()->at(i).getTimeStamp() << std::endl;
//            std::cout << currentTimeStampOfInterest << std::endl;



            //sort in
            int j = 0;
            while (this->graphSaved.getVertexList()->at(i).getTimeStamp() >= this->currentPositionGTDeque[j].timeStamp) {
                j++;
                if (j == this->currentPositionGTDeque.size()) {
                    break;
                }
            }
            if (j == this->currentPositionGTDeque.size()) {
                break;
            }
//            std::cout << this->graphSaved.getVertexList()->at(i).getTimeStamp() << std::endl;
//            std::cout << this->currentPositionGTDeque[j].timeStamp << std::endl;
            this->graphSaved.getVertexList()->at(i).setGroundTruthTransformation(this->currentPositionGTDeque[j].transformation);



            for(int k = 0 ; k<j+1;k++){
                this->currentPositionGTDeque.pop_front();
            }




//            this->currentPositionGTDeque.pop_front();
        }

    }

    void createMapAndSaveToFile(){

        std::vector<intensityValues> dataSet;
        double maximumIntensity = slamToolsRos::getDatasetFromGraphForMap(dataSet, this->graphSaved,
                                                                          this->graphSlamMutex);
        //homePosition is 0 0
        //size of mapData is defined in NUMBER_OF_POINTS_MAP

        int *voxelDataIndex;
        voxelDataIndex = (int *) malloc(sizeof(int) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
        double *mapData;
        mapData = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
        //set zero voxel and index
        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
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

            int ignoreDistance = (int) (IGNORE_DISTANCE_TO_ROBOT / (dataSet[currentPosition].intensity.range /
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
                            (int) (positionOfIntensity.x() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                   2) +
                            NUMBER_OF_POINTS_MAP / 2;
                    int indexY =
                            (int) (positionOfIntensity.y() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                   2) +
                            NUMBER_OF_POINTS_MAP / 2;


                    if (indexX < NUMBER_OF_POINTS_MAP && indexY < NUMBER_OF_POINTS_MAP && indexY >= 0 &&
                        indexX >= 0) {
                        //                    std::cout << indexX << " " << indexY << std::endl;
                        //if index fits inside of our data, add that data. Else Ignore
                        voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY] =
                                voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY] + 1;
                        //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                        mapData[indexX + NUMBER_OF_POINTS_MAP * indexY] =
                                mapData[indexX + NUMBER_OF_POINTS_MAP * indexY] +
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

        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
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


        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {

            mapData[i] = (mapData[i] - minimumOfVoxelData) / (maximumOfVoxelData - minimumOfVoxelData) * 250;
        }


        std::ofstream myFile1;
        myFile1.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/currentMap" + std::string(NAME_OF_CURRENT_METHOD)+ ".csv");
        for (int j = 0; j < NUMBER_OF_POINTS_MAP ; j++) {
            for (int i = 0; i < NUMBER_OF_POINTS_MAP; i++) {
                myFile1 << mapData[j + NUMBER_OF_POINTS_MAP * i] << std::endl;//number of possible rotations
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
        //size of mapData is defined in NUMBER_OF_POINTS_MAP

        int *voxelDataIndex;
        voxelDataIndex = (int *) malloc(sizeof(int) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
        double *mapData;
        mapData = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
        //set zero voxel and index
        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
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

            int ignoreDistance = (int) (IGNORE_DISTANCE_TO_ROBOT / (dataSet[currentPosition].intensity.range /
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
                            (int) (positionOfIntensity.x() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                   2) +
                            NUMBER_OF_POINTS_MAP / 2;
                    int indexY =
                            (int) (positionOfIntensity.y() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                   2) +
                            NUMBER_OF_POINTS_MAP / 2;


                    if (indexX < NUMBER_OF_POINTS_MAP && indexY < NUMBER_OF_POINTS_MAP && indexY >= 0 &&
                        indexX >= 0) {
                        //                    std::cout << indexX << " " << indexY << std::endl;
                        //if index fits inside of our data, add that data. Else Ignore
                        voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY] =
                                voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY] + 1;
                        //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                        mapData[indexX + NUMBER_OF_POINTS_MAP * indexY] =
                                mapData[indexX + NUMBER_OF_POINTS_MAP * indexY] +
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

        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
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


        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {

            mapData[i] = (mapData[i] - minimumOfVoxelData) / (maximumOfVoxelData - minimumOfVoxelData) * 250;
        }


        nav_msgs::OccupancyGrid occupanyMap;
        occupanyMap.header.frame_id = "map_ned";
        occupanyMap.info.height = NUMBER_OF_POINTS_MAP;
        occupanyMap.info.width = NUMBER_OF_POINTS_MAP;
        occupanyMap.info.resolution = DIMENSION_OF_MAP / NUMBER_OF_POINTS_MAP;
        occupanyMap.info.origin.position.x = -DIMENSION_OF_MAP / 2;
        occupanyMap.info.origin.position.y = -DIMENSION_OF_MAP / 2;
        occupanyMap.info.origin.position.z = +0.5;
        for (int i = 0; i < NUMBER_OF_POINTS_MAP; i++) {
            for (int j = 0; j < NUMBER_OF_POINTS_MAP; j++) {
                //determine color:
                occupanyMap.data.push_back((int) (mapData[j + NUMBER_OF_POINTS_MAP * i]));
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

    double number_to_get;
    n_.getParam("/custom_prefix/number_float", number_to_get);











    rosClassEKF rosClassForTests(n_);


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
