//
// Created by tim on 26.03.21.
//

#include "slamToolsRos.h"

void slamToolsRos::visualizeCurrentPoseGraph(graphSlamSaveStructure &graphSaved, ros::Publisher &publisherPath,
                                             ros::Publisher &publisherMarkerArray, double sigmaScaling,
                                             ros::Publisher &publisherPoseSlam, ros::Publisher &publisherLoopClosures) {

    nav_msgs::Path posOverTime;
    posOverTime.header.frame_id = "map_ned";
    Eigen::Matrix4d currentTransformation, completeTransformation;
    //pcl::PointCloud<pcl::PointXYZ> completeCloudWithPos;


    //std::vector<vertex> vertexList =;
    for (int i = 0; i < graphSaved.getVertexList()->size(); i++) {//skip the first pointCloud
        vertex vertexElement = graphSaved.getVertexList()->at(i);

        geometry_msgs::PoseStamped pos;
        pos.pose.position.x = vertexElement.getPositionVertex().x();
        pos.pose.position.y = vertexElement.getPositionVertex().y();
        pos.pose.position.z = vertexElement.getPositionVertex().z();
        pos.pose.orientation.x = vertexElement.getRotationVertex().x();
        pos.pose.orientation.y = vertexElement.getRotationVertex().y();
        pos.pose.orientation.z = vertexElement.getRotationVertex().z();
        pos.pose.orientation.w = vertexElement.getRotationVertex().w();

        posOverTime.poses.push_back(pos);


    }

    visualization_msgs::MarkerArray markerArray;
    int k = 0;
    for (int i = 0; i < graphSaved.getVertexList()->size(); i = i + 100) {//skip the first pointCloud
        vertex vertexElement = graphSaved.getVertexList()->at(i);
        Eigen::Matrix3d currentCovariance = vertexElement.getCovarianceMatrix();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(currentCovariance.block<2, 2>(0, 0));
        Eigen::Vector2d e1 = eig.eigenvectors().col(0);
        Eigen::Vector2d e2 = eig.eigenvectors().col(1);
        double l1 = eig.eigenvalues().x();
        double l2 = eig.eigenvalues().y();

        double phi;
        if (l2 >= l1) {
            phi = atan2(e2.y(), e2.x());
        } else {
            phi = atan2(e1.y(), e1.x());
        }

        visualization_msgs::Marker currentMarker;
        currentMarker.pose.position.x = vertexElement.getPositionVertex().x();
        currentMarker.pose.position.y = vertexElement.getPositionVertex().y();
        currentMarker.pose.position.z = vertexElement.getPositionVertex().z();
        Eigen::Vector3d rpyTMP = generalHelpfulTools::getRollPitchYaw(vertexElement.getRotationVertex());
        double inputAngle = generalHelpfulTools::normalizeAngle(phi + rpyTMP[2]);
        Eigen::Quaterniond tmpQuad = generalHelpfulTools::getQuaternionFromRPY(0, 0, inputAngle);


        currentMarker.pose.orientation.x = tmpQuad.x();
        currentMarker.pose.orientation.y = tmpQuad.y();
        currentMarker.pose.orientation.z = tmpQuad.z();
        currentMarker.pose.orientation.w = tmpQuad.w();
        currentMarker.header.frame_id = "map_ned";
        if (l2 >= l1) {
            currentMarker.scale.x = sigmaScaling * sqrt(l2);
            currentMarker.scale.y = sigmaScaling * sqrt(l1);
        } else {
            currentMarker.scale.x = sigmaScaling * sqrt(l1);
            currentMarker.scale.y = sigmaScaling * sqrt(l2);
        }
        currentMarker.scale.z = 0;
        currentMarker.color.r = 0;
        currentMarker.color.g = 1;
        currentMarker.color.b = 0;
        currentMarker.color.a = 0.1;
        //currentMarker.lifetime.sec = 10;
        currentMarker.type = 2;
        currentMarker.id = k;
        k++;
        markerArray.markers.push_back(currentMarker);

    }
    publisherMarkerArray.publish(markerArray);
    publisherPath.publish(posOverTime);


    geometry_msgs::PoseStamped pos;
    pos.header.stamp = ros::Time(graphSaved.getVertexList()->back().getTimeStamp());
    pos.header.frame_id = "map_ned";
    pos.pose.position.x = graphSaved.getVertexList()->back().getPositionVertex().x();
    pos.pose.position.y = graphSaved.getVertexList()->back().getPositionVertex().y();
    pos.pose.position.z = graphSaved.getVertexList()->back().getPositionVertex().z();
    pos.pose.orientation.x = graphSaved.getVertexList()->back().getRotationVertex().x();
    pos.pose.orientation.y = graphSaved.getVertexList()->back().getRotationVertex().y();
    pos.pose.orientation.z = graphSaved.getVertexList()->back().getRotationVertex().z();
    pos.pose.orientation.w = graphSaved.getVertexList()->back().getRotationVertex().w();

    publisherPoseSlam.publish(pos);




    //create marker for evey loop closure
    visualization_msgs::MarkerArray markerArrowsArray;
    int j = 0;
    for (int i = 0; i < graphSaved.getEdgeList()->size(); i++) {

        if (graphSaved.getEdgeList()->at(i).getTypeOfEdge() ==
            LOOP_CLOSURE) {//if its a loop closure then create arrow from vertex a to vertex b
            visualization_msgs::Marker currentMarker;
            currentMarker.header.frame_id = "map_ned";
            currentMarker.scale.x = 0.1;
            currentMarker.scale.y = 0.3;
            currentMarker.scale.z = 0;
            currentMarker.color.r = 0;
            currentMarker.color.g = 0;
            currentMarker.color.b = 1;
            currentMarker.color.a = 0.8;
            //currentMarker.lifetime.sec = 10;
            geometry_msgs::Point startPoint;
            geometry_msgs::Point endPoint;

            startPoint.x = graphSaved.getVertexList()->at(
                    graphSaved.getEdgeList()->at(i).getFromKey()).getPositionVertex()[0];
            startPoint.y = graphSaved.getVertexList()->at(
                    graphSaved.getEdgeList()->at(i).getFromKey()).getPositionVertex()[1];
            startPoint.z = graphSaved.getVertexList()->at(
                    graphSaved.getEdgeList()->at(i).getFromKey()).getPositionVertex()[2];

            endPoint.x = graphSaved.getVertexList()->at(
                    graphSaved.getEdgeList()->at(i).getToKey()).getPositionVertex()[0];
            endPoint.y = graphSaved.getVertexList()->at(
                    graphSaved.getEdgeList()->at(i).getToKey()).getPositionVertex()[1];
            endPoint.z = graphSaved.getVertexList()->at(
                    graphSaved.getEdgeList()->at(i).getToKey()).getPositionVertex()[2];
            currentMarker.points.push_back(startPoint);
            currentMarker.points.push_back(endPoint);
            currentMarker.type = 0;
            currentMarker.id = j;
            j++;
            markerArrowsArray.markers.push_back(currentMarker);
        }
    }
    publisherLoopClosures.publish(markerArrowsArray);

}

//start means later in the graph . example: vertex 800 start ;  vertex 340 end
double slamToolsRos::createVoxelOfGraphStartEndPoint(double voxelData[], int indexStart, int indexEnd,
                                                     int numberOfPoints, graphSlamSaveStructure &usedGraph,
                                                     double ignoreDistanceToRobot, double dimensionOfVoxelData,
                                                     Eigen::Matrix4d transformationInTheEndOfCalculation) {
    int *voxelDataIndex;
    voxelDataIndex = (int *) malloc(sizeof(int) * numberOfPoints * numberOfPoints);
    //set zero voxel and index
    for (int i = 0; i < numberOfPoints * numberOfPoints; i++) {
        voxelDataIndex[i] = 0;
        voxelData[i] = 0;
    }


    int i = 0;
    do {
        //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.


        //get position of current intensityRay
        Eigen::Matrix4d transformationOfIntensityRay =
                usedGraph.getVertexList()->at(indexStart).getTransformation().inverse() *
                usedGraph.getVertexList()->at(indexStart - i).getTransformation();

        //positionOfIntensity has to be rotated by    graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
        Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                         usedGraph.getVertexList()->at(
                                                                                                                 indexStart -
                                                                                                                 i).getIntensities().angle);

        int ignoreDistance = (int) (ignoreDistanceToRobot /
                                    (usedGraph.getVertexList()->at(indexStart - i).getIntensities().range /
                                     ((double) usedGraph.getVertexList()->at(
                                             indexStart - i).getIntensities().intensities.size())));


        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            double distanceOfIntensity =
                    j / ((double) usedGraph.getVertexList()->at(
                            indexStart - i).getIntensities().intensities.size()) *
                    ((double) usedGraph.getVertexList()->at(indexStart - i).getIntensities().range);

            int incrementOfScan = usedGraph.getVertexList()->at(indexStart - i).getIntensities().increment;
            for (int l = -incrementOfScan - 5; l <= incrementOfScan + 5; l++) {
                Eigen::Vector4d positionOfIntensity(
                        distanceOfIntensity,
                        0,
                        0,
                        1);
                double rotationOfPoint = l / 400.0;
                Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                            rotationOfPoint);
                positionOfIntensity = rotationForBetterView * positionOfIntensity;

                positionOfIntensity = transformationInTheEndOfCalculation * transformationOfIntensityRay *
                                      rotationOfSonarAngleMatrix * positionOfIntensity;
                //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
                int indexX =
                        (int) (positionOfIntensity.x() / (dimensionOfVoxelData / 2) * numberOfPoints /
                               2) +
                        numberOfPoints / 2;
                int indexY =
                        (int) (positionOfIntensity.y() / (dimensionOfVoxelData / 2) * numberOfPoints /
                               2) +
                        numberOfPoints / 2;


                if (indexX < numberOfPoints && indexY < numberOfPoints && indexY >= 0 &&
                    indexX >= 0) {
                    //if index fits inside of our data, add that data. Else Ignore
                    voxelDataIndex[indexY + numberOfPoints * indexX] =
                            voxelDataIndex[indexY + numberOfPoints * indexX] + 1;
                    voxelData[indexY + numberOfPoints * indexX] =
                            voxelData[indexY + numberOfPoints * indexX] +
                            usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j];
                }
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             indexStart - i != indexEnd);

    double maximumOfVoxelData = 0;
    for (i = 0; i < numberOfPoints * numberOfPoints; i++) {
        if (voxelDataIndex[i] > 0) {
            voxelData[i] = voxelData[i] / voxelDataIndex[i];
            if (maximumOfVoxelData < voxelData[i]) {
                maximumOfVoxelData = voxelData[i];
            }

        }
    }

    free(voxelDataIndex);
    return maximumOfVoxelData;
}


edge
slamToolsRos::calculatePoseDiffByTimeDepOnEKF(double startTimetoAdd, double endTimeToAdd,
                                              std::deque<double> &timeVector,
                                              std::deque<double> &xPositionVector, std::deque<double> &yPositionVector,
                                              std::deque<double> &zPositionVector,
                                              std::deque<Eigen::Quaterniond> &rotationVector,
                                              std::mutex &stateEstimationMutex) {
    //this is done to make sure 1 more message is coming from the EKF directly

    std::lock_guard<std::mutex> lock(stateEstimationMutex);
    //find index of start and end
    int indexOfStart = 0;
    while (timeVector[indexOfStart] < startTimetoAdd && timeVector.size() > indexOfStart) {
        indexOfStart++;
    }
    if (indexOfStart > 0) {
        indexOfStart--;
    }

    int indexOfEnd = 0;
    while (timeVector[indexOfEnd] < endTimeToAdd && timeVector.size() > indexOfEnd) {
        indexOfEnd++;
    }
    indexOfEnd--;

    Eigen::Matrix4d transformationTMP = Eigen::Matrix4d::Identity();

    if (indexOfStart > 0) {
        double interpolationFactor = 1.0 - ((timeVector[indexOfStart + 1] - startTimetoAdd) /
                                            (timeVector[indexOfStart + 1] - timeVector[indexOfStart]));

        Eigen::Matrix4d transformationOfEKFStart = Eigen::Matrix4d::Identity();
        transformationOfEKFStart.block<3, 3>(0, 0) = rotationVector[indexOfStart - 1].toRotationMatrix();
        transformationOfEKFStart(0, 3) = xPositionVector[indexOfStart];
        transformationOfEKFStart(1, 3) = yPositionVector[indexOfStart];
        transformationOfEKFStart(2, 3) = zPositionVector[indexOfStart];

        Eigen::Matrix4d transformationOfEKFEnd = Eigen::Matrix4d::Identity();
        transformationOfEKFEnd.block<3, 3>(0, 0) = rotationVector[indexOfStart].toRotationMatrix();
        transformationOfEKFEnd(0, 3) = xPositionVector[indexOfStart + 1];
        transformationOfEKFEnd(1, 3) = yPositionVector[indexOfStart + 1];
        transformationOfEKFEnd(2, 3) = zPositionVector[indexOfStart + 1];

        transformationTMP = transformationTMP *
                            generalHelpfulTools::interpolationTwo4DTransformations(transformationOfEKFStart,
                                                                                   transformationOfEKFEnd,
                                                                                   interpolationFactor).inverse() *
                            transformationOfEKFEnd;
    }


    int i = indexOfStart + 1;
    while (i < indexOfEnd) {
        Eigen::Matrix4d transformationOfEKFEnd = Eigen::Matrix4d::Identity();
        transformationOfEKFEnd.block<3, 3>(0, 0) = rotationVector[i].toRotationMatrix();
        transformationOfEKFEnd(0, 3) = xPositionVector[i];
        transformationOfEKFEnd(1, 3) = yPositionVector[i];
        transformationOfEKFEnd(2, 3) = zPositionVector[i];

        Eigen::Matrix4d transformationOfEKFStart = Eigen::Matrix4d::Identity();
        transformationOfEKFStart.block<3, 3>(0, 0) = rotationVector[i - 1].toRotationMatrix();
        transformationOfEKFStart(0, 3) = xPositionVector[i - 1];
        transformationOfEKFStart(1, 3) = yPositionVector[i - 1];
        transformationOfEKFStart(2, 3) = zPositionVector[i - 1];

        transformationTMP = transformationTMP * (transformationOfEKFStart.inverse() * transformationOfEKFEnd);
        i++;
    }

    if (indexOfEnd > 0) {


        double interpolationFactor = ((endTimeToAdd - timeVector[indexOfEnd]) /
                                      (timeVector[indexOfEnd + 1] - timeVector[indexOfEnd]));

        Eigen::Matrix4d transformationOfEKFStart = Eigen::Matrix4d::Identity();
        transformationOfEKFStart.block<3, 3>(0, 0) = rotationVector[indexOfEnd].toRotationMatrix();
        transformationOfEKFStart(0, 3) = xPositionVector[indexOfEnd];
        transformationOfEKFStart(1, 3) = yPositionVector[indexOfEnd];
        transformationOfEKFStart(2, 3) = zPositionVector[indexOfEnd];

        Eigen::Matrix4d transformationOfEKFEnd = Eigen::Matrix4d::Identity();
        transformationOfEKFEnd.block<3, 3>(0, 0) = rotationVector[indexOfEnd + 1].toRotationMatrix();
        transformationOfEKFEnd(0, 3) = xPositionVector[indexOfEnd + 1];
        transformationOfEKFEnd(1, 3) = yPositionVector[indexOfEnd + 1];
        transformationOfEKFEnd(2, 3) = zPositionVector[indexOfEnd + 1];

        transformationTMP = transformationTMP * transformationOfEKFStart.inverse() *
                            generalHelpfulTools::interpolationTwo4DTransformations(transformationOfEKFStart,
                                                                                   transformationOfEKFEnd,
                                                                                   interpolationFactor);
    }
    Eigen::Vector3d tmpPosition = transformationTMP.block<3, 1>(0, 3);

    //set z pos diff to zero
    tmpPosition[2] = 0;
    Eigen::Quaterniond tmpRot(transformationTMP.block<3, 3>(0, 0));
    Eigen::Vector3d rpyTMP = generalHelpfulTools::getRollPitchYaw(tmpRot);
    //set rp on zero only yaw is interesting
    tmpRot = generalHelpfulTools::getQuaternionFromRPY(0, 0, rpyTMP[2]);//was +0.00001
    Eigen::Matrix3d positionCovariance = Eigen::Matrix3d::Zero();
    edge tmpEdge(0, 0, tmpPosition, tmpRot, positionCovariance, 3,
                 INTEGRATED_POSE, 0);

    return tmpEdge;
}

double
slamToolsRos::getDatasetFromGraphForMap(std::vector<intensityValues> &dataSet, graphSlamSaveStructure &graphSaved,
                                        std::mutex &graphSlamMutex) {
    std::lock_guard<std::mutex> lock(graphSlamMutex);
//        std::vector<dataPointStruct> dataSet;

//        std::random_device rd;  // Will be used to obtain a seed for the random number engine
//        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
//        std::uniform_real_distribution<> dis(0.0, 1.0);
    double maxOverall = 0;
    for (int i = 0; i < graphSaved.getVertexList()->size(); i++) {
        intensityValues tmpInt;
        tmpInt.transformation = graphSaved.getVertexList()->at(i).getTransformation();
        tmpInt.intensity = graphSaved.getVertexList()->at(i).getIntensities();


        double it = *max_element(std::begin(tmpInt.intensity.intensities),
                                 std::end(tmpInt.intensity.intensities)); // C++11
        if (it > maxOverall) {
            maxOverall = it;
        }
        dataSet.push_back(tmpInt);
    }


    return maxOverall;
}

int slamToolsRos::getLastIntensityKeyframe(graphSlamSaveStructure &graphSaved) {//the absolut last entry is ignored
    int lastKeyframeIndex = graphSaved.getVertexList()->size() - 2;//ignore the last index
    //find last keyframe
    while (graphSaved.getVertexList()->at(lastKeyframeIndex).getTypeOfVertex() !=
           INTENSITY_SAVED_AND_KEYFRAME &&
           graphSaved.getVertexList()->at(lastKeyframeIndex).getTypeOfVertex() != FIRST_ENTRY) {
        lastKeyframeIndex--;
    }
    return lastKeyframeIndex;
}

double slamToolsRos::angleBetweenLastKeyframeAndNow(graphSlamSaveStructure &graphSaved) {
    double resultingAngleSonar = 0;
    double resultingAngleMovement = 0;
    int lastKeyframeIndex = getLastIntensityKeyframe(graphSaved);

    for (int i = lastKeyframeIndex; i < graphSaved.getVertexList()->size() - 1; i++) {
        Eigen::Quaterniond currentRot =
                graphSaved.getVertexList()->at(i).getRotationVertex().inverse() *
                graphSaved.getVertexList()->at(i + 1).getRotationVertex();


        Eigen::Vector3d rpy = generalHelpfulTools::getRollPitchYaw(currentRot);
        resultingAngleMovement += rpy(2);
        resultingAngleSonar += generalHelpfulTools::angleDiff(
                graphSaved.getVertexList()->at(i + 1).getIntensities().angle,
                graphSaved.getVertexList()->at(i).getIntensities().angle);
    }

    return resultingAngleMovement + resultingAngleSonar;

}

void slamToolsRos::clearSavingsOfPoses(double upToTime, std::deque<double> &timeVector,
                                       std::deque<double> &xPositionVector, std::deque<double> &yPositionVector,
                                       std::deque<double> &zPositionVector,
                                       std::deque<Eigen::Quaterniond> &rotationVector,
                                       std::mutex &stateEstimationMutex) {
    std::lock_guard<std::mutex> lock(stateEstimationMutex);
    while (timeVector[0] < upToTime) {
        rotationVector.pop_front();
        timeVector.pop_front();
        xPositionVector.pop_front();
        yPositionVector.pop_front();
        zPositionVector.pop_front();
    }
}

bool slamToolsRos::calculateStartAndEndIndexForVoxelCreation(int indexMiddle, int &indexStart, int &indexEnd,
                                                             graphSlamSaveStructure &usedGraph) {
    indexStart = indexMiddle + 1;
    indexEnd = indexMiddle - 1;
    double currentAngleOfScan;
    do {
        indexStart += 1;
        indexEnd -= 1;
        bool indexStartBool = false;
        bool indexEndBool = false;
        if (indexEnd <= 0) {
            indexEnd = 0;
            indexEndBool = true;
        }
        if (indexStart >= usedGraph.getVertexList()->size()) {
            indexStart = usedGraph.getVertexList()->size() - 1;
            indexStartBool = true;
        }
        if (indexEndBool && indexStartBool) {
            return false;
        }
        double resultingAngleSonar = 0;
        double resultingAngleMovement = 0;

        for (int i = indexEnd; i < indexStart; i++) {
            Eigen::Quaterniond currentRot =
                    usedGraph.getVertexList()->at(i).getRotationVertex().inverse() *
                    usedGraph.getVertexList()->at(i + 1).getRotationVertex();


            Eigen::Vector3d rpy = generalHelpfulTools::getRollPitchYaw(currentRot);
            resultingAngleMovement += rpy(2);
            resultingAngleSonar += generalHelpfulTools::angleDiff(
                    usedGraph.getVertexList()->at(i + 1).getIntensities().angle,
                    usedGraph.getVertexList()->at(i).getIntensities().angle);
        }
        currentAngleOfScan = resultingAngleMovement + resultingAngleSonar;
    } while (abs(currentAngleOfScan) < 2 * M_PI);


    return true;
}

bool slamToolsRos::calculateEndIndexForVoxelCreationByStartIndex(int indexStart, int &indexEnd,
                                                                 graphSlamSaveStructure &usedGraph) {
    //index start is stiff, and index end is searched
    indexEnd = indexStart - 1;
    double currentAngleOfScan;
    do {
        indexEnd -= 1;
        if (indexEnd <= 0) {
            indexEnd = 0;
            return false;
        }
        double resultingAngleSonar = 0;
        double resultingAngleMovement = 0;

        for (int i = indexEnd; i < indexStart; i++) {
            Eigen::Quaterniond currentRot =
                    usedGraph.getVertexList()->at(i).getRotationVertex().inverse() *
                    usedGraph.getVertexList()->at(i + 1).getRotationVertex();


            Eigen::Vector3d rpy = generalHelpfulTools::getRollPitchYaw(currentRot);
            resultingAngleMovement += rpy(2);
            resultingAngleSonar += generalHelpfulTools::angleDiff(
                    usedGraph.getVertexList()->at(i + 1).getIntensities().angle,
                    usedGraph.getVertexList()->at(i).getIntensities().angle);
        }
        currentAngleOfScan = resultingAngleMovement + resultingAngleSonar;
    } while (abs(currentAngleOfScan) < 2 * M_PI);


    return true;
}


Eigen::Matrix4d slamToolsRos::registrationOfTwoVoxelsFast(double voxelData1Input[],
                                                          double voxelData2Input[],
                                                          Eigen::Matrix4d initialGuess,
                                                          Eigen::Matrix3d &covarianceMatrix,
                                                          bool useInitialAngle,
                                                          bool useInitialTranslation,
                                                          double cellSize,
                                                          bool useGauss,
                                                          scanRegistrationClass &scanRegistrationObject,
                                                          bool debug, double potentialNecessaryForPeak,
                                                          bool multipleRadii,
                                                          bool useClahe,
                                                          bool useHamming) {


    Eigen::Matrix4d returnMatrix = scanRegistrationObject.registrationOfTwoVoxelsSOFFTFast(voxelData1Input,
                                                                                           voxelData2Input,
                                                                                           initialGuess,
                                                                                           covarianceMatrix,
                                                                                           useInitialAngle,
                                                                                           useInitialTranslation,
                                                                                           cellSize, useGauss, debug);


    return returnMatrix;
}


bool
slamToolsRos::loopDetectionByClosestPath(graphSlamSaveStructure &graphSaved,
                                         scanRegistrationClass &scanRegistrationObject,
                                         int dimensionOfVoxelData,
                                         double ignoreDistanceToRobot, double distanceOfVoxelDataLengthSI,
                                         bool debugRegistration, bool useInitialTranslation,
                                         double potentialNecessaryForPeak, double maxLoopClosure) {


    Eigen::Vector3d estimatedPosLastPoint = graphSaved.getVertexList()->back().getPositionVertex();
    // we dont look for loop closures in the first 250 vertexes, and the last 500
    int ignoreStartLoopClosure = 250; // 450
    int ignoreEndLoopClosure = 500;
    if (graphSaved.getVertexList()->size() < ignoreEndLoopClosure) {
        return false;
    }
    std::vector<int> potentialLoopClosureVector;
    int potentialLoopClosure = ignoreStartLoopClosure;
    for (int s = ignoreStartLoopClosure; s < graphSaved.getVertexList()->size() - ignoreEndLoopClosure; s++) {

        double d1 = sqrt(
                pow((estimatedPosLastPoint.x() - graphSaved.getVertexList()->at(s).getPositionVertex().x()), 2) +
                pow((estimatedPosLastPoint.y() - graphSaved.getVertexList()->at(s).getPositionVertex().y()), 2));
        double d2 = sqrt(
                pow((estimatedPosLastPoint.x() -
                     graphSaved.getVertexList()->at(potentialLoopClosure).getPositionVertex().x()), 2) +
                pow((estimatedPosLastPoint.y() -
                     graphSaved.getVertexList()->at(potentialLoopClosure).getPositionVertex().y()), 2));
        if (d2 > d1 && maxLoopClosure > d1) {
            potentialLoopClosure = s;
        }
    }


    potentialLoopClosureVector.push_back(potentialLoopClosure);

    double d2 = sqrt(
            pow((estimatedPosLastPoint.x() -
                 graphSaved.getVertexList()->at(potentialLoopClosureVector[0]).getPositionVertex().x()), 2) +
            pow((estimatedPosLastPoint.y() -
                 graphSaved.getVertexList()->at(potentialLoopClosureVector[0]).getPositionVertex().y()), 2));
    if (maxLoopClosure < d2 || potentialLoopClosure == ignoreStartLoopClosure) {
        return false;
    }

    int loopclosureNumber = 0;
    bool foundLoopClosure = false;
    for (const auto &potentialKey: potentialLoopClosureVector) {
        double fitnessScore = 1;

        //create voxel
        double *voxelData1;
        double *voxelData2;
        voxelData1 = (double *) malloc(sizeof(double) * dimensionOfVoxelData * dimensionOfVoxelData);
        voxelData2 = (double *) malloc(sizeof(double) * dimensionOfVoxelData * dimensionOfVoxelData);
        int indexStart1, indexEnd1, indexStart2, indexEnd2;

        if (!slamToolsRos::calculateStartAndEndIndexForVoxelCreation(potentialKey, indexStart2, indexEnd2,
                                                                     graphSaved)) {
            return false;
        }

        indexStart1 = graphSaved.getVertexList()->back().getKey();
        slamToolsRos::calculateEndIndexForVoxelCreationByStartIndex(indexStart1,
                                                                    indexEnd1, graphSaved);

        double maximumVoxel1 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData1,
                                                                             indexStart1, indexEnd1,
                                                                             dimensionOfVoxelData,
                                                                             graphSaved,
                                                                             ignoreDistanceToRobot,
                                                                             distanceOfVoxelDataLengthSI,
                                                                             Eigen::Matrix4d::Identity());

        double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData2,
                                                                             indexStart2, indexEnd2,
                                                                             dimensionOfVoxelData,
                                                                             graphSaved,
                                                                             ignoreDistanceToRobot,
                                                                             distanceOfVoxelDataLengthSI,
                                                                             Eigen::Matrix4d::Identity());


        Eigen::Matrix4d initialGuessTransformation =
                (graphSaved.getVertexList()->at(indexStart1).getTransformation().inverse() *
                 graphSaved.getVertexList()->at(indexStart2).getTransformation()).inverse();
        Eigen::Matrix3d covarianceEstimation = Eigen::Matrix3d::Zero();
        Eigen::Matrix4d currentTransformation = slamToolsRos::registrationOfTwoVoxelsFast(voxelData1,
                                                                                          voxelData2,
                                                                                          initialGuessTransformation,
                                                                                          covarianceEstimation,
                                                                                          true, true,
                                                                                          (double) distanceOfVoxelDataLengthSI /
                                                                                          (double) dimensionOfVoxelData,
                                                                                          false, scanRegistrationObject,
                                                                                          debugRegistration,
                                                                                          potentialNecessaryForPeak);


//        slamToolsRos::saveResultingRegistrationTMPCOPY(indexStart1,
//                                                       indexEnd1, indexStart2, indexEnd2, graphSaved,
//                                                       dimensionOfVoxelData,
//                                                       ignoreDistanceToRobot, distanceOfVoxelDataLengthSI,
//                                                       debugRegistration, currentTransformation,
//                                                       initialGuessTransformation);

        //inverse the transformation because we want the robot transformation, not the scan transformation
        Eigen::Matrix4d transformationEstimationRobot1_2 = currentTransformation.inverse();
        Eigen::Vector3d currentPosDiff;
        Eigen::Quaterniond currentRotDiff(transformationEstimationRobot1_2.block<3, 3>(0, 0));
        currentPosDiff.x() = transformationEstimationRobot1_2(0, 3);
        currentPosDiff.y() = transformationEstimationRobot1_2(1, 3);
        currentPosDiff.z() = 0;
        graphSaved.addEdge(graphSaved.getVertexList()->back().getKey(), indexStart2, currentPosDiff,
                           currentRotDiff, covarianceEstimation, LOOP_CLOSURE);
        foundLoopClosure = true;
        loopclosureNumber++;
        if (loopclosureNumber > 1) { break; }// break if multiple loop closures are found

    }
    if (foundLoopClosure) {
        return true;
    }
    return false;
}




