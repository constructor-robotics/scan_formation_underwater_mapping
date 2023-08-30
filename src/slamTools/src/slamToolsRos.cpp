//
// Created by tim on 26.03.21.
//

#include "slamToolsRos.h"
//#include "generalHelpfulTools.h"

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
        //pcl::PointCloud<pcl::PointXYZ> currentScanTransformed;
        //vertexElement.getTransformation();
        //completeTransformation = vertexElement.getTransformation();
//        if(vertexElement.getTypeOfVertex()==graphSlamSaveStructure::POINT_CLOUD_SAVED){
//            pcl::io::savePCDFileASCII("/home/jurobotics/DataForTests/savingRandomPCL/firstPCL.pcd",*vertexElement.getPointCloudCorrected());
//            pcl::io::savePCDFileASCII("/home/jurobotics/DataForTests/savingRandomPCL/secondPCL.pcd",currentScanTransformed);
//        }

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
//        std::cout << i << std::endl;
//        std::cout << currentCovariance << std::endl;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(currentCovariance.block<2, 2>(0, 0));
        Eigen::Vector2d e1 = eig.eigenvectors().col(0);
        Eigen::Vector2d e2 = eig.eigenvectors().col(1);
        double l1 = eig.eigenvalues().x();
        double l2 = eig.eigenvalues().y();
//        std::cout << eigenVector << std::endl;
//        std::cout << eigenValues << std::endl;
        double phi;
        if (l2 >= l1) {
            phi = atan2(e2.y(), e2.x());
        } else {
            phi = atan2(e1.y(), e1.x());
        }
//        double phi = atan2(e2.y(), e2.x());
//        std::cout << currentCovariance << std::endl;
        visualization_msgs::Marker currentMarker;
        currentMarker.pose.position.x = vertexElement.getPositionVertex().x();
        currentMarker.pose.position.y = vertexElement.getPositionVertex().y();
        currentMarker.pose.position.z = vertexElement.getPositionVertex().z();
        Eigen::Vector3d rpyTMP = generalHelpfulTools::getRollPitchYaw(vertexElement.getRotationVertex());
        double inputAngle = generalHelpfulTools::normalizeAngle(phi + rpyTMP[2]);
        Eigen::Quaterniond tmpQuad = generalHelpfulTools::getQuaternionFromRPY(0, 0, inputAngle);
//        Eigen::Quaterniond tmpQuad = generalHelpfulTools::getQuaternionFromRPY(0, 0, inputAngle);
//        tmpQuad = vertexElement.getRotationVertex()*tmpQuad;



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
            //currentMarker.pose.position.x = pos.pose.position.x;
            //currentMarker.pose.position.y = pos.pose.position.y;
            //currentMarker.pose.position.z = pos.pose.position.z;
            //currentMarker.pose.orientation.w = 1;
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

std::vector<measurement> slamToolsRos::parseCSVFile(std::istream &stream) {
    std::vector<measurement> returnVector;

    std::string firstLine;
    std::getline(stream, firstLine);

    //std::stringstream          lineStream(line);
    std::string cell;


    for (std::string line; std::getline(stream, line);) {
        std::stringstream lineStream(line);
        std::vector<std::string> result;
        while (std::getline(lineStream, cell, ',')) {
            result.push_back(cell);
            //std::cout << cell << std::endl;
        }
        measurement tmpMeas{};
        tmpMeas.keyframe = std::stoi(result[0]);
        tmpMeas.x = std::stof(result[1]);
        tmpMeas.y = std::stof(result[2]);
        tmpMeas.z = std::stof(result[3]);
        tmpMeas.timeStamp = std::stof(result[4]);
        returnVector.push_back(tmpMeas);
    }
    return returnVector;
}

std::vector<std::vector<measurement>> slamToolsRos::sortToKeyframe(std::vector<measurement> &input) {
    int currentKeyframe = input[0].keyframe;
    std::vector<std::vector<measurement>> output;
    std::vector<measurement> tmp1;
    output.push_back(tmp1);
    for (auto currentMeasurement: input) {
        if (currentMeasurement.keyframe != currentKeyframe) {//new keyframe reached
            std::vector<measurement> tmp;
            currentKeyframe = currentMeasurement.keyframe;
            output.push_back(tmp);
            output[currentKeyframe].push_back(currentMeasurement);
        } else {
            output[currentKeyframe].push_back(currentMeasurement);
        }
    }
    return output;
}


std::vector<double> slamToolsRos::linspace(double start_in, double end_in, int num_in) {
    if (num_in < 0) {
        std::cout << "number of linspace negative" << std::endl;
        exit(-1);
    }
    std::vector<double> linspaced;

    double start = start_in;
    double end = end_in;
    auto num = (double) num_in;

    if (num == 0) { return linspaced; }
    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);//stepSize

    for (int i = 0; i < num - 1; ++i) {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
    // are exactly the same as the input
    return linspaced;
}

void slamToolsRos::calculatePositionOverTime(std::deque<ImuData> &angularVelocityList,
                                             std::deque<DvlData> &bodyVelocityList,
                                             std::vector<edge> &posOverTimeEdge,
                                             double lastScanTimeStamp,
                                             double currentScanTimeStamp,
                                             double noiseAddedStdDiv, int numberOfEdges) {//last then current
    posOverTimeEdge.clear();
    std::vector<double> timeSteps = slamToolsRos::linspace(lastScanTimeStamp, currentScanTimeStamp,
                                                           numberOfEdges);// could be changed this is the number of pos+1 between the scans(10th is the scan itself)
    std::vector<double> angularX;
    std::vector<double> angularY;
    std::vector<double> angularZ;
    for (int i = 1;
         i < timeSteps.size(); i++) {//calculate angular between lastScanTimeStamp and currentScanTimeStamp
        std::vector<ImuData> measurementsOfInterest;
        for (int j = 0; j < angularVelocityList.size(); j++) {
            if (timeSteps[i - 1] <= angularVelocityList[j].timeStamp &&
                timeSteps[i] > angularVelocityList[j].timeStamp) {
                measurementsOfInterest.push_back(angularVelocityList[j]);
            }
        }

        double integratorX = 0;
        double integratorY = 0;
        double integratorZ = 0;
        if (measurementsOfInterest.size() > 1) {
            for (int j = 0; j < measurementsOfInterest.size(); j++) {
                if (j == measurementsOfInterest.size() - 1) {
                    integratorX +=
                            measurementsOfInterest[j].wx * (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                    integratorY +=
                            measurementsOfInterest[j].wy * (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                    integratorZ +=
                            measurementsOfInterest[j].wz * (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                } else {
                    if (j == 0) {
                        integratorX +=
                                measurementsOfInterest[j].wx * (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                        integratorY +=
                                measurementsOfInterest[j].wy * (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                        integratorZ +=
                                measurementsOfInterest[j].wz * (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                    } else {
                        integratorX += (measurementsOfInterest[j].wx + measurementsOfInterest[j - 1].wx) / 2 *
                                       (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                        integratorY += (measurementsOfInterest[j].wy + measurementsOfInterest[j - 1].wy) / 2 *
                                       (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                        integratorZ += (measurementsOfInterest[j].wz + measurementsOfInterest[j - 1].wz) / 2 *
                                       (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                    }
                }
            }
        } else {//only one or zero velocity measurement exists
            integratorX = measurementsOfInterest[0].wx * (timeSteps[i] - timeSteps[i - 1]);
            integratorY = measurementsOfInterest[0].wy * (timeSteps[i] - timeSteps[i - 1]);
            integratorZ = measurementsOfInterest[0].wz * (timeSteps[i] - timeSteps[i - 1]);
        }
        angularX.push_back(integratorX);
        angularY.push_back(integratorY);
        angularZ.push_back(integratorZ);
    }

    std::default_random_engine generator;
    std::normal_distribution<double> dist(0, noiseAddedStdDiv);
    std::vector<double> linearX;
    std::vector<double> linearY;
    std::vector<double> linearZ;
    for (int i = 1;
         i < timeSteps.size(); i++) {//calculate that between lastScanTimeStamp and currentScanTimeStamp
        std::vector<DvlData> measurementsOfInterest;
        for (int j = 0; j < bodyVelocityList.size(); j++) {
            if (timeSteps[i - 1] <= bodyVelocityList[j].timeStamp &&
                timeSteps[i] > bodyVelocityList[j].timeStamp) {
                measurementsOfInterest.push_back(bodyVelocityList[j]);
            }
        }

        double integratorX = 0;
        double integratorY = 0;
        double integratorZ = 0;

        if (measurementsOfInterest.size() > 1) {
            for (int j = 0; j < measurementsOfInterest.size(); j++) {
                if (j == measurementsOfInterest.size() - 1) {
                    integratorX += (dist(generator) + measurementsOfInterest[j].vx) *
                                   (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                    integratorY += (dist(generator) + measurementsOfInterest[j].vy) *
                                   (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                    integratorZ += (dist(generator) + measurementsOfInterest[j].vz) *
                                   (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                } else {
                    if (j == 0) {
                        integratorX +=
                                (dist(generator) + measurementsOfInterest[j].vx) *
                                (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                        integratorY +=
                                (dist(generator) + measurementsOfInterest[j].vy) *
                                (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                        integratorZ +=
                                (dist(generator) + measurementsOfInterest[j].vz) *
                                (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                    } else {
                        integratorX +=
                                (dist(generator) +
                                 (measurementsOfInterest[j].vx + measurementsOfInterest[j - 1].vx) / 2) *
                                (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                        integratorY +=
                                (dist(generator) +
                                 (measurementsOfInterest[j].vy + measurementsOfInterest[j - 1].vy) / 2) *
                                (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                        integratorZ +=
                                (dist(generator) +
                                 (measurementsOfInterest[j].vz + measurementsOfInterest[j - 1].vz) / 2) *
                                (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                    }
                }
            }
        } else {//only one velocity measurement exists
            integratorX = measurementsOfInterest[0].vx * (timeSteps[i] - timeSteps[i - 1]);
            integratorY = measurementsOfInterest[0].vy * (timeSteps[i] - timeSteps[i - 1]);
            integratorZ = measurementsOfInterest[0].vz * (timeSteps[i] - timeSteps[i - 1]);
        }
        linearX.push_back(integratorX);
        linearY.push_back(integratorY);
        linearZ.push_back(integratorZ);
    }



    //std::vector<vertex> &posOverTimeVertex,
    //std::vector<edge> &posOverTimeEdge,

    for (int i = 0; i < timeSteps.size() - 1; i++) {
        Eigen::Vector3d posDiff(linearX[i], linearY[i], 0);//linear Z missing
        Eigen::Quaterniond rotDiff = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())//should be added somewhen(6DOF)
                                     * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())//should be added somewhen(6DOF)
                                     * Eigen::AngleAxisd(angularZ[i],
                                                         Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
        edge currentEdge(0, 0, posDiff, rotDiff, covarianceMatrix, 3,
                         INTEGRATED_POSE, 0);
        //currentEdge.setTimeStamp(timeSteps[i + 1]);
        posOverTimeEdge.push_back(currentEdge);
    }

}

double slamToolsRos::createVoxelOfGraph(double voxelData[], int indexStart,
                                        Eigen::Matrix4d transformationInTheEndOfCalculation,
                                        int numberOfPoints, graphSlamSaveStructure &usedGraph,
                                        double ignoreDistanceToRobot, double dimensionOfVoxelData) {
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
                    //                    std::cout << indexX << " " << indexY << std::endl;
                    //if index fits inside of our data, add that data. Else Ignore
                    voxelDataIndex[indexY + numberOfPoints * indexX] =
                            voxelDataIndex[indexY + numberOfPoints * indexX] + 1;
                    //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                    voxelData[indexY + numberOfPoints * indexX] =
                            voxelData[indexY + numberOfPoints * indexX] +
                            usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j];
                    //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                    //                    std::cout << "random: " << std::endl;
                }
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);

// std::cout << "number of intensity values used: " << i << std::endl;

    double maximumOfVoxelData = 0;
    for (i = 0; i < numberOfPoints * numberOfPoints; i++) {
        if (voxelDataIndex[i] > 0) {
            voxelData[i] = voxelData[i] / voxelDataIndex[i];
            if (maximumOfVoxelData < voxelData[i]) {
                maximumOfVoxelData = voxelData[i];
            }
            //std::cout << voxelData[i] << std::endl;

        }
    }// @TODO calculate the maximum and normalize "somehow"




    free(voxelDataIndex);
    return maximumOfVoxelData;
}

//start means later in the graph . example: 800 start ;  340 end
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
                    //                    std::cout << indexX << " " << indexY << std::endl;
                    //if index fits inside of our data, add that data. Else Ignore
                    voxelDataIndex[indexY + numberOfPoints * indexX] =
                            voxelDataIndex[indexY + numberOfPoints * indexX] + 1;
                    //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                    voxelData[indexY + numberOfPoints * indexX] =
                            voxelData[indexY + numberOfPoints * indexX] +
                            usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j];
                    //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                    //                    std::cout << "random: " << std::endl;
                }
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             indexStart - i != indexEnd);

// std::cout << "number of intensity values used: " << i << std::endl;

    double maximumOfVoxelData = 0;
    for (i = 0; i < numberOfPoints * numberOfPoints; i++) {
        if (voxelDataIndex[i] > 0) {
            voxelData[i] = voxelData[i] / voxelDataIndex[i];
            if (maximumOfVoxelData < voxelData[i]) {
                maximumOfVoxelData = voxelData[i];
            }
            //std::cout << voxelData[i] << std::endl;

        }
    }// @TODO calculate the maximum and normalize "somehow"




    free(voxelDataIndex);
    return maximumOfVoxelData;
}

pcl::PointCloud<pcl::PointXYZ> slamToolsRos::createPCLFromGraphOneValue(int indexStart,
                                                                        Eigen::Matrix4d transformationInTheEndOfCalculation,
                                                                        graphSlamSaveStructure &usedGraph,
                                                                        double ignoreDistanceToRobo,
                                                                        double thresholdFactorPoint) {
    pcl::PointCloud<pcl::PointXYZ> scan;
    //create array with all intencities.
    // Calculate maximum of intensities.
    // only use maximum of 10% of max value as points
    int i = 0;
    double maximumIntensity = 0;

    int ignoreDistance = (int) (ignoreDistanceToRobo /
                                (usedGraph.getVertexList()->at(indexStart - i).getIntensities().range /
                                 ((double) usedGraph.getVertexList()->at(
                                         indexStart - i).getIntensities().intensities.size())));

    do {
        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                maximumIntensity) {
                maximumIntensity = usedGraph.getVertexList()->at(
                        indexStart - i).getIntensities().intensities[j];
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);

    double thresholdIntensityScan = maximumIntensity * thresholdFactorPoint;//maximum intensity of 0.9



    i = 0;
    do {
        //find max Position
        int maxPosition = ignoreDistance;
        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[maxPosition]) {
                maxPosition = j;
            }
        }
        if (maxPosition > ignoreDistance &&
            usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[maxPosition] >
            thresholdIntensityScan) {
            Eigen::Matrix4d transformationOfIntensityRay =
                    usedGraph.getVertexList()->at(indexStart).getTransformation().inverse() *
                    usedGraph.getVertexList()->at(indexStart - i).getTransformation();

            //positionOfIntensity has to be rotated by    graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             usedGraph.getVertexList()->at(
                                                                                                                     indexStart -
                                                                                                                     i).getIntensities().angle);

            double distanceOfIntensity =
                    maxPosition / ((double) usedGraph.getVertexList()->at(
                            indexStart - i).getIntensities().intensities.size()) *
                    ((double) usedGraph.getVertexList()->at(indexStart - i).getIntensities().range);
            Eigen::Vector4d positionOfIntensity(
                    distanceOfIntensity,
                    0,
                    0,
                    1);

            positionOfIntensity = transformationInTheEndOfCalculation * transformationOfIntensityRay *
                                  rotationOfSonarAngleMatrix * positionOfIntensity;
            //create point for PCL
            pcl::PointXYZ tmpPoint((float) positionOfIntensity[0],
                                   (float) positionOfIntensity[1],
                                   (float) positionOfIntensity[2]);
            scan.push_back(tmpPoint);
        }


        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);
    return scan;
}


pcl::PointCloud<pcl::PointXYZ> slamToolsRos::createPCLFromGraphOnlyThreshold(int indexStart,
                                                                             Eigen::Matrix4d transformationInTheEndOfCalculation,
                                                                             graphSlamSaveStructure &usedGraph,
                                                                             double ignoreDistanceToRobo,
                                                                             double thresholdFactorPoint) {
    pcl::PointCloud<pcl::PointXYZ> scan;
    //create array with all intencities.
    // Calculate maximum of intensities.


    double maximumIntensity = 0;
    int i = 0;

    int ignoreDistance = (int) (ignoreDistanceToRobo /
                                (usedGraph.getVertexList()->at(indexStart - i).getIntensities().range /
                                 ((double) usedGraph.getVertexList()->at(
                                         indexStart - i).getIntensities().intensities.size())));


    do {
        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                maximumIntensity) {
                maximumIntensity = usedGraph.getVertexList()->at(
                        indexStart - i).getIntensities().intensities[j];
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);

    double thresholdIntensityScan = maximumIntensity * thresholdFactorPoint;//maximum intensity of 0.9



    i = 0;
    do {
        Eigen::Matrix4d transformationOfIntensityRay =
                usedGraph.getVertexList()->at(indexStart).getTransformation().inverse() *
                usedGraph.getVertexList()->at(indexStart - i).getTransformation();

        //positionOfIntensity has to be rotated by    graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
        Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                         usedGraph.getVertexList()->at(
                                                                                                                 indexStart -
                                                                                                                 i).getIntensities().angle);
        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                thresholdIntensityScan) {
                double distanceOfIntensity =
                        j / ((double) usedGraph.getVertexList()->at(
                                indexStart - i).getIntensities().intensities.size()) *
                        ((double) usedGraph.getVertexList()->at(indexStart - i).getIntensities().range);
                Eigen::Vector4d positionOfIntensity(
                        distanceOfIntensity,
                        0,
                        0,
                        1);
                positionOfIntensity = transformationInTheEndOfCalculation * transformationOfIntensityRay *
                                      rotationOfSonarAngleMatrix * positionOfIntensity;
                //create point for PCL
                pcl::PointXYZ tmpPoint((float) positionOfIntensity[0],
                                       (float) positionOfIntensity[1],
                                       (float) positionOfIntensity[2]);
                scan.push_back(tmpPoint);
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);
    return scan;
}


bool slamToolsRos::getNodes(ros::V_string &nodes) {
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();

    if (!ros::master::execute("getSystemState", args, result, payload, true)) {
        return false;
    }

    ros::S_string node_set;
    for (int i = 0; i < payload.size(); ++i) {
        for (int j = 0; j < payload[i].size(); ++j) {
            XmlRpc::XmlRpcValue val = payload[i][j][1];
            for (int k = 0; k < val.size(); ++k) {
                std::string name = payload[i][j][1][k];
                node_set.insert(name);
            }
        }
    }

    nodes.insert(nodes.end(), node_set.begin(), node_set.end());

    return true;
}

edge
slamToolsRos::calculatePoseDiffByTimeDepOnEKF(double startTimetoAdd, double endTimeToAdd,
                                              std::deque<double> &timeVector,
                                              std::deque<double> &xPositionVector, std::deque<double> &yPositionVector,
                                              std::deque<double> &zPositionVector,
                                              std::deque<Eigen::Quaterniond> &rotationVector,
                                              std::mutex &stateEstimationMutex) {
    //this is done to make sure 1 more message is coming from the EKF directly
    //ros::Duration(0.001).sleep();

//    while (timeVector.empty()) {
//        ros::Duration(0.002).sleep();
//    }
//
//
//    while (endTimeToAdd > timeVector[timeVector.size() - 1]) {
//        ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf");
//        ros::Duration(0.001).sleep();
//    }

    //@TEST
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
//        std::cout << endTimeToAdd << std::endl;
//        std::cout << timeVector[indexOfEnd] << std::endl;
//        std::cout << timeVector[indexOfEnd + 1] << std::endl;


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
    //std::cout << diffMatrix << std::endl;
    Eigen::Vector3d tmpPosition = transformationTMP.block<3, 1>(0, 3);
    //WRONG HERE
//    double tmpX = tmpPosition[0];
//    double tmpY = tmpPosition[1];
//    tmpPosition[0] = tmpY;
//    tmpPosition[1] = -tmpX;
    //set z pos diff to zero
    tmpPosition[2] = 0;
    Eigen::Quaterniond tmpRot(transformationTMP.block<3, 3>(0, 0));
    Eigen::Vector3d rpyTMP = generalHelpfulTools::getRollPitchYaw(tmpRot);
    //set rp on zero only yaw interesting
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

void slamToolsRos::updateRegistration(int numberOfEdge, graphSlamSaveStructure &usedGraph, int dimensionOfVoxelData,
                                      double ignoreDistanceToRobot, double distanceOfVoxelDataLengthSI,
                                      scanRegistrationClass &scanRegistrationObject,
                                      bool debugRegistration) {

    int tmpFromKey = usedGraph.getEdgeList()->at(numberOfEdge).getFromKey();
    int tmpToKey = usedGraph.getEdgeList()->at(numberOfEdge).getToKey();


    //match these voxels together
    Eigen::Matrix4d initialGuessTransformation =
            usedGraph.getVertexList()->at(tmpFromKey).getTransformation().inverse() *
            usedGraph.getVertexList()->at(tmpToKey).getTransformation();

    double initialGuessAngle = std::atan2(initialGuessTransformation(1, 0),
                                          initialGuessTransformation(0, 0));
    double fitnessScoreX, fitnessScoreY;

    double *voxelData1;
    double *voxelData2;
    voxelData1 = (double *) malloc(sizeof(double) * dimensionOfVoxelData * dimensionOfVoxelData);
    voxelData2 = (double *) malloc(sizeof(double) * dimensionOfVoxelData * dimensionOfVoxelData);

    double maximumVoxel1 = slamToolsRos::createVoxelOfGraph(voxelData1,
                                                            tmpFromKey,
                                                            Eigen::Matrix4d::Identity(),
                                                            dimensionOfVoxelData, usedGraph,
                                                            ignoreDistanceToRobot,
                                                            distanceOfVoxelDataLengthSI);//get voxel


    double maximumVoxel2 = slamToolsRos::createVoxelOfGraph(voxelData2,
                                                            tmpToKey,
                                                            Eigen::Matrix4d::Identity(),
                                                            dimensionOfVoxelData, usedGraph,
                                                            ignoreDistanceToRobot,
                                                            distanceOfVoxelDataLengthSI);//get voxel

    // transform from 1 to 2
//         currentTransformation = scanRegistrationObject.registrationOfTwoVoxelsSOFFTFast(
//                voxelData1, voxelData2,
//                 initialGuessTransformation, true,
//                true, (double) distanceOfVoxelDataLengthSI /
//                      (double) dimensionOfVoxelData, true, debugRegistration);
    Eigen::Matrix3d covarianceEstimation = Eigen::Matrix3d::Zero();
    Eigen::Matrix4d currentTransformation = slamToolsRos::registrationOfTwoVoxels(voxelData1, voxelData2,
                                                                                  initialGuessTransformation,
                                                                                  covarianceEstimation, true,
                                                                                  true,
                                                                                  (double) distanceOfVoxelDataLengthSI /
                                                                                  (double) dimensionOfVoxelData, true,
                                                                                  scanRegistrationObject,
                                                                                  debugRegistration);
    slamToolsRos::saveResultingRegistration(voxelData1, voxelData2, usedGraph, dimensionOfVoxelData,
                                            ignoreDistanceToRobot, distanceOfVoxelDataLengthSI, debugRegistration,
                                            currentTransformation);

    double differenceAngleBeforeAfter = generalHelpfulTools::angleDiff(
            std::atan2(currentTransformation(1, 0), currentTransformation(0, 0)),
            initialGuessAngle);


    std::cout << "FitnessScore X: " << fitnessScoreX << " FitnessScore Y: " << fitnessScoreY << std::endl;

    std::cout << "currentTransformation:" << std::endl;
    std::cout << currentTransformation << std::endl;

    std::cout << "initial Guess Transformation:" << std::endl;
    std::cout << initialGuessTransformation << std::endl;


    std::cout << "Initial guess angle: "
              << initialGuessAngle * 180 / M_PI
              << std::endl;
    std::cout << "Registration angle: "
              << std::atan2(currentTransformation(1, 0), currentTransformation(0, 0)) * 180 / M_PI
              << std::endl;
    std::cout << "difference of angle after Registration: " << differenceAngleBeforeAfter * 180 / M_PI
              << std::endl;
    //only if angle diff is smaller than 10 degreece its ok

//        usedGraph.getEdgeList()->at(numberOfEdge).setPositionDifference( currentTransformation.block<3, 1>(0, 3));
//        usedGraph.getEdgeList()->at(numberOfEdge).setRotationDifference(Eigen::Quaterniond( currentTransformation.block<3, 3>(0, 0)));
    usedGraph.setPoseDifferenceEdge(numberOfEdge, currentTransformation);

    free(voxelData1);
    free(voxelData2);

}

//creates transformation from voxel 1 to be close to voxel 2
Eigen::Matrix4d slamToolsRos::registrationOfTwoVoxels(double voxelData1Input[],
                                                      double voxelData2Input[],
                                                      Eigen::Matrix4d initialGuess,
                                                      Eigen::Matrix3d &covarianceMatrix,
                                                      bool useInitialAngle,
                                                      bool useInitialTranslation,
                                                      double cellSize,
                                                      bool useGauss,
                                                      scanRegistrationClass &scanRegistrationObject,
                                                      bool debug, double potentialNecessaryForPeak, bool multipleRadii,
                                                      bool useClahe,
                                                      bool useHamming) {


    std::vector<transformationPeak> listOfTransformations = scanRegistrationObject.registrationOfTwoVoxelsSOFFTAllSoluations(
            voxelData1Input, voxelData2Input, cellSize, useGauss, debug, potentialNecessaryForPeak,multipleRadii,useClahe,useHamming);
    double initialAngle = std::atan2(initialGuess(1, 0),
                                     initialGuess(0, 0));
    std::vector<transformationPeak> potentialTranslationsWithAngle;
    if (listOfTransformations.size() > 1) {
        potentialTranslationsWithAngle.push_back(listOfTransformations[0]);

        if (useInitialAngle) {
            for (int i = 1; i < listOfTransformations.size(); i++) {

                if (abs(generalHelpfulTools::angleDiff(potentialTranslationsWithAngle[0].potentialRotation.angle,
                                                       initialAngle)) >
                    abs(generalHelpfulTools::angleDiff(listOfTransformations[i].potentialRotation.angle,
                                                       initialAngle))) {
                    potentialTranslationsWithAngle[0] = listOfTransformations[i];
                }
            }
        } else {
            potentialTranslationsWithAngle = potentialTranslationsWithAngle;
        }
    } else {
        potentialTranslationsWithAngle.push_back(listOfTransformations[0]);
    }

    Eigen::Vector2d posVector = initialGuess.block<2, 1>(0, 3);
    double maximumValue = 0;
    int indexMaximumAngle = 0;
    int indexMaximumTranslation = 0;
    if (useInitialTranslation) {
        for (int i = 0; i < potentialTranslationsWithAngle.size(); i++) {
            for (int j = 0; j < potentialTranslationsWithAngle[i].potentialTranslations.size(); j++) {
                if ((potentialTranslationsWithAngle[i].potentialTranslations[j].translationSI - posVector).norm() <
                    (potentialTranslationsWithAngle[indexMaximumAngle].potentialTranslations[indexMaximumTranslation].translationSI -
                     posVector).norm()) {
                    indexMaximumAngle = i;
                    indexMaximumTranslation = j;
                }
            }
        }

    } else {
        for (int i = 0; i < potentialTranslationsWithAngle.size(); i++) {
            for (int j = 0; j < potentialTranslationsWithAngle[i].potentialTranslations.size(); j++) {
                if (potentialTranslationsWithAngle[i].potentialTranslations[j].peakHeight >
                    potentialTranslationsWithAngle[indexMaximumAngle].potentialTranslations[indexMaximumTranslation].peakHeight) {
                    indexMaximumAngle = i;
                    indexMaximumTranslation = j;
                }
            }
        }
    }

    //create matrix4d
    Eigen::Matrix4d returnTransformation = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                               potentialTranslationsWithAngle[indexMaximumAngle].potentialRotation.angle);

    returnTransformation.block<2, 1>(0,
                                     3) = potentialTranslationsWithAngle[indexMaximumAngle].potentialTranslations[indexMaximumTranslation].translationSI;
    covarianceMatrix.block<2, 2>(0,
                                 0) = potentialTranslationsWithAngle[indexMaximumAngle].potentialTranslations[indexMaximumTranslation].covariance;
    covarianceMatrix(2, 2) = potentialTranslationsWithAngle[indexMaximumAngle].potentialRotation.covariance;
    return returnTransformation;
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
                                                      bool debug, double potentialNecessaryForPeak, bool multipleRadii,
                                                      bool useClahe,
                                                      bool useHamming){




    Eigen::Matrix4d returnMatrix = scanRegistrationObject.registrationOfTwoVoxelsSOFFTFast(voxelData1Input,
            voxelData2Input,
            initialGuess,covarianceMatrix,useInitialAngle,useInitialTranslation,cellSize,useGauss,debug);


    return returnMatrix;
}
void slamToolsRos::saveResultingRegistration(double *voxelData1, double *voxelData2,
                                             graphSlamSaveStructure &usedGraph, int dimensionOfVoxelData,
                                             double ignoreDistanceToRobot, double distanceOfVoxelDataLengthSI,
                                             bool debugRegistration, Eigen::Matrix4d currentTransformation) {


    if (debugRegistration) {
//        voxelData1 = (double *) malloc(sizeof(double) * dimensionOfVoxelData * dimensionOfVoxelData);
//        voxelData2 = (double *) malloc(sizeof(double) * dimensionOfVoxelData * dimensionOfVoxelData);


//        double maximumVoxel1 = slamToolsRos::createVoxelOfGraph(voxelData1,
//                                                                indexFirstKeyFrame,
//                                                                currentTransformation,
//                                                                dimensionOfVoxelData, usedGraph,
//                                                                ignoreDistanceToRobot,
//                                                                distanceOfVoxelDataLengthSI);//get voxel
//
//        double maximumVoxel2 = slamToolsRos::createVoxelOfGraph(voxelData2,
//                                                                indexSecondKeyFrame,
//                                                                Eigen::Matrix4d::Identity(),
//                                                                dimensionOfVoxelData, usedGraph,
//                                                                ignoreDistanceToRobot,
//                                                                distanceOfVoxelDataLengthSI);//get voxel
        std::ofstream myFile1, myFile2;
        myFile1.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultVoxel1.csv");
        myFile2.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultVoxel2.csv");
        for (int j = 0; j < dimensionOfVoxelData; j++) {
            for (int i = 0; i < dimensionOfVoxelData; i++) {
                myFile1 << voxelData1[j + dimensionOfVoxelData * i]; // real part
                myFile1 << "\n";
                myFile2 << voxelData2[j + dimensionOfVoxelData * i]; // imaginary part
                myFile2 << "\n";
            }
        }
        myFile1.close();
        myFile2.close();
        free(voxelData1);
        free(voxelData2);
    }
}


bool
slamToolsRos::loopDetectionByClosestPath(graphSlamSaveStructure &graphSaved,
                                         scanRegistrationClass &scanRegistrationObject,
                                         int dimensionOfVoxelData,
                                         double ignoreDistanceToRobot, double distanceOfVoxelDataLengthSI,
                                         bool debugRegistration, bool useInitialTranslation,
                                         double potentialNecessaryForPeak, double maxLoopClosure) {
//    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


    Eigen::Vector3d estimatedPosLastPoint = graphSaved.getVertexList()->back().getPositionVertex();
    int ignoreStartLoopClosure = 250; // 450
    int ignoreEndLoopClosure = 500;
    if (graphSaved.getVertexList()->size() < ignoreEndLoopClosure) {
        return false;
    }
    std::vector<int> potentialLoopClosureVector;
    int potentialLoopClosure = ignoreStartLoopClosure;
    for (int s = ignoreStartLoopClosure; s < graphSaved.getVertexList()->size() - ignoreEndLoopClosure; s++) {
        //dist.row(s) = (graphSaved.getVertexList()[s].getPositionVertex() - estimatedPosLastPoint).norm();
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

//    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//    std::cout << "calculation For Loop Closure Potential: "
//              << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;
    double d2 = sqrt(
            pow((estimatedPosLastPoint.x() -
                 graphSaved.getVertexList()->at(potentialLoopClosureVector[0]).getPositionVertex().x()), 2) +
            pow((estimatedPosLastPoint.y() -
                 graphSaved.getVertexList()->at(potentialLoopClosureVector[0]).getPositionVertex().y()), 2));
    if (maxLoopClosure < d2 || potentialLoopClosure == ignoreStartLoopClosure) {
        return false;
    }


//        std::shuffle(potentialLoopClosureVector.begin(), potentialLoopClosureVector.end(), std::mt19937(std::random_device()()));

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

//        double maximumVoxel1 = slamToolsRos::createVoxelOfGraph(voxelData1,
//                                                                graphSaved.getVertexList()->back().getKey(),
//                                                                Eigen::Matrix4d::Identity(),
//                                                                dimensionOfVoxelData, graphSaved,
//                                                                ignoreDistanceToRobot,
//                                                                distanceOfVoxelDataLengthSI);//get voxel
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

//            std::cout << "Found Loop Closure with fitnessScore: " << fitnessScore << std::endl;
//        std::cout << "Estimated Transformation:" << std::endl;
//        std::cout << currentTransformation.inverse() << std::endl;
////
//        std::cout << "initial Guess Transformation:" << std::endl;
//        std::cout << initialGuessTransformation.inverse() << std::endl;
////
//        std::cout << "covarianceEstimation" << std::endl;
//
//        std::cout << covarianceEstimation << std::endl;
//        std::cout << "Initial guess angle: "
//                  << std::atan2(initialGuessTransformation(1, 0), initialGuessTransformation(0, 0)) *
//                     180 / M_PI
//                  << std::endl;
//        std::cout << "Registration angle: "
//                  <<
//                  std::atan2(currentTransformation(1, 0), currentTransformation(0, 0)) *
//                  180 / M_PI << std::endl;

//            saveResultingRegistration( graphSaved.getVertexList()->back().getKey(), potentialKey);


        slamToolsRos::saveResultingRegistrationTMPCOPY(indexStart1,
                                                       indexEnd1, indexStart2, indexEnd2, graphSaved,
                                                       dimensionOfVoxelData,
                                                       ignoreDistanceToRobot, distanceOfVoxelDataLengthSI,
                                                       debugRegistration, currentTransformation,
                                                       initialGuessTransformation);

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


void slamToolsRos::saveResultingRegistrationTMPCOPY(int indexStart1, int indexEnd1, int indexStart2, int indexEnd2,
                                                    graphSlamSaveStructure &usedGraph, int dimensionOfVoxelData,
                                                    double ignoreDistanceToRobot, double distanceOfVoxelDataLengthSI,
                                                    bool debugRegistration, Eigen::Matrix4d currentTransformation,
                                                    Eigen::Matrix4d initialGuess) {


    if (debugRegistration) {
        double *voxelData1;
        double *voxelData2;
        voxelData1 = (double *) malloc(sizeof(double) * dimensionOfVoxelData * dimensionOfVoxelData);
        voxelData2 = (double *) malloc(sizeof(double) * dimensionOfVoxelData * dimensionOfVoxelData);


        double maximumVoxel1 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData1,
                                                                             indexStart1, indexEnd1,
                                                                             dimensionOfVoxelData,
                                                                             usedGraph,
                                                                             ignoreDistanceToRobot,
                                                                             distanceOfVoxelDataLengthSI,
                                                                             currentTransformation);

        double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData2,
                                                                             indexStart2, indexEnd2,
                                                                             dimensionOfVoxelData,
                                                                             usedGraph,
                                                                             ignoreDistanceToRobot,
                                                                             distanceOfVoxelDataLengthSI,
                                                                             Eigen::Matrix4d::Identity());


        std::ofstream myFile1, myFile2;
        myFile1.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultVoxel1.csv");
        myFile2.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultVoxel2.csv");
        for (int j = 0; j < dimensionOfVoxelData; j++) {
            for (int i = 0; i < dimensionOfVoxelData; i++) {
                myFile1 << voxelData1[j + dimensionOfVoxelData * i]; // real part
                myFile1 << "\n";
                myFile2 << voxelData2[j + dimensionOfVoxelData * i]; // imaginary part
                myFile2 << "\n";
            }
        }
        myFile1.close();
        myFile2.close();
        free(voxelData1);
        free(voxelData2);


        std::ofstream myFile12;
        myFile12.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/initialGuess.csv");
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                myFile12 << initialGuess(i, j) << " ";//number of possible rotations
            }
            myFile12 << "\n";
        }
        myFile12.close();
    }
}

bool slamToolsRos::simpleLoopDetectionByKeyFrames(graphSlamSaveStructure &graphSaved,
                                                  scanRegistrationClass &scanRegistrationObject,
                                                  int dimensionOfVoxelData,
                                                  double ignoreDistanceToRobot, double distanceOfVoxelDataLengthSI,
                                                  double maxLoopClosure,
                                                  bool debugRegistration) {
//    double numberOfLoopClosures = 2.0;// was 2.0
//    double distanceLoopClousureAllowed = 3.0;// was 3.0

    Eigen::Vector3d estimatedPosLastPoint = graphSaved.getVertexList()->back().getPositionVertex();
    int ignoreStartLoopClosure = 450;
    int ignoreEndLoopClosure = 800;
    if (graphSaved.getVertexList()->size() < ignoreEndLoopClosure) {
        return false;
    }
//    std::vector<int> potentialLoopClosureVector;
    int potentialLoopClosure = ignoreStartLoopClosure;
    for (int s = potentialLoopClosure; s < graphSaved.getVertexList()->size() - ignoreEndLoopClosure; s++) {
        //dist.row(s) = (graphSaved.getVertexList()[s].getPositionVertex() - estimatedPosLastPoint).norm();
        double d = sqrt(
                pow((estimatedPosLastPoint.x() - graphSaved.getVertexList()->at(s).getPositionVertex().x()), 2) +
                pow((estimatedPosLastPoint.y() - graphSaved.getVertexList()->at(s).getPositionVertex().y()), 2));
        double r1 = (graphSaved.getVertexList()->at(s).getCovarianceMatrix()(0, 0) +
                     graphSaved.getVertexList()->at(s).getCovarianceMatrix()(1, 1)) / 2;
        double r2 = (graphSaved.getVertexList()->back().getCovarianceMatrix()(0, 0) +
                     graphSaved.getVertexList()->back().getCovarianceMatrix()(1, 1)) / 2;
        if ((d <= r1 - r2 || d <= r2 - r1 || d < r1 + r2 || d == r1 + r2) && d < maxLoopClosure &&
            graphSaved.getVertexList()->at(s).getTypeOfVertex() == INTENSITY_SAVED_AND_KEYFRAME) {
            potentialLoopClosure = graphSaved.getVertexList()->at(s).getKey();
        }
    }
//    potentialLoopClosureVector.push_back(potentialLoopClosure);
    double d2 = sqrt(
            pow((estimatedPosLastPoint.x() -
                 graphSaved.getVertexList()->at(potentialLoopClosure).getPositionVertex().x()), 2) +
            pow((estimatedPosLastPoint.y() -
                 graphSaved.getVertexList()->at(potentialLoopClosure).getPositionVertex().y()), 2));
    if (maxLoopClosure < d2 || potentialLoopClosure == ignoreStartLoopClosure) {
        return false;
    }


//    std::shuffle(potentialLoopClosure.begin(), potentialLoopClosure.end(), std::mt19937(std::random_device()()));

    int loopclosureNumber = 0;
    bool foundLoopClosure = false;
    double fitnessScore = 1;

    //create voxel
    double *voxelData1;
    double *voxelData2;
    voxelData1 = (double *) malloc(sizeof(double) * dimensionOfVoxelData * dimensionOfVoxelData);
    voxelData2 = (double *) malloc(sizeof(double) * dimensionOfVoxelData * dimensionOfVoxelData);
//    int indexStart, indexEnd;

//        if (!slamToolsRos::calculateStartAndEndIndexForVoxelCreation(potentialKey, indexStart, indexEnd,
//                                                                     graphSaved)) {
//            return false;
//        }


    graphSaved.getVertexList()->back().getIntensityScan(voxelData1, dimensionOfVoxelData);
    graphSaved.getVertexList()->at(potentialLoopClosure).getIntensityScan(voxelData2, dimensionOfVoxelData);


    Eigen::Matrix4d initialGuessTransformation =
            (graphSaved.getVertexList()->back().getTransformation().inverse() *
             graphSaved.getVertexList()->at(potentialLoopClosure).getTransformation()).inverse();
    // transform from 1 to 2
//             currentTransformation = tmpregistrationClass.registrationOfTwoVoxelsSOFFTFast(voxelData1,
//                                                                                                voxelData2,
//                                                                                                 initialGuessTransformation,
//                                                                                                true, false,
//                                                                                                (double) distanceOfVoxelDataLengthSI /
//                                                                                                (double) dimensionOfVoxelData,
//                                                                                                true,
//                                                                                                debugRegistration);
    Eigen::Matrix3d covarianceEstimation = Eigen::Matrix3d::Zero();

//    Eigen::Matrix4d currentTransformation = slamToolsRos::registrationOfTwoVoxels(voxelData1,
//                                                                                      voxelData2,
//                                                                                      initialGuessTransformation,
//                                                                                      covarianceEstimation,
//                                                                                      true, true,
//                                                                                      (double) distanceOfVoxelDataLengthSI /
//                                                                                      (double) dimensionOfVoxelData,
//                                                                                      true, scanRegistrationObject,
//                                                                                      debugRegistration);
//    std::cout << currentTransformation << std::endl;
    Eigen::Matrix4d  currentTransformation = slamToolsRos::registrationOfTwoVoxelsFast(voxelData1,
                                                                                  voxelData2,
                                                                                  initialGuessTransformation,
                                                                                  covarianceEstimation,
                                                                                  true, true,
                                                                                  (double) distanceOfVoxelDataLengthSI /
                                                                                  (double) dimensionOfVoxelData,
                                                                                  true, scanRegistrationObject,
                                                                                  debugRegistration);
//    std::cout << currentTransformation << std::endl;

//            std::cout << "Found Loop Closure with fitnessScore: " << fitnessScore << std::endl;

//    std::cout << "covarianceEstimation" << std::endl;
//    std::cout << covarianceEstimation << std::endl;
//
//    std::cout << "Estimated Transformation:" << std::endl;
//    std::cout << currentTransformation << std::endl;
//
//    std::cout << "initial Guess Transformation:" << std::endl;
//    std::cout << initialGuessTransformation << std::endl;
//
//    std::cout << "Initial guess angle: "
//              << std::atan2(initialGuessTransformation(1, 0), initialGuessTransformation(0, 0)) *
//                 180 / M_PI
//              << std::endl;
//    std::cout << "Registration angle: "
//              <<
//              std::atan2(currentTransformation(1, 0), currentTransformation(0, 0)) *
//              180 / M_PI << std::endl;

//            saveResultingRegistration( graphSaved.getVertexList()->back().getKey(), potentialKey);
//        int indexEndTMP;
//        slamToolsRos::calculateEndIndexForVoxelCreationByStartIndex(graphSaved.getVertexList()->back().getKey(),
//                                                                    indexEndTMP, graphSaved);
    slamToolsRos::saveResultingRegistration(voxelData1, voxelData2, graphSaved, dimensionOfVoxelData,
                                            ignoreDistanceToRobot, distanceOfVoxelDataLengthSI, debugRegistration,
                                            currentTransformation);


    //inverse the transformation because we want the robot transformation, not the scan transformation
    Eigen::Matrix4d transformationEstimationRobot1_2 = currentTransformation.inverse();
    Eigen::Vector3d currentPosDiff;
    Eigen::Quaterniond currentRotDiff(transformationEstimationRobot1_2.block<3, 3>(0, 0));
    currentPosDiff.x() = transformationEstimationRobot1_2(0, 3);
    currentPosDiff.y() = transformationEstimationRobot1_2(1, 3);
    currentPosDiff.z() = 0;

    graphSaved.addEdge(graphSaved.getVertexList()->back().getKey(), potentialLoopClosure, currentPosDiff,
                       currentRotDiff, covarianceEstimation, LOOP_CLOSURE);

    if (foundLoopClosure) {
        return true;
    }
    return false;
}


pcl::PointCloud<pcl::PointXYZ>
slamToolsRos::convertVoxelToPointcloud(double voxelData[], double thresholdFactor, double maximumVoxelData, int dimensionVoxel,double dimensionOfVoxelDataForMatching) {

    pcl::PointCloud<pcl::PointXYZ> ourPCL;
    for (int j = 0; j < dimensionVoxel; j++) {
        for (int k = 0; k < dimensionVoxel; k++) {
            if (voxelData[j + dimensionVoxel * k] > maximumVoxelData * thresholdFactor) {
                double xPosPoint = (j - dimensionVoxel / 2.0) * dimensionOfVoxelDataForMatching /
                                   ((double) dimensionVoxel);
                double yPosPoint = (k - dimensionVoxel / 2.0) * dimensionOfVoxelDataForMatching /
                                   ((double) dimensionVoxel);
                //mix X and Y for enu to ned
                ourPCL.push_back(pcl::PointXYZ(yPosPoint, xPosPoint, 0));

            }
        }
    }


    return ourPCL;
}

Eigen::Matrix4d slamToolsRos::registrationOfDesiredMethod(pcl::PointCloud<pcl::PointXYZ> pclNotShifted,
                                             pcl::PointCloud<pcl::PointXYZ> pclShifted,
                                             pcl::PointCloud<pcl::PointXYZ> &final, double voxelData[],
                                             double voxelDataShifted[],
                                             Eigen::Matrix4d initialGuess, double currentCellSize,
                                             int whichMethod, bool useInitialGuess,
                                             scanRegistrationClass &scanRegistrationObject) {

    //1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform,
    //6: Our FMS 2D 7: FMS hamming 8: FMS none
    //9: Feature0 10: Feature1  11: Feature2 12: Feature3 13: Feature4 14: Feature5
    //15: gmmRegistrationD2D 16: gmmRegistrationP2D
    Eigen::Matrix4d returnMatrix;
    Eigen::Matrix3d covarianceEstimation;
    switch (whichMethod) {
        case 6:

            covarianceEstimation = Eigen::Matrix3d::Zero();


            if(useInitialGuess){
                returnMatrix = slamToolsRos::registrationOfTwoVoxelsFast(voxelData, voxelDataShifted,
                                                                     initialGuess,
                                                                     covarianceEstimation, useInitialGuess,
                                                                     useInitialGuess,
                                                                     currentCellSize,
                                                                     false, scanRegistrationObject,
                                                                     false,
                                                                     0.05);
            }else{
                // THRESHOLD_FOR_TRANSLATION_MATCHING 0.05 // standard is 0.1, 0.05 und 0.01  // 0.05 for valentin Oben
                returnMatrix = slamToolsRos::registrationOfTwoVoxels(voxelData, voxelDataShifted,
                                                                     initialGuess,
                                                                     covarianceEstimation, useInitialGuess,
                                                                     useInitialGuess,
                                                                     currentCellSize,
                                                                     false, scanRegistrationObject,
                                                                     false,
                                                                     0.05);
            }

            break;
        case 7:
            covarianceEstimation = Eigen::Matrix3d::Zero();
            // THRESHOLD_FOR_TRANSLATION_MATCHING 0.05 // standard is 0.1, 0.05 und 0.01  // 0.05 for valentin Oben
            returnMatrix = slamToolsRos::registrationOfTwoVoxels(voxelData, voxelDataShifted,
                                                                 initialGuess,
                                                                 covarianceEstimation, useInitialGuess,
                                                                 useInitialGuess,
                                                                 currentCellSize,
                                                                 false, scanRegistrationObject,
                                                                 false,
                                                                 0.05, false, false, true);
            break;
        case 8:
            covarianceEstimation = Eigen::Matrix3d::Zero();
            // THRESHOLD_FOR_TRANSLATION_MATCHING 0.05 // standard is 0.1, 0.05 und 0.01  // 0.05 for valentin Oben
            returnMatrix = slamToolsRos::registrationOfTwoVoxels(voxelData, voxelDataShifted,
                                                                 initialGuess,
                                                                 covarianceEstimation, useInitialGuess,
                                                                 useInitialGuess,
                                                                 currentCellSize,
                                                                 false, scanRegistrationObject,
                                                                 false,
                                                                 0.05, false, false, false);
            break;
    }

    return returnMatrix;

}