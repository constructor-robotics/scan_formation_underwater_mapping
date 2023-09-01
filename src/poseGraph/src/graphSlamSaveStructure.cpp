//
// Created by tim on 23.02.21.
//

#include "graphSlamSaveStructure.h"
#include "random"

void
graphSlamSaveStructure::addEdge(int fromKey, int toKey, Eigen::Vector3d positionDifference,
                                Eigen::Quaterniond rotationDifference, Eigen::Matrix3d covarianceMatrix, int typeOfEdge) {


    edge edgeToAdd(fromKey, toKey, positionDifference, rotationDifference, covarianceMatrix,
                   this->degreeOfFreedom, typeOfEdge,this->graph.size());

    auto model = gtsam::noiseModel::Gaussian::Covariance(covarianceMatrix);
    // add a factor between the keys
    if (abs(toKey - fromKey) > 2) {
        this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(fromKey, toKey, gtsam::Pose2(
                                                                                edgeToAdd.getPositionDifference().x(), edgeToAdd.getPositionDifference().y(),
                                                                                generalHelpfulTools::getRollPitchYaw(edgeToAdd.getRotationDifference())[2]),
                                                                        model);//@TODO different Noise Model

    } else {
        this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(fromKey, toKey, gtsam::Pose2(edgeToAdd.getPositionDifference().x(), edgeToAdd.getPositionDifference().y(),
                                                                                                     generalHelpfulTools::getRollPitchYaw(edgeToAdd.getRotationDifference())[2]),
                                                                        model);

    }
    // add edge to edge list
    this->edgeList.push_back(edgeToAdd);

}

void graphSlamSaveStructure::addVertex(int key, const Eigen::Vector3d &positionVertex,
                                       const Eigen::Quaterniond &rotationVertex,
                                       const Eigen::Matrix3d &covarianceMatrix,
                                       intensityMeasurement intensityInput, double timeStamp,
                                       int typeOfVertex) {

    vertex vertexToAdd(key, positionVertex, rotationVertex, this->degreeOfFreedom,
                       intensityInput, covarianceMatrix, timeStamp, typeOfVertex);
    this->vertexList.push_back(vertexToAdd);
    //ADD BETTER INITIAL STATE
    double tmpYaw = generalHelpfulTools::getRollPitchYaw(rotationVertex)[2];
    this->currentEstimate.insert(key,gtsam::Pose2(positionVertex[0],positionVertex[1],tmpYaw));
}


void graphSlamSaveStructure::isam2OptimizeGraph(bool verbose, int numberOfUpdates) {




    this->isam->update(this->graph,this->currentEstimate);


    for(int i = 0 ; i<numberOfUpdates ; i++){
        this->isam->update();
    }

    this->currentEstimate = this->isam->calculateEstimate();


    // sort in the covariances and new positions
    for(int i  = 1 ; i<this->vertexList.size() ; i++){
        gtsam::Pose2 iterativePose = this->currentEstimate.at(this->vertexList[i].getKey()).cast<gtsam::Pose2>();
        this->vertexList.at(i).setPositionVertex(Eigen::Vector3d(iterativePose.x(),iterativePose.y(),0));
        this->vertexList.at(i).setRotationVertex(generalHelpfulTools::getQuaternionFromRPY(0,0,iterativePose.theta()));
//        std::cout << "covariance:\n" << marginals.marginalCovariance(this->vertexList.at(i).getKey()) << std::endl;
//        std::cout << "Pose :\n" << iterativePose << std::endl;
        this->vertexList.at(i).setCovarianceMatrix(this->isam->marginalCovariance(this->vertexList.at(i).getKey()));
    }

    this->graph.resize(0);
    this->currentEstimate.clear();
}



void graphSlamSaveStructure::classicalOptimizeGraph(bool verbose) {


    gtsam::LevenbergMarquardtParams params;
//    params.setVerbosity("ERROR");
    params.setOrderingType("METIS");
    params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");




    gtsam::LevenbergMarquardtOptimizer optimizer(this->graph, this->currentEstimate,params);

    // ... and optimize
    this->currentEstimate = optimizer.optimize();



//    this->currentEstimate.print("Final Result:\n");
    gtsam::Marginals marginals(this->graph, this->currentEstimate);


    for(int i  = 1 ; i<this->vertexList.size() ; i++){
        gtsam::Pose2 iterativePose = this->currentEstimate.at(this->vertexList[i].getKey()).cast<gtsam::Pose2>();
        this->vertexList.at(i).setPositionVertex(Eigen::Vector3d(iterativePose.x(),iterativePose.y(),0));
        this->vertexList.at(i).setRotationVertex(generalHelpfulTools::getQuaternionFromRPY(0,0,iterativePose.theta()));

        this->vertexList.at(i).setCovarianceMatrix(this->isam->marginalCovariance(this->vertexList.at(i).getKey()));

    }

}

void graphSlamSaveStructure::printCurrentState() {
    std::cout << "current State:" << std::endl;
    for (int i = 0; i < this->vertexList.size(); i++) {
        std::cout << "State:" << i << std::endl;
        std::cout << "Pose:" << std::endl;
        this->currentEstimate.at(this->vertexList[i].getKey()).print();
    }
}

void graphSlamSaveStructure::printCurrentStateGeneralInformation() {
    std::cout << "Number of Vertex:" << this->vertexList.size() << std::endl;
    for (int i = 0; i < this->vertexList.size(); i++) {
        std::cout << "Vertex Nr. " << i << " Pos:\n"
                  << this->vertexList[i].getPositionVertex() << std::endl;
    }
}

std::vector<vertex> *graphSlamSaveStructure::getVertexList() {
    return &this->vertexList;
}

std::vector<edge> *graphSlamSaveStructure::getEdgeList() {
    return &this->edgeList;
}


void graphSlamSaveStructure::addRandomNoiseToGraph(double stdDiviationGauss, double percentageOfRandomNoise) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    std::normal_distribution<> disNormal(0.0, stdDiviationGauss);


    double maximumIntensity = 0;
    for (int i = 0; i < this->vertexList.size(); i++) {
        intensityMeasurement tmpIntensity = this->vertexList[i].getIntensities();
        //find max intensity;
        for (int j = 0; j < tmpIntensity.intensities.size(); j++) {
            if (tmpIntensity.intensities[j] > maximumIntensity) {
                maximumIntensity = tmpIntensity.intensities[j];
            }
        }
    }

    for (int i = 0; i < this->vertexList.size(); i++) {
        intensityMeasurement tmpIntensity = this->vertexList[i].getIntensities();
        //find max intensity;
        for (int j = 0; j < tmpIntensity.intensities.size(); j++) {
            if (dis(gen) < percentageOfRandomNoise) {
                tmpIntensity.intensities[j] = dis(gen) * maximumIntensity;
//                std::cout << tmpIntensity.intensities[j] << std::endl;
            }


            tmpIntensity.intensities[j] = tmpIntensity.intensities[j] + disNormal(gen) * maximumIntensity;
            if (tmpIntensity.intensities[j] < 0) {
                tmpIntensity.intensities[j] = 0;
            }

        }
        this->vertexList[i].setIntensities(tmpIntensity);
    }




//    this->vertexList[i].setIntensities(tmpIntensity);

}

void graphSlamSaveStructure::setPoseDifferenceEdge(int numberOfEdge, Eigen::Matrix4d poseDiff){
    this->edgeList.at(numberOfEdge).setPositionDifference(poseDiff.block<3, 1>(0, 3));
    this->edgeList.at(numberOfEdge).setRotationDifference(Eigen::Quaterniond(poseDiff.block<3, 3>(0, 0)));
    int fromKey = this->edgeList.at(numberOfEdge).getFromKey();
    int toKey = this->edgeList.at(numberOfEdge).getToKey();
    int keyFactorInGraph = this->edgeList.at(numberOfEdge).getKeyOfEdgeInGraph();


    this->graph.remove(this->edgeList.at(numberOfEdge).getKeyOfEdgeInGraph());
    int tmpInt = this->graph.size();
    this->edgeList.at(numberOfEdge).setKeyOfEdgeInGraph(tmpInt);

    this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(fromKey, toKey, gtsam::Pose2(
                                                                            this->edgeList.at(numberOfEdge).getPositionDifference().x(), this->edgeList.at(numberOfEdge).getPositionDifference().y(),
                                                                                          generalHelpfulTools::getRollPitchYaw(this->edgeList.at(numberOfEdge).getRotationDifference())[2]),
                                                                                  this->loopClosureNoiseModel);//@TODO different Noise Model

}

void graphSlamSaveStructure::print(){
    this->graph.print();
}
