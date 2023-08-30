//
// Created by tim on 23.02.21.
//
#include "edge.h"
#include "vertex.h"
#include <fstream>
#include<Eigen/SparseCholesky>
//#include "json.h"
#include <chrono>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2/LinearMath/Matrix3x3.h>
//#include <tf2/utils.h>
#include "generalHelpfulTools.h"
//gtsam includes
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>


#ifndef SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H
#define SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H


class graphSlamSaveStructure {
public:
    graphSlamSaveStructure(int degreeOfFreedom,int typeOfGraphSlam){
        if (degreeOfFreedom == 3) {
            this->degreeOfFreedom = degreeOfFreedom;
            this->typeOfGraphSlam =typeOfGraphSlam;
        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }

        //adds the Prior. Makes sure that the initial position is at 0 0 0 and stays there.
        auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.01));
        this->graph.addPrior(0, gtsam::Pose2(0, 0, 0), priorNoise);
        
//        this->deadReckoningNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.02, 0.02, 0.005));
        this->deadReckoningNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.03, 0.03, 0.01));
        this->loopClosureNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1, 1, 0.05));

        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        parameters.print();
        isam = new gtsam::ISAM2(parameters);


    }

    void addEdge(int fromKey, int toKey, Eigen::Vector3d positionDifference,
                 Eigen::Quaterniond rotationDifference, Eigen::Matrix3d covarianceMatrix, int typeOfEdge);

    void addVertex(int key, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
                   const Eigen::Matrix3d &covarianceMatrix,
                   intensityMeasurement intensityInput, double timeStamp, int typeOfVertex);



    void printCurrentState();

    void printCurrentStateGeneralInformation();

    std::vector<vertex> *getVertexList();

    std::vector<edge> *getEdgeList();

    void isam2OptimizeGraph(bool verbose, int numberOfUpdates = 10);

    void classicalOptimizeGraph(bool verbose);

    void saveGraphJson(std::string nameSavingFile);

    void addRandomNoiseToGraph(double stdDiviationGauss, double percentageOfRandomNoise);

    void setPoseDifferenceEdge(int numberOfEdge, Eigen::Matrix4d poseDiff);

    void print();

private:





    int degreeOfFreedom;//3 for [x y alpha] or 6 for [x y z alpha beta gamma]
    int typeOfGraphSlam;
    std::vector<edge> edgeList;
    std::vector<vertex> vertexList;
    gtsam::NonlinearFactorGraph graph;
    boost::shared_ptr<gtsam::noiseModel::Diagonal> deadReckoningNoiseModel, loopClosureNoiseModel;
    gtsam::Values currentEstimate;
    gtsam::ISAM2* isam;




};


#endif //SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H
