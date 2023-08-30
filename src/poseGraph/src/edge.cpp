
//
// Created by tim on 23.02.21.
//

#include "edge.h"

int edge::getFromKey() const {
    return this->fromKey;
}

void edge::setFromKey(int &fromVertexInput) {
    this->fromKey = fromVertexInput;
}

int edge::getToKey() const {
    return this->toKey;
}

void edge::setToKey(int &toVertexInput) {
    this->toKey = toVertexInput;
}

Eigen::Vector3d edge::getCovariancePosition() const {
    return covariancePosition;
}

void edge::setEdge(edge &edgeToCopy){
    this->fromKey = edgeToCopy.getFromKey();
    this->toKey = edgeToCopy.getToKey();
    this->covariancePosition = edgeToCopy.getCovariancePosition();
    this->covarianceQuaternion = edgeToCopy.getCovarianceQuaternion();
    this->positionDifference = edgeToCopy.getPositionDifference();
    this->rotationDifference = edgeToCopy.getRotationDifference();
    this->rotationDifference.normalize();
}

void edge::setCovariancePosition(Eigen::Vector3d &covariancePositionInput) {
    this->covariancePosition = covariancePositionInput;
}

double edge::getCovarianceQuaternion() const {
    return covarianceQuaternion;
}

void edge::setCovarianceQuaternion(double &covarianceQuaternionInput) {
    this->covarianceQuaternion = covarianceQuaternionInput;
}

Eigen::Vector3d edge::getPositionDifference() const {
    return positionDifference;
}

void edge::setPositionDifference(const Eigen::Vector3d &positionDifferenceInput) {
    this->positionDifference = positionDifferenceInput;
}

Eigen::Quaterniond edge::getRotationDifference() const {
    return rotationDifference;
}

void edge::setRotationDifference(const Eigen::Quaterniond &rotationDifferenceInput) {
    this->rotationDifference = rotationDifferenceInput;
    this->rotationDifference.normalize();
}

int edge::getTypeOfEdge() const {
    return typeOfEdge;
}

void edge::setTypeOfEdge(int &typeOfEdgeInput) {// 0=pointCloud    %%%%%%%%%   1 = integratedPosDiff
    this->typeOfEdge = typeOfEdgeInput;
}



Eigen::Matrix4d edge::getTransformation() const{
    Eigen::Matrix4d transformation;

    transformation << 1, 0, 0, this->positionDifference.x(),
            0, 1, 0, this->positionDifference.y(),
            0, 0, 1, this->positionDifference.z(),
            0, 0, 0, 1;//transformation missing currently
    Eigen::Matrix3d m(this->rotationDifference.toRotationMatrix());
    transformation.block<3, 3>(0, 0) = m;
    return transformation;
}

int edge::getKeyOfEdgeInGraph() const{
    return(this->keyOfEdgeInGraph);
}

void edge::setKeyOfEdgeInGraph(int &inputKey){
    this->keyOfEdgeInGraph = inputKey;
}