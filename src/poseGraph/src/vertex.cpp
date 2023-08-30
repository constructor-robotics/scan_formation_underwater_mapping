//
// Created by tim on 23.02.21.
//

#include "vertex.h"

int vertex::getKey() const {
    return vertex::keyNumber;
}

void vertex::setKey(int &vertexNumberInput) {
    vertex::keyNumber = vertexNumberInput;
}

Eigen::Vector3d vertex::getPositionVertex() const {
    return positionVertex;
}

void vertex::setPositionVertex(const Eigen::Vector3d &positionVertexInput) {
    vertex::positionVertex = positionVertexInput;
}

Eigen::Quaterniond vertex::getRotationVertex() const {
    return rotationVertex;
}

void vertex::setRotationVertex(const Eigen::Quaterniond &rotationVertexInput) {
    this->rotationVertex = rotationVertexInput;
    this->rotationVertex.normalize();
}

const Eigen::Matrix3d vertex::getCovarianceMatrix() const {
    return this->covariance;
}

void vertex::setCovarianceMatrix(Eigen::Matrix3d covariancePositionInput) {
    this->covariance = covariancePositionInput;
}

Eigen::Matrix4d vertex::getTransformation() {
    Eigen::Matrix4d transformation;

    transformation << 1, 0, 0, this->positionVertex.x(),
            0, 1, 0, this->positionVertex.y(),
            0, 0, 1, this->positionVertex.z(),
            0, 0, 0, 1;//transformation missing currently
    Eigen::Matrix3d m(this->rotationVertex.toRotationMatrix());
    transformation.block<3, 3>(0, 0) = m;
    return transformation;
}

int vertex::getTypeOfVertex() const {
    return typeOfVertex;
}

void vertex::setTypeOfVertex(int typeOfVertexInput) {// 0=pointCloud    %%%%%%%%%   1 = integratedPosDiff
    vertex::typeOfVertex = typeOfVertexInput;
}

double vertex::getTimeStamp() const {
    return timeStamp;
}

void vertex::setTimeStamp(double timeStampInput) {
    vertex::timeStamp = timeStampInput;
}

intensityMeasurement vertex::getIntensities() const {
    return intensities;
}

void vertex::setIntensities(const intensityMeasurement &intensitiesInput) {
    vertex::intensities = intensitiesInput;
}


Eigen::Matrix4d vertex::getGroundTruthTransformation() const {
    return this->groundTruthTransformation;
}

void vertex::setGroundTruthTransformation(Eigen::Matrix4d inputMatrix) {

    this->groundTruthTransformation = inputMatrix;
}

void vertex::setIntensityScan(double *inputVoxelData, int sizeVoxelData) {
    if (!this->voxelData) {
        free(this->voxelData);
    }

    this->voxelData = (double *) malloc(sizeof(double) * sizeVoxelData * sizeVoxelData);
    for (int i = 0; i < sizeVoxelData * sizeVoxelData; i++) {
        this->voxelData[i] = inputVoxelData[i];
    }

}

void vertex::getIntensityScan(double *outputVoxelData, int sizeVoxelData) {
    for (int i = 0; i < sizeVoxelData * sizeVoxelData; i++) {
        outputVoxelData[i] = this->voxelData[i];
    }

}



