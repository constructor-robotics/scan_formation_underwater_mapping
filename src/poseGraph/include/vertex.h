//
// Created by tim on 23.02.21.
//

#ifndef SIMULATION_BLUEROV_VETREX_H
#define SIMULATION_BLUEROV_VETREX_H
//#include <vector>
#include "eigen3/Eigen/Geometry"
//#include <Eigen3/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include "generalStructs.h"

class vertex {

public:
    //no point cloud or intensities
    vertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
           int degreeOfFreedom, const Eigen::Matrix3d &covariance,
           double timeStamp, int typeOfVertex) {
        if (degreeOfFreedom == 3) {
            this->keyNumber = vertexNumber;
            this->positionVertex = positionVertex;
            this->rotationVertex = rotationVertex;
            this->rotationVertex.normalize();
            this->covariance = covariance;
            this->typeOfVertex = typeOfVertex;
            this->timeStamp = timeStamp;
            this->voxelData = NULL;
        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
    }

    // point cloud and no intensities
    vertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
           int degreeOfFreedom, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudRaw1,
           const Eigen::Matrix3d &covariance, double timeStamp, int typeOfVertex) {
        if (degreeOfFreedom == 3) {
            this->keyNumber = vertexNumber;
            this->positionVertex = positionVertex;
            this->rotationVertex = rotationVertex;
            this->rotationVertex.normalize();
            this->covariance = covariance;
            setPointCloudRawPTRCP(pointCloudRaw1);
            this->typeOfVertex = typeOfVertex;
            this->timeStamp = timeStamp;
            this->groundTruthTransformation = Eigen::Matrix4d::Zero();
            this->voxelData = NULL;
        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
    }

    // only intensities
    vertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
           int degreeOfFreedom, intensityMeasurement &intensities,
           const Eigen::Matrix3d &covariance, double timeStamp, int typeOfVertex) {
        if (degreeOfFreedom == 3) {
            this->keyNumber = vertexNumber;
            this->positionVertex = positionVertex;
            this->rotationVertex = rotationVertex;
            this->rotationVertex.normalize();
            this->covariance = covariance;
            this->intensities = intensities;
            this->typeOfVertex = typeOfVertex;
            this->timeStamp = timeStamp;
        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
    }


    [[nodiscard]] int getKey() const;

    void setKey(int &vertexNumberInput);

    [[nodiscard]] Eigen::Vector3d getPositionVertex() const;

    void setPositionVertex(const Eigen::Vector3d &positionVertexInput);

    [[nodiscard]] Eigen::Quaterniond getRotationVertex() const;

    void setRotationVertex(const Eigen::Quaterniond &rotationVertexInput);

    [[nodiscard]] pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudCorrected() const;

    void setPointCloudCorrected(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    [[nodiscard]] pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudRaw() const;

    void setPointCloudRawPTRCP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    const Eigen::Matrix3d getCovarianceMatrix() const;

    void setCovarianceMatrix(Eigen::Matrix3d covariancePositionInput);


    Eigen::Matrix4d getTransformation();

    [[nodiscard]] int getTypeOfVertex() const;

    void setTypeOfVertex(int typeOfVertexInput);

    [[nodiscard]] double getTimeStamp() const;

    void setTimeStamp(double timeStampInput);

    [[nodiscard]] intensityMeasurement getIntensities() const;

    void setIntensities(const intensityMeasurement &intensitiesInput);

    [[nodiscard]] Eigen::Matrix4d getGroundTruthTransformation() const;

    void setGroundTruthTransformation(Eigen::Matrix4d inputMatrix);

    void setIntensityScan(double *inputVoxelData, int sizeVoxelData);

    void getIntensityScan(double *outputVoxelData, int sizeVoxelData);

private:
    int keyNumber;
    Eigen::Vector3d positionVertex;// position w.r.t. Initiial Starting Position
    Eigen::Quaterniond rotationVertex;// rotation w.r.t. Initial Starting Rotation
    Eigen::Matrix3d covariance;
    intensityMeasurement intensities;
    Eigen::Matrix4d groundTruthTransformation;
    double *voxelData;
    int typeOfVertex;
    double timeStamp;
};


#endif //SIMULATION_BLUEROV_VETREX_H
