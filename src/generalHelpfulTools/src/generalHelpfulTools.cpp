//
// Created by jurobotics on 15.09.21.
//

#include "generalHelpfulTools.h"

Eigen::Vector3d generalHelpfulTools::getRollPitchYaw(Eigen::Quaterniond quat) {
    tf2::Quaternion tmp(quat.x(), quat.y(), quat.z(), quat.w());
    tf2::Matrix3x3 m(tmp);
    double r, p, y;
    m.getRPY(r, p, y);
    Eigen::Vector3d returnVector(r, p, y);
    return returnVector;
}

Eigen::Matrix4d generalHelpfulTools::getTransformationMatrixFromRPY(double roll, double pitch, double yaw) {
    Eigen::Quaterniond rotationAsQuaternion = generalHelpfulTools::getQuaternionFromRPY(roll, pitch, yaw);
    Eigen::Matrix4d returnMatrix = Eigen::Matrix4d::Identity();
    returnMatrix.block<3, 3>(0, 0) = rotationAsQuaternion.toRotationMatrix();
    return returnMatrix;
}

Eigen::Quaterniond generalHelpfulTools::getQuaternionFromRPY(double roll, double pitch, double yaw) {
//        tf2::Matrix3x3 m;
//        m.setRPY(roll,pitch,yaw);
//        Eigen::Matrix3d m2;
    tf2::Quaternion qtf2;
    qtf2.setRPY(roll, pitch, yaw);
    Eigen::Quaterniond q;
    q.x() = qtf2.x();
    q.y() = qtf2.y();
    q.z() = qtf2.z();
    q.w() = qtf2.w();

//        q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
//            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
//            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    return q;
}

double generalHelpfulTools::angleDiff(double first, double second) {//first-second
    return atan2(sin(first - second), cos(first - second));
}


Eigen::Matrix4d generalHelpfulTools::interpolationTwo4DTransformations(Eigen::Matrix4d &transformation1,
                                                                       Eigen::Matrix4d &transformation2, double &t) {
    if (t < 0 || t > 1) {
        std::cout << "t value not between 0 and 1: " << t << std::endl;
        exit(-1);
    }
    Eigen::Vector3d translation1 = transformation1.block<3, 1>(0, 3);
    Eigen::Vector3d translation2 = transformation2.block<3, 1>(0, 3);
    Eigen::Quaterniond rot1(transformation1.block<3, 3>(0, 0));
    Eigen::Quaterniond rot2(transformation2.block<3, 3>(0, 0));

//    Eigen::Quaterniond rotTMP = rot1.inverse()*rot2;
//    Eigen::Quaterniond resultingRot = rotTMP.slerp(t, Eigen::Quaterniond(1,0,0,0));
//    Eigen::Vector3d resultingTranslation = (translation2-translation1)*t;
//    Eigen::Matrix4d resultingTransformation = Eigen::Matrix4d::Identity();
//    resultingTransformation.block<3, 3>(0, 0) = resultingRot.toRotationMatrix();
//    resultingTransformation.block<3, 1>(0, 3) = resultingTranslation;

    Eigen::Quaterniond resultingRot = rot1.slerp(t, rot2);
    Eigen::Vector3d resultingTranslation = translation1 * t + translation2 * (1.0 - t);

    Eigen::Matrix4d resultingTransformation = Eigen::Matrix4d::Identity();
    resultingTransformation.block<3, 3>(0, 0) = resultingRot.toRotationMatrix();
    resultingTransformation.block<3, 1>(0, 3) = resultingTranslation;

    return resultingTransformation;


}


Eigen::Matrix4d
generalHelpfulTools::getTransformationMatrix(Eigen::Vector3d &translation, Eigen::Quaterniond &rotation) {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 1>(0, 3) = translation;
    transformation.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    return transformation;
}

double generalHelpfulTools::weighted_mean(const std::vector<double> &data) {
    double mean = 0.0;

    for (int i = 0; i < data.size(); i++) {
        mean += data[i];
    }
    return mean / double(data.size());
}

void generalHelpfulTools::smooth_curve(const std::vector<double> &input, std::vector<double> &smoothedOutput,
                                       int window_half_width) {

    int window_width = 2 * window_half_width + 1;

    int size = input.size();

    std::vector<double> sample(window_width);

    for (int i = 0; i < size; i++) {

        for (int j = 0; j < window_width; j++) {

            int shifted_index = i + j - window_half_width;
            if (shifted_index < 0) shifted_index = 0;
            if (shifted_index > size - 1) shifted_index = size - 1;
            sample[j] = input[shifted_index];

        }

        smoothedOutput.push_back(generalHelpfulTools::weighted_mean(sample));

    }

}


Eigen::Matrix4d generalHelpfulTools::convertMatrixFromOurSystemToOpenCV(Eigen::Matrix4d inputMatrix) {

//    return(generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 180.0 / 180.0 * M_PI)*inputMatrix);
    double x1 = inputMatrix(0, 3);
    double y1 = inputMatrix(1, 3);
    inputMatrix(0, 3) = y1;
    inputMatrix(1, 3) = x1;
    Eigen::Matrix3d tmpMatrix = inputMatrix.block<3, 3>(0, 0).inverse();
    inputMatrix.block<3, 3>(0, 0) = tmpMatrix;
    return inputMatrix;
}


double generalHelpfulTools::normalizeAngle(double inputAngle){

    while(inputAngle<0){
        inputAngle = inputAngle+M_PI*2;
    }
    inputAngle = inputAngle+M_PI*2;

    return std::fmod(inputAngle,M_PI*2);

}

std::vector<std::string> generalHelpfulTools::getNextLineAndSplitIntoTokens(std::istream &str) {
    std::vector<std::string> result;
    std::string line;
    std::getline(str, line);

    std::stringstream lineStream(line);
    std::string cell;

    while (std::getline(lineStream, cell, ',')) {
        result.push_back(cell);
    }
    // This checks for a trailing comma with no data after it.
    if (!lineStream && cell.empty()) {
        // If there was a trailing comma then add an empty element.
        result.push_back("");
    }
    return result;
}


