//
// Created by tim on 16.02.21.
//


#include "scanRegistrationClass.h"





//double scanRegistrationClass::sofftRegistrationVoxel2DRotationOnly(double voxelData1Input[], double voxelData2Input[],
//                                                                   double goodGuessAlpha,double &covariance, bool debug) {
//
//
//    return mySofftRegistrationClass.sofftRegistrationVoxel2DRotationOnly(voxelData1Input, voxelData2Input,
//                                                                         goodGuessAlpha,covariance, debug);
//
//}

Eigen::Matrix4d scanRegistrationClass::registrationOfTwoVoxelsSOFFTFast(double voxelData1Input[],
                                                                        double voxelData2Input[],
                                                                        Eigen::Matrix4d initialGuess,
                                                                        Eigen::Matrix3d &covarianceMatrix,
                                                                        bool useInitialAngle,
                                                                        bool useInitialTranslation,
                                                                        double cellSize,
                                                                        bool useGauss,
                                                                        bool debug) {

    std::lock_guard<std::mutex> guard(*this->oursMutex);

    //changing voxel 1 and 2 because we want to have the transformation from 1 to 2 and not from 2 to 1(which is the registration)@TODO
    return mySofftRegistrationClass.registrationOfTwoVoxelsSOFFTFast(voxelData1Input,
                                                                     voxelData2Input,
                                                                     initialGuess,covarianceMatrix,
                                                                     useInitialAngle, useInitialTranslation,
                                                                     cellSize,
                                                                     useGauss,
                                                                     debug);
}

//std::vector<transformationPeak>
//scanRegistrationClass::registrationOfTwoVoxelsSOFFTAllSoluations(double voxelData1Input[],
//                                                                 double voxelData2Input[],
//                                                                 double cellSize,
//                                                                 bool useGauss,
//                                                                 bool debug, double potentialNecessaryForPeak,
//                                                                 bool multipleRadii,
//                                                                 bool useClahe,
//                                                                 bool useHamming) {
//
//    std::lock_guard<std::mutex> guard(*this->oursMutex);
//
//    return mySofftRegistrationClass.registrationOfTwoVoxelsSOFFTAllSoluations(voxelData1Input,
//                                                                              voxelData2Input,
//                                                                              cellSize,
//                                                                              useGauss,
//                                                                              debug, potentialNecessaryForPeak,
//                                                                              multipleRadii,
//                                                                              useClahe,
//                                                                              useHamming);
//}

