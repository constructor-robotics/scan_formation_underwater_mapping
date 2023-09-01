//
// Created by tim on 16.02.21.
//
#include <iostream>

#include "softDescriptorRegistration.h"
//#include "gr/algorithms/match4pcsBase.h"
//#include "gr/algorithms/FunctorSuper4pcs.h"
//#include "gr/utils/geometry.h"
//#include <gr/algorithms/PointPairFilter.h>


#include <Eigen/Dense>


#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"



#ifndef SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H
#define SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H


class scanRegistrationClass {
public:
    scanRegistrationClass(int N = 64, int bwOut = 64 / 2, int bwIn = 64 / 2, int degLim = 64 / 2 - 1)
            : mySofftRegistrationClass(N, bwOut, bwIn, degLim) {
        sizeVoxelData = N;
        // the mutex is used for making it possible to just call for a registration in different threads.
        oursMutex = new std::mutex();


    }

    ~scanRegistrationClass() {
        mySofftRegistrationClass.~softDescriptorRegistration();
    }





    double
    sofftRegistrationVoxel2DRotationOnly(double voxelData1Input[], double voxelData2Input[], double goodGuessAlpha,double &covariance,
                                         bool debug = false);




    Eigen::Matrix4d registrationOfTwoVoxelsSOFFTFast(double voxelData1Input[],
                                                     double voxelData2Input[],
                                                     Eigen::Matrix4d initialGuess,Eigen::Matrix3d &covarianceMatrix,
                                                     bool useInitialAngle, bool useInitialTranslation,
                                                     double cellSize,
                                                     bool useGauss,
                                                     bool debug = false);

    std::vector<transformationPeak> registrationOfTwoVoxelsSOFFTAllSoluations(double voxelData1Input[],
                                                                              double voxelData2Input[],
                                                                              double cellSize,
                                                                              bool useGauss,
                                                                              bool debug = false,
                                                                              double potentialNecessaryForPeak = 0.1,
                                                                              bool multipleRadii = false,
                                                                              bool useClahe = true,
                                                                              bool useHamming = true);



private:
    softDescriptorRegistration mySofftRegistrationClass;

    int sizeVoxelData;

    std::mutex *oursMutex;
};


#endif //SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H
