//
// Created by tim on 16.02.21.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/projection_matrix.h>

//#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/common/time.h>
//#include <pcl/console/print.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/fpfh_omp.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/visualization/pcl_visualizer.h>

#include "softDescriptorRegistration.h"
#include "gr/algorithms/match4pcsBase.h"
#include "gr/algorithms/FunctorSuper4pcs.h"
#include "gr/utils/geometry.h"
#include <gr/algorithms/PointPairFilter.h>


#include <Eigen/Dense>


#include <ndt_matcher_p2d.h>
#include <ndt_matcher_d2d_2d.h>

#include <image_registration.h>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"

#include <gmm_registration/front_end/GaussianMixturesModel.h>
#include <gmm_registration/front_end/GmmFrontEnd.hpp>
#include <gmm_registration/method/DistributionToDistribution2D.h>
#include <gmm_registration/solver/CholeskyLineSearchNewtonMethod.h>
#include <gmm_registration/method/PointsToDistribution2D.h>


#ifndef SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H
#define SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H


class scanRegistrationClass {
public:
    scanRegistrationClass(int N = 64, int bwOut = 64 / 2, int bwIn = 64 / 2, int degLim = 64 / 2 - 1)
            : mySofftRegistrationClass(N, bwOut, bwIn, degLim) {
        sizeVoxelData = N;

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
