//
// Created by tim-linux on 01.03.22.
//

#ifndef UNDERWATERSLAM_SOFTDESCRIPTORREGISTRATION_H
#define UNDERWATERSLAM_SOFTDESCRIPTORREGISTRATION_H

#include "sofftCorrelationClass.h"
#include "PeakFinder.h"
#include "generalHelpfulTools.h"
//#include "slamToolsRos.h"

//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/common/transforms.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_points_d_traits_2.h>
#include <CGAL/Random.h>

#include <iostream>
#include <fstream>
#include "findpeaks/mask.hpp"
#include <findpeaks/persistence.hpp>


struct rotationPeak {
    double angle;
    double peakCorrelation;
    double covariance;
};

struct translationPeak {
    Eigen::Vector2d translationSI;
    Eigen::Vector2i translationVoxel;
    double peakHeight;
    Eigen::Matrix2d covariance;
};

struct transformationPeak {
    std::vector<translationPeak> potentialTranslations;
    rotationPeak potentialRotation;
};

class softDescriptorRegistration {
public:
    softDescriptorRegistration(int N, int bwOut, int bwIn, int degLim) : sofftCorrelationObject(N, bwOut, bwIn,
                                                                                                degLim) {
        this->N = N;
        this->correlationN = N * 2 - 1;
        this->bwOut = bwOut;
        this->bwIn = bwIn;
        this->degLim = degLim;
        this->resultingCorrelationDouble = (double *) malloc(sizeof(double) * this->correlationN * this->correlationN);
        this->resultingCorrelationComplex = fftw_alloc_complex(8 * bwOut * bwOut * bwOut);
//        (fftw_complex *) fftw_malloc(
//                sizeof(fftw_complex) * (8 * bwOut * bwOut * bwOut));
        this->resultingPhaseDiff2D = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N);
        this->resultingPhaseDiff2DCorrelation = (fftw_complex *) fftw_malloc(
                sizeof(fftw_complex) * this->correlationN * this->correlationN);
        this->resultingShiftPeaks2D = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N);
        this->resultingShiftPeaks2DCorrelation = (fftw_complex *) fftw_malloc(
                sizeof(fftw_complex) * this->correlationN * this->correlationN);

        this->magnitude1Shifted = (double *) malloc(sizeof(double) * N * N * N);
        this->magnitude2Shifted = (double *) malloc(sizeof(double) * N * N * N);
        this->voxelData1 = (double *) malloc(sizeof(double) * N * N * N);
        this->voxelData2 = (double *) malloc(sizeof(double) * N * N * N);
//        this->spectrum1 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
//        this->spectrum2 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
        this->spectrumOut = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
        this->spectrumOutCorrelation = (fftw_complex *) fftw_malloc(
                sizeof(fftw_complex) * this->correlationN * this->correlationN * this->correlationN);
        this->phase1 = (double *) malloc(sizeof(double) * N * N * N);
        this->phase2 = (double *) malloc(sizeof(double) * N * N * N);
        this->magnitude1 = (double *) malloc(sizeof(double) * N * N * N);
        this->magnitude2 = (double *) malloc(sizeof(double) * N * N * N);
        this->phase1Correlation = (double *) malloc(
                sizeof(double) * this->correlationN * this->correlationN * this->correlationN);
        this->phase2Correlation = (double *) malloc(
                sizeof(double) * this->correlationN * this->correlationN * this->correlationN);
        this->magnitude1Correlation = (double *) malloc(
                sizeof(double) * this->correlationN * this->correlationN * this->correlationN);
        this->magnitude2Correlation = (double *) malloc(
                sizeof(double) * this->correlationN * this->correlationN * this->correlationN);
        resampledMagnitudeSO3_1 = (double *) malloc(sizeof(double) * N * N);
        resampledMagnitudeSO3_2 = (double *) malloc(sizeof(double) * N * N);
        resampledMagnitudeSO3_1TMP = (double *) malloc(sizeof(double) * N * N);
        resampledMagnitudeSO3_2TMP = (double *) malloc(sizeof(double) * N * N);
        inputSpacialData = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
        inputSpacialDataCorrelation = (fftw_complex *) fftw_malloc(
                sizeof(fftw_complex) * this->correlationN * this->correlationN * this->correlationN);
//        planToFourierVoxel = fftw_plan_dft_3d(N, N, N, resultingPhaseDiff2D,
//                                              resultingShiftPeaks2D, FFTW_BACKWARD, FFTW_ESTIMATE);
        planFourierToVoxel2D = fftw_plan_dft_2d(N, N, resultingPhaseDiff2D,
                                                resultingShiftPeaks2D, FFTW_BACKWARD, FFTW_ESTIMATE);
        planFourierToVoxel2DCorrelation = fftw_plan_dft_2d(this->correlationN, this->correlationN,
                                                           resultingPhaseDiff2DCorrelation,
                                                           resultingShiftPeaks2DCorrelation, FFTW_BACKWARD,
                                                           FFTW_ESTIMATE);
//        correlation2DResult = (double *) malloc(sizeof(double) * N * N);


        planVoxelToFourier3D = fftw_plan_dft_3d(N, N, N, inputSpacialData,
                                                spectrumOut, FFTW_FORWARD, FFTW_ESTIMATE);
        planVoxelToFourier2D = fftw_plan_dft_2d(N, N, inputSpacialData,
                                                spectrumOut, FFTW_FORWARD, FFTW_ESTIMATE);
        planVoxelToFourier2DCorrelation = fftw_plan_dft_2d(this->correlationN, this->correlationN,
                                                           inputSpacialDataCorrelation,
                                                           spectrumOutCorrelation, FFTW_FORWARD, FFTW_ESTIMATE);
    }

    ~softDescriptorRegistration() {
        sofftCorrelationObject.~sofftCorrelationClass();


//        free(this->resultingCorrelationDouble);
//        fftw_free(this->resultingCorrelationComplex);
//        fftw_free(this->resultingPhaseDiff2D );
//        fftw_free(this->resultingShiftPeaks2D);
//        fftw_free(this->magnitude1Shifted );
//        fftw_free(this->magnitude2Shifted );
//        fftw_free(this->voxelData1 );
//        fftw_free(this->voxelData2 );
//        fftw_free(this->spectrumOut );
//        fftw_free(this->phase1);
//        fftw_free(this->phase2);
//        fftw_free(this->magnitude1 );
//        fftw_free(this->magnitude2 );
//        fftw_free(resampledMagnitudeSO3_1 );
//        fftw_free(resampledMagnitudeSO3_2);
//        fftw_free(resampledMagnitudeSO3_1TMP);
//        fftw_free(resampledMagnitudeSO3_2TMP );
//        fftw_free(inputSpacialData);
//        fftw_destroy_plan(planFourierToVoxel2D);
//        fftw_destroy_plan(planVoxelToFourier3D);
//        fftw_destroy_plan(planVoxelToFourier2D);


    }


    double
    getSpectrumFromVoxelData2D(double voxelData[], double magnitude[], double phase[], bool gaussianBlur = false);

//    Eigen::Matrix4d
//    registrationOfTwoVoxel2D(double voxelData1[], double voxelData2[], double &fitnessX, double &fitnessY,
//                             double goodGuessAlpha, bool debug);


    double
    sofftRegistrationVoxel2DRotationOnly(double voxelData1Input[], double voxelData2Input[], double goodGuessAlpha,double &covariance,
                                         bool debug = false);

    std::vector<rotationPeak>
    sofftRegistrationVoxel2DListOfPossibleRotations(double voxelData1Input[], double voxelData2Input[],
                                                    bool debug = false, bool multipleRadii = false,
                                                    bool useClahe = true, bool useHamming = true);

//    Eigen::Vector2d sofftRegistrationVoxel2DTranslation(double voxelData1Input[],
//                                                        double voxelData2Input[],
//                                                        double &fitnessX, double &fitnessY, double cellSize,
//                                                        Eigen::Vector3d initialGuess, bool useInitialGuess,
//                                                        double &heightMaximumPeak, bool debug = false);

    Eigen::Matrix4d registrationOfTwoVoxelsSOFFTFast(double voxelData1Input[],
                                                     double voxelData2Input[],
                                                     Eigen::Matrix4d &initialGuess,Eigen::Matrix3d &covarianceMatrix,
                                                     bool useInitialAngle, bool useInitialTranslation,
                                                     double cellSize,
                                                     bool useGauss,
                                                     bool debug = false,
                                                     double potentialNecessaryForPeak = 0.1);

    std::vector<transformationPeak> registrationOfTwoVoxelsSOFFTAllSoluations(double voxelData1Input[],
                                                                              double voxelData2Input[],
                                                                              double cellSize,
                                                                              bool useGauss,
                                                                              bool debug = false,
                                                                              double potentialNecessaryForPeak = 0.1,
                                                                              bool multipleRadii = false,
                                                                              bool useClahe = true,
                                                                              bool useHamming = true);

    double getSpectrumFromVoxelData2DCorrelation(double voxelData[], double magnitude[], double phase[],
                                                 bool gaussianBlur, double normalizationFactor);

    std::vector<translationPeak> sofftRegistrationVoxel2DTranslationAllPossibleSolutions(double voxelData1Input[],
                                                                                         double voxelData2Input[],
                                                                                         double cellSize,
                                                                                         double normalizationFactor,
                                                                                         bool debug = false,
                                                                                         int numberOfRotationForDebug = 0,
                                                                                         double potentialNecessaryForPeak = 0.1);

    std::vector<translationPeak>
    peakDetectionOf2DCorrelationSimpleDouble1D(double maximumCorrelation, double cellSize, int impactOfNoiseFactor = 2,
                                               double percentageOfMaxCorrelationIgnored = 0.10);

    std::vector<translationPeak>
    peakDetectionOf2DCorrelationOpenCVHoughTransform(double maximumCorrelation, double cellSize,
                                                     int impactOfNoiseFactor = 2,
                                                     double percentageOfMaxCorrelationIgnored = 0.10);

    bool isPeak(cv::Mat mx[], std::vector<cv::Point> &conn_points);

    cv::Mat imregionalmax(cv::Mat &src);

    double normalizationFactorCalculation(int x, int y);

    cv::Mat opencv_imextendedmax(cv::Mat &inputMatrix, double hParam);

    void imextendedmax_imreconstruct(cv::Mat g, cv::Mat f, cv::Mat &dest);

    std::vector<translationPeak>
    peakDetectionOf2DCorrelationFindPeaksLibrary(double cellSize, double potentialNecessaryForPeak = 0.1,
                                                 double ignoreSidesPercentage = 0.05);


private://here everything is created. malloc is done in the constructor




    int N, correlationN;//describes the size of the overall voxel system + correlation N
    int bwOut, bwIn, degLim;
    double *voxelData1;
    double *voxelData2;
//    fftw_complex *spectrum1;
//    fftw_complex *spectrum2;
    fftw_complex *spectrumOut;
    fftw_complex *spectrumOutCorrelation;
    double *magnitude1;
    double *magnitude2;
    double *magnitude1Correlation;
    double *magnitude2Correlation;
    double *phase1;
    double *phase2;
    double *phase1Correlation;
    double *phase2Correlation;
    double *magnitude1Shifted;
    double *magnitude2Shifted;
    double *resampledMagnitudeSO3_1;
    double *resampledMagnitudeSO3_2;
    double *resampledMagnitudeSO3_1TMP;
    double *resampledMagnitudeSO3_2TMP;
    sofftCorrelationClass sofftCorrelationObject;
    fftw_complex *resultingCorrelationComplex;
    fftw_complex *resultingPhaseDiff2D;
    fftw_complex *resultingPhaseDiff2DCorrelation;
    fftw_complex *resultingShiftPeaks2D;
    fftw_complex *resultingShiftPeaks2DCorrelation;
    double *resultingCorrelationDouble;
//    fftw_plan planToFourierVoxel;
//    double *correlation2DResult;
    fftw_complex *inputSpacialData;
    fftw_complex *inputSpacialDataCorrelation;
    fftw_plan planVoxelToFourier3D;
    fftw_plan planVoxelToFourier2D;
    fftw_plan planVoxelToFourier2DCorrelation;
    fftw_plan planFourierToVoxel2D;
    fftw_plan planFourierToVoxel2DCorrelation;
};


#endif //UNDERWATERSLAM_SOFTDESCRIPTORREGISTRATION_H
