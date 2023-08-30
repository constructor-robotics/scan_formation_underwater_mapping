//
// Created by tim-linux on 26.03.22.
//

//
// Created by jurobotics on 13.09.21.
//
// /home/tim-external/dataFolder/StPereDataset/lowNoise52/scanNumber_0/00_ForShow.jpg /home/tim-external/dataFolder/StPereDataset/lowNoise52/scanNumber_1/00_ForShow.jpg
// /home/tim-external/dataFolder/ValentinBunkerData/noNoise305_52/scanNumber_0/00_ForShow.jpg  /home/tim-external/dataFolder/ValentinBunkerData/noNoise305_52/scanNumber_1/00_ForShow.jpg
#include "generalHelpfulTools.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include "scanRegistrationClass.h"
#include "sofftCorrelationClass.h"

//#include "soft20/s2_cospmls.h"
//#include "soft20/s2_semi_memo.h"
//#include "soft20/makeweights.h"
//#include "soft20/so3_correlate_fftw.h"
//#include "soft20/soft_fftw.h"
//#include "fftw3.h"
#include "soft20/wrap_fftw.h"
void convertMatToDoubleArray(cv::Mat inputImg, double voxelData[],int dimensionScan) {

    std::vector<uchar> array;
    if (inputImg.isContinuous()) {
        // array.assign(mat.datastart, mat.dataend); // <- has problems for sub-matrix like mat = big_mat.row(i)
        array.assign(inputImg.data, inputImg.data + inputImg.total() * inputImg.channels());
    } else {
        for (int i = 0; i < inputImg.rows; ++i) {
            array.insert(array.end(), inputImg.ptr<uchar>(i),
                         inputImg.ptr<uchar>(i) + inputImg.cols * inputImg.channels());
        }
    }

//    for (int i = 0; i < array.size(); i++) {
//        voxelData[i] = array[i];
////        std::cout << voxelData[i] <<std::endl;
////        std::cout << "i: "<< i <<std::endl;
//    }
    for (int j = 0; j < dimensionScan; j++) {
        for (int i = 0; i < dimensionScan; i++) {
            voxelData[j + dimensionScan * i] = array[i + dimensionScan * j];
        }
    }
}


int main(int argc, char **argv) {
    // input needs to be two scans as voxelData
    int N = 64;
    int bwOut = N / 2;
    int bwIn = N / 2;
    int degLim = N / 2 - 1;

    sofftCorrelationClass sofftCorrelationObject(N, bwOut, bwIn,
                                                 degLim);
    double *resampledMagnitudeSO3_1Input = (double *) malloc(sizeof(double) * N * N);

    double *resampledMagnitudeSO3_1 = (double *) malloc(sizeof(double) * N * N);

    double *resampledMagnitudeSO3_2 = (double *) malloc(sizeof(double) * N * N);
    double *resampledMagnitudeSO3_2Input = (double *) malloc(sizeof(double) * N * N);

    int bandwidth = N/2;
    for (int j = 0; j < 2 * bandwidth; j++) {
        for (int k = 0; k < 2 * bandwidth; k++) {
            resampledMagnitudeSO3_1Input[j + k * bandwidth * 2] = 0;
            resampledMagnitudeSO3_1[j + k * bandwidth * 2] = 0;
            resampledMagnitudeSO3_2[j + k * bandwidth * 2] = 0;
            resampledMagnitudeSO3_2Input[j + k * bandwidth * 2] = 0;
        }
    }

//    for (int j = 30; j < 38; j++) {
//        for (int k = 0; k < 2 * bandwidth; k++) {
//            resampledMagnitudeSO3_1[j + k * bandwidth * 2] = 1;
//        }
//    }
    for (int j = 38; j < 46; j++) {
        for (int k = 0; k < 2 * bandwidth; k++) {
            resampledMagnitudeSO3_1Input[j + k * bandwidth * 2] = 1;
        }
    }
    for (int j = 38; j < 46; j++) {
        for (int k = 0; k < 2 * bandwidth; k++) {
            resampledMagnitudeSO3_2Input[j + k * bandwidth * 2] = 1;
        }
    }


//    s2RotateFFTW(bwOut,resampledMagnitudeSO3_1Input,resampledMagnitudeSO3_1,0.0,M_PI/2,0.0,1);
    s2RotateFFTW(bwOut,resampledMagnitudeSO3_1Input,resampledMagnitudeSO3_1,3.04,1.7181,4.4179,1);

    s2RotateFFTW(bwOut,resampledMagnitudeSO3_2Input,resampledMagnitudeSO3_2,0.0,0,0,1);


//    for (int j = 0; j < 2 * bandwidth; j++) {
//        for (int k = 38; k < 46; k++) {
//            resampledMagnitudeSO3_2[j + k * bandwidth * 2] = 1;
//        }
//    }


//    for (int j = 0; j < N; j++) {
//        for (int k = 50; k < N; k++) {
//            resampledMagnitudeSO3_1[j + k * bandwidth * 2] = 1;
//        }
//    }
//
//    for (int j = 0; j < N; j++) {
//        for (int k = 0; k < 14; k++) {
//            resampledMagnitudeSO3_2[j + k * bandwidth * 2] = 1;
//        }
//    }



    std::ofstream myFile7, myFile8;
    myFile7.open(
            "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resampledVoxel1.csv");
    myFile8.open(
            "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resampledVoxel2.csv");

    for (int j = 0; j < N; j++) {
        for (int k = 0; k < N; k++) {
            myFile7 << resampledMagnitudeSO3_1[j + k * bandwidth * 2]; // real part
            myFile7 << "\n";
            myFile8 << resampledMagnitudeSO3_2[j + k * bandwidth * 2]; // real part
            myFile8 << "\n";
        }
    }
    myFile7.close();
    myFile8.close();

    fftw_complex *resultingCorrelationComplex;

    resultingCorrelationComplex = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * (8 * bwOut * bwOut * bwOut));



    sofftCorrelationObject.correlationOfTwoSignalsInSO3(resampledMagnitudeSO3_1,resampledMagnitudeSO3_2,resultingCorrelationComplex);
//
//
    FILE *fp;
    fp = fopen(
            "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultCorrelation3D.csv",
            "w");
    for (int i = 0; i < 8 * bwOut * bwOut * bwOut; i++)
        fprintf(fp, "%.16f\n", resultingCorrelationComplex[i][0]);
    fclose(fp);



//    Eigen::Matrix4d estimatedTransformation = scanRegistrationObject.sofftRegistration(*scan1,*scan2,fitnessX,fitnessY,-100,true);

    fftw_free(resultingCorrelationComplex);
    return (0);
}
