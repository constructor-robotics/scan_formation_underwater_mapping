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



int main(int argc, char **argv) {
    // input needs to be two scans as voxelData
    fftw_complex *resultingCorrelationComplex;
    resultingCorrelationComplex = fftw_alloc_complex(5);

    double customRotation = +0;

    std::string current_exec_name = argv[0]; // Name of the current exec program
    std::vector<std::string> all_args;

    if (argc > 0) {
        //std::cout << "temp1" << std::endl;
        all_args.assign(argv + 1, argv + argc);
        //std::cout << "12"<< all_args[1]<<std::endl;
    } else {
        std::cout << "no arguments given" << std::endl;
        exit(-1);
    }



    int numberOfPoints = stoi(all_args[2]);

    double *voxelData1;
    voxelData1 = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);

    std::ifstream voxelDataFile(all_args[0]);
    for (int i = 0; i < numberOfPoints; i++) {
        std::vector<std::string> voxelDataVector = generalHelpfulTools::getNextLineAndSplitIntoTokens(voxelDataFile);
        for (int j = 0; j < numberOfPoints; j++) {
            voxelData1[i + numberOfPoints * j] = std::stod(voxelDataVector[j]);
        }
    }

    double *voxelData2;
    voxelData2 = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);
    std::ifstream voxelDataShiftedFile(all_args[1]);
    for (int i = 0; i < numberOfPoints; i++) {
        std::vector<std::string> voxelDataShiftedVector = generalHelpfulTools::getNextLineAndSplitIntoTokens(
                voxelDataShiftedFile);
        for (int j = 0; j < numberOfPoints; j++) {
            voxelData2[i + numberOfPoints * j] = std::stod(voxelDataShiftedVector[j]);
            //                std::cout <<voxelDataShifted[i + numberOfPoints * j] << std::endl;
        }
    }





    scanRegistrationClass scanRegistrationObject(numberOfPoints, numberOfPoints / 2, numberOfPoints / 2, numberOfPoints / 2 - 1);

    double maximumMagnitude1 = 0;
    double maximumMagnitude2 = 0;
    //get magnitude and find maximum
    for (int j = 0; j < numberOfPoints * numberOfPoints; j++) {
        if (voxelData1[j] > maximumMagnitude1) {
            maximumMagnitude1 = voxelData1[j];
        }
    }
    for (int j = 0; j < numberOfPoints * numberOfPoints; j++) {
        if (voxelData2[j] > maximumMagnitude2) {
            maximumMagnitude2 = voxelData2[j];
        }
    }
    if (maximumMagnitude2 > maximumMagnitude1) {
        maximumMagnitude1 = maximumMagnitude2;
    }

    for (int j = 0; j < numberOfPoints * numberOfPoints; j++) {
        voxelData2[j] = voxelData2[j] / maximumMagnitude1;
        voxelData1[j] = voxelData1[j] / maximumMagnitude1;
    }

    double fitnessX;
    double fitnessY;
    std::vector<transformationPeak> estimatedTransformations = scanRegistrationObject.registrationOfTwoVoxelsSOFFTAllSoluations(
            voxelData1, voxelData2, 1.0,false, true, 5,
            false, true, true);// 5 and


    double highestPeak = 0;
    Eigen::Matrix4d currentMatrix = Eigen::Matrix4d::Identity();
    for (auto &estimatedTransformation: estimatedTransformations) {
        //rotation

        for (auto &potentialTranslation: estimatedTransformation.potentialTranslations) {
            if (potentialTranslation.peakHeight > highestPeak) {
                currentMatrix.block<3, 3>(0, 0) = generalHelpfulTools::getQuaternionFromRPY(0, 0,
                                                                                            estimatedTransformation.potentialRotation.angle).toRotationMatrix();
                currentMatrix.block<3, 1>(0, 3) = Eigen::Vector3d(potentialTranslation.translationSI.x(),
                                                                  potentialTranslation.translationSI.y(), 0);
                std::cout << estimatedTransformation.potentialRotation.angle << std::endl;
                highestPeak = potentialTranslation.peakHeight;
            }
            //translation

        }

    }
    std::cout << "our match" << std::endl;
    std::cout << currentMatrix << std::endl;

    cv::Mat magTMP1(numberOfPoints, numberOfPoints, CV_64F, voxelData1);
    //add gaussian blur
    //            cv::imwrite("/home/tim-external/Documents/imreg_fmt/firstImage.jpg", magTMP1);

    cv::Mat magTMP2(numberOfPoints, numberOfPoints, CV_64F, voxelData2);
//    cv::imshow("testTest1",magTMP1);
//    cv::imshow("testTest2",magTMP2);
//    cv::waitKey(0);

    std::cout << "estimatedTransformation:" << std::endl;

    std::ofstream myFile1, myFile2;
    myFile1.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultVoxel1.csv");
    myFile2.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultVoxel2.csv");
    for (int j = 0; j < numberOfPoints; j++) {
        for (int i = 0; i < numberOfPoints; i++) {
            myFile1 << voxelData1[j + numberOfPoints * i]; // real part
            myFile1 << "\n";
            myFile2 << voxelData2[j + numberOfPoints * i]; // imaginary part
            myFile2 << "\n";
        }
    }
    myFile1.close();
    myFile2.close();





//    Eigen::Matrix4d estimatedTransformation = scanRegistrationObject.sofftRegistration(*scan1,*scan2,fitnessX,fitnessY,-100,true);


    return (0);
}
