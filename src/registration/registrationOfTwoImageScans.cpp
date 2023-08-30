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

void convertMatToDoubleArray(cv::Mat inputImg, double voxelData[], int dimensionScan) {

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
    fftw_complex *resultingCorrelationComplex;
    resultingCorrelationComplex = fftw_alloc_complex(5);

    double customRotation = +80;

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


    cv::Mat img1 = cv::imread(
            all_args[0],
            cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(
            all_args[1],
            cv::IMREAD_GRAYSCALE);

    cv::Point2f pc(img2.cols / 2., img2.rows / 2.);
    cv::Mat r = cv::getRotationMatrix2D(pc, -customRotation, 1.0);
    std::cout << r << std::endl;
//    cv::imshow("Display window", img2);
//    int k = cv::waitKey(0); // Wait for a keystroke in the window

    cv::warpAffine(img2, img2, r, img2.size()); // what size I should use?
    std::cout << img2.data[6] << std::endl;

//    cv::imshow("test1",img1);
//    cv::imshow("test2",img2);
//    cv::waitKey(0);
//    cv::Mat img1 = cv::imread("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/FMT/firstImage.jpg", cv::IMREAD_GRAYSCALE);
//    cv::Mat img2 = cv::imread("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/FMT/secondImage.jpg", cv::IMREAD_GRAYSCALE);
    int dimensionScan = img2.rows;
    std::cout << "image size: " << dimensionScan << std::endl;
    double *voxelData1;
    double *voxelData2;
    voxelData1 = (double *) malloc(sizeof(double) * dimensionScan * dimensionScan);
    voxelData2 = (double *) malloc(sizeof(double) * dimensionScan * dimensionScan);

    convertMatToDoubleArray(img1, voxelData1, dimensionScan);
    convertMatToDoubleArray(img2, voxelData2, dimensionScan);

    scanRegistrationClass scanRegistrationObject(img1.rows, img1.rows / 2, img1.rows / 2, img1.rows / 2 - 1);

    double maximumMagnitude1 = 0;
    double maximumMagnitude2 = 0;
    //get magnitude and find maximum
    for (int j = 0; j < dimensionScan * dimensionScan; j++) {
        if (voxelData1[j] > maximumMagnitude1) {
            maximumMagnitude1 = voxelData1[j];
        }
    }
    for (int j = 0; j < dimensionScan * dimensionScan; j++) {
        if (voxelData2[j] > maximumMagnitude2) {
            maximumMagnitude2 = voxelData2[j];
        }
    }
    if (maximumMagnitude2 > maximumMagnitude1) {
        maximumMagnitude1 = maximumMagnitude2;
    }

    for (int j = 0; j < dimensionScan * dimensionScan; j++) {

        voxelData2[j] = voxelData2[j] / maximumMagnitude1;
        voxelData1[j] = voxelData1[j] / maximumMagnitude1;
    }

    double fitnessX;
    double fitnessY;
    std::vector<transformationPeak> estimatedTransformations = scanRegistrationObject.registrationOfTwoVoxelsSOFFTAllSoluations(
            voxelData1, voxelData2, 0.5,false, true, 5,
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




//    Eigen::Matrix4d tmpMatrix4d = estimatedTransformation.inverse();
//    estimatedTransformation = tmpMatrix4d;

//    estimatedTransformation(1, 3) = estimatedTransformation(1, 3)-5;
//    estimatedTransformation(0, 3) = estimatedTransformation(0, 3)-2;
//    Eigen::Matrix4d tmpMatrixRotationCV = generalHelpfulTools::convertMatrixFromOurSystemToOpenCV(currentMatrix);

    Eigen::Vector3d rpyTMP = generalHelpfulTools::getRollPitchYaw(Eigen::Quaterniond(currentMatrix.block<3, 3>(0, 0)));

    r = cv::getRotationMatrix2D(pc, rpyTMP[2] * 180.0 / M_PI, 1.0);
    double warp_values[] = {1.0, 0.0, currentMatrix(1, 3), 0.0, 1.0, currentMatrix(0, 3)};
    cv::Mat translation_matrix = cv::Mat(2, 3, CV_64F, warp_values);
//    cv::Mat transformMat = (cv::Mat_<double>(2, 3) << tmpMatrixRotationCV(0, 0),
//            tmpMatrixRotationCV(0,1),
//            tmpMatrixRotationCV(0, 3),
//            tmpMatrixRotationCV(1,0),
//            tmpMatrixRotationCV(1,1),
//            tmpMatrixRotationCV(1, 3));//@TODO we need to create a "conversion" system for rotating the data






    cv::Mat magTMP1(dimensionScan, dimensionScan, CV_64F, voxelData1);
    //add gaussian blur
    //            cv::imwrite("/home/tim-external/Documents/imreg_fmt/firstImage.jpg", magTMP1);

    cv::Mat magTMP2(dimensionScan, dimensionScan, CV_64F, voxelData2);
//    cv::imshow("testTest1",magTMP1);
//    cv::imshow("testTest2",magTMP2);
//    cv::waitKey(0);

    std::cout << "estimatedTransformation:" << std::endl;
//    std::cout << estimatedTransformation << std::endl;
//    std::cout << estimatedTransformation.inverse() << std::endl;
//    std::cout << "transformMat:" << std::endl;
//    std::cout << transformMat << std::endl;


    //rotation
    cv::warpAffine(magTMP1, magTMP1, r, magTMP1.size());
    //translation
    cv::warpAffine(magTMP1, magTMP1, translation_matrix, magTMP1.size());



//            convertMatToDoubleArray(img1, voxelData1);
//            convertMatToDoubleArray(img2, voxelData2);
    std::ofstream myFile1, myFile2;
    myFile1.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultVoxel1.csv");
    myFile2.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultVoxel2.csv");
    for (int j = 0; j < dimensionScan; j++) {
        for (int i = 0; i < dimensionScan; i++) {
            myFile1 << voxelData1[j + dimensionScan * i]; // real part
            myFile1 << "\n";
            myFile2 << voxelData2[j + dimensionScan * i]; // imaginary part
            myFile2 << "\n";
        }
    }
    myFile1.close();
    myFile2.close();





//    Eigen::Matrix4d estimatedTransformation = scanRegistrationObject.sofftRegistration(*scan1,*scan2,fitnessX,fitnessY,-100,true);


    return (0);
}
