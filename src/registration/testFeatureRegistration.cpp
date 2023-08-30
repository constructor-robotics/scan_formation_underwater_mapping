//
// Created by tim-external on 02.05.23.
//
// scale invariant feature transform SIFT
// speeded up robust features SURF
// ORB
//  feature detection -> Horn -> RANSAC -> Horn

#include "generalHelpfulTools.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <filesystem>
//#include "scanRegistrationClass.h"
#include "fstream"
#include <opencv2/video/tracking.hpp>

int main(int argc, char **argv) {



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


    cv::Mat magTMP1(numberOfPoints, numberOfPoints, CV_64F, voxelData1);
    cv::Mat magTMP2(numberOfPoints, numberOfPoints, CV_64F, voxelData2);

    cv::Mat convertedMat1;
    cv::Mat convertedMat2;

    magTMP1.convertTo(convertedMat1, CV_8U);
    magTMP2.convertTo(convertedMat2, CV_8U);

    cv::cvtColor(convertedMat1,convertedMat1,cv::COLOR_GRAY2BGR);
    cv::cvtColor(convertedMat2,convertedMat2,cv::COLOR_GRAY2BGR);


//    cv::Ptr<cv::AKAZE> D = cv::AKAZE::create();
//    cv::Ptr<cv::KAZE> D = cv::KAZE::create();

//    cv::Ptr<cv::ORB> D = cv::ORB::create();
//    cv::Ptr<cv::BRISK> D = cv::BRISK::create();

    cv::Ptr<cv::xfeatures2d::SURF> D = cv::xfeatures2d::SURF::create();
//    cv::Ptr<cv::SIFT> D = cv::SIFT::create();


    std::vector<cv::KeyPoint> kpts1, kpts2, matched1, matched2;
    cv::Mat desc1, desc2;

    D->detectAndCompute(convertedMat1, cv::noArray(), kpts1, desc1);
    D->detectAndCompute(convertedMat2, cv::noArray(), kpts2, desc2);

    cv::Mat J; //to save temporal images
    drawKeypoints(convertedMat1, kpts1, J);
    imshow("Keypoints 1", J);
//    imwrite("Keypoints1.jpg", J);
    drawKeypoints(convertedMat2, kpts2, J);
    imshow("Keypoints 2", J);
//    imwrite("Keypoints2.jpg", J);
    cv::waitKey(0);

    //Use brute-force matcher to find 2-nn matches
    cv::BFMatcher M(cv::NORM_L2);
//    cv::BFMatcher M(cv::NORM_HAMMING);
    std::vector< std::vector<cv::DMatch> > knn_matches;
    std::vector<cv::DMatch> good_matches;
    M.knnMatch(desc2, desc1, knn_matches, 2);
    const float nn_match_ratio = 0.8f;   // Nearest neighbor matching ratio


    //Use 2-nn matches to find correct keypoint matches
    for(size_t i = 0; i < knn_matches.size(); i++) {
        cv::DMatch nearest = knn_matches[i][0];
        double dist1 = knn_matches[i][0].distance;
        double dist2 = knn_matches[i][1].distance;
        if(dist1 < dist2*nn_match_ratio) {
            int new_i = static_cast<int>(matched1.size());
            matched1.push_back(kpts1[nearest.trainIdx]);
            matched2.push_back(kpts2[nearest.queryIdx]);
            good_matches.push_back(cv::DMatch(new_i, new_i, 0));
        }
    }

    drawMatches(convertedMat1, matched1, convertedMat2, matched2, good_matches, J);
    imshow("Matches", J);

  cv::waitKey(0);

//Use matches to compute the homography matrix
    std::vector<cv::Point2f> first;
    std::vector<cv::Point2f> second;

    for( int i = 0; i < good_matches.size(); i++ )
    {
        first.push_back( matched1[i].pt);
        second.push_back( matched2[i].pt);
    }
    cv::Mat H = findHomography(second, first, cv::RANSAC, 5.0);
    std::cout << H << std::endl;
    cv::Mat HNew = cv::estimateAffine2D(second, first);
    std::cout << HNew << std::endl;
    H = cv::Mat::eye(3,3,CV_64F);
    Eigen::Matrix3d testResultMatrix = Eigen::Matrix3d::Identity();
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 3; j++){
            H.at<double>(i,j) = HNew.at<double>(i,j);
            testResultMatrix(i,j) = H.at<double>(i,j);
        }
    }
    Eigen::Matrix3d onlyRotationMatrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d negativeTranslation = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d positiveTranslation = Eigen::Matrix3d::Identity();

    onlyRotationMatrix.block<2,2>(0,0) = testResultMatrix.block<2,2>(0,0);
//    negativeTranslation.block<2,2>(0,0) = testResultMatrix.block<2,2>(0,0);
    negativeTranslation(0,2) =-256/2;//-testResultMatrix(0,2);
    negativeTranslation(1,2) =-256/2;//-testResultMatrix(1,2);
    positiveTranslation(0,2) =256/2;//testResultMatrix(0,2);
    positiveTranslation(1,2) =256/2;//testResultMatrix(1,2);

//    std::cout << H << std::endl;
//    std::cout << onlyRotationMatrix << std::endl;
//    std::cout << positiveTranslation << std::endl;
//    std::cout << negativeTranslation << std::endl;
//    std::cout << testResultMatrix << std::endl;
//    std::cout << positiveTranslation*testResultMatrix*negativeTranslation << std::endl;
    Eigen::Matrix3d testHowGood = negativeTranslation*testResultMatrix*positiveTranslation;
    std::cout << testHowGood << std::endl;
    double x = testHowGood(0,2);
    double y = testHowGood(1,2);
    testHowGood(0,2) = -y*50.0/512.0;
    testHowGood(1,2) = -x*50.0/512.0;
    std::cout << testHowGood << std::endl;

//    std::cout << testResultMatrix << std::endl;



    cv::cvtColor(convertedMat1,convertedMat1,cv::COLOR_BGR2GRAY);
    cv::cvtColor(convertedMat2,convertedMat2,cv::COLOR_BGR2GRAY);

    //Apply the computed homography matrix to warp the second image
    cv::Mat Panorama(convertedMat1.rows, 2 * convertedMat1.cols,  CV_8U);
    warpPerspective(convertedMat2, Panorama, H, Panorama.size());


    Panorama.convertTo(Panorama,CV_64F,1.0/255.0);
    convertedMat1.convertTo(convertedMat1,CV_64F,1.0/255.0);
    convertedMat2.convertTo(convertedMat2,CV_64F,1.0/255.0);

    for(int i = 0; i < convertedMat1.rows; i++){
        for(int j = 0; j < convertedMat1.cols; j++){
            Panorama.at<double>(i,j) = (Panorama.at<double>(i,j)+convertedMat1.at<double>(i,j))/2.0;
        }
    }
    imshow("Panorama1", Panorama);
    for(int i = 0; i < convertedMat1.rows; i++){
        for(int j = 0; j < convertedMat1.cols; j++){
            Panorama.at<double>(i,j) = (convertedMat1.at<double>(i,j)+convertedMat2.at<double>(i,j))/2.0;
        }
    }
//    imshow("Panorama2", Panorama);
//    imshow("convertedMat1", convertedMat1);
//    imshow("convertedMat2", convertedMat2);
    //Combining the result with the first image
//    for(int i = 0; i < Panorama.rows; i++){
//        for(int j = 0; j < Panorama.cols; j++){
//
//            std::cout << Panorama.at<double>(i,j) << std::endl;
////            std::cout << convertedMat1.at<double>(i,j) << std::endl;
//            Panorama.at<double>(i,j) = 0.0;//convertedMat1.at<double>(i,j);
//            std::cout << Panorama.at<double>(i,j) << std::endl;
//            std::cout << "next" << std::endl;
//        }
//
//    }

//    magTMP1.convertTo(convertedMat1, CV_64F,1.0/255.0);
//    magTMP2.convertTo(convertedMat2, CV_64F,1.0/255.0);
////    std::cout << convertedMat1.type() << std::endl;
//    for(int i = 50; i < 250; i++){
//        for(int j = 50; j < 250; j++) {
//            convertedMat1.at<double>(i,j) = 1.0;
//        }
//    }
//    imshow("Panorama2", convertedMat1);

    cv::waitKey(0);
    return (0);
}
