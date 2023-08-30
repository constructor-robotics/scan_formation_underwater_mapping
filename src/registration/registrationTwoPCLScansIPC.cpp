//
// Created by tim-linux on 26.03.22.
//

//
// Created by jurobotics on 13.09.21.
//


#include "generalHelpfulTools.h"
#include "slamToolsRos.h"

int main(int argc, char **argv) {
    // 1Cloud 2Cloud InitialGuessAngle
    std::string current_exec_name = argv[0]; // Name of the current exec program
    std::vector<std::string> all_args;

    if (argc > 0) {
        //std::cout << "temp1" << std::endl;
        all_args.assign(argv + 1, argv + argc);
        //std::cout << "12"<< all_args[1]<<std::endl;
    }else{
        std::cout << "no arguments given" << std::endl;
        exit(-1);
    }

//    for(int i=0;i<all_args.size();i++){
//        std::cout << "here: "<< i << " string: " << all_args[i]<< std::endl;
//    }
//    std::cout<< "test" << std::endl;
//    std::cout << "Keyframe: " << all_args[0]<< std::endl;
//    std::cout<< "test1" << std::endl;
//    std::cout << "Keyframe: " << all_args[1]<< std::endl;
//    std::cout<< "test2" << std::endl;

    double initialGuessAngleYaw = std::atof(all_args[2].c_str());//this is the angle






    scanRegistrationClass scanRegistrationObject;

//    for (int numberOfScan = 15; numberOfScan < 100; numberOfScan++) {
//        std::cout << "current KeyFrame: " << numberOfScan << std::endl;
    pcl::PointCloud<pcl::PointXYZ> scan1;
    pcl::PointCloud<pcl::PointXYZ> scan2;
    pcl::PointCloud<pcl::PointXYZ> final;
    pcl::io::loadPCDFile(
            all_args[0],
            scan1);
    pcl::io::loadPCDFile(
            all_args[1],
            scan2);
    double fitnessX,fitnessY;
    Eigen::Matrix4d initialGuess = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d m(generalHelpfulTools::getQuaternionFromRPY(0,0,initialGuessAngleYaw));
    initialGuess.block<3, 3>(0, 0) = m;
    Eigen::Matrix4d estimatedTransformation;
//    std::vector<double> numberOfTimeItTakes;
//    for(int i =0 ; i<80;i++){


//    pcl::io::loadPCDFile(
//            "/home/tim-linux/dataFolder/gazeboCorrectedEvenAnglesPCLs_2_75/pclKeyFrame"+ std::to_string(i)+".pcd",
//            *scan1);
//    pcl::io::loadPCDFile(
//            "/home/tim-linux/dataFolder/gazeboCorrectedEvenAnglesPCLs_2_75/pclKeyFrame"+ std::to_string(i+1)+".pcd",
//            *scan2);
//    double fitnessX,fitnessY;
//    Eigen::Matrix4d initialGuess = Eigen::Matrix4d::Identity();
//    Eigen::Matrix3d m(generalHelpfulTools::getQuaternionFromRPY(0,0,initialGuessAngleYaw));
//    initialGuess.block<3, 3>(0, 0) = m;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//    estimatedTransformation = scanRegistrationObject.generalizedIcpRegistration(scan1,scan2,final,fitnessY,initialGuess);
//    estimatedTransformation = scanRegistrationObject.ndt_p2d(scan1,scan2,initialGuess,true);
//    estimatedTransformation = scanRegistrationObject.ndt_d2d_2d(scan1,scan2,initialGuess,true);
//    estimatedTransformation = scanRegistrationObject.super4PCSRegistration(scan1,scan2,initialGuess,false,false);
    estimatedTransformation = scanRegistrationObject.gmmRegistrationD2D(scan1,scan2,initialGuess,true);
//    estimatedTransformation = scanRegistrationObject.gmmRegistrationP2D(scan1,scan2,initialGuess,true);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//
//    std::cout << "Time difference complete Registration = "
//              << std::chrono::duration_cast<std::chrono::millisecond  s>(end - begin).count()
//              << "[ms]" << std::endl;
//        numberOfTimeItTakes.push_back((double)(std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()));
//    }


    std::cout << "registration with ICP done" << std::endl;
    std::cout << estimatedTransformation << std::endl;
    std::cout << "registration with ICP done NEXT" << std::endl;
    std::cout << estimatedTransformation.inverse() << std::endl;


    //saving resulting PCL
    pcl::io::savePCDFileASCII("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resulting0PCL1.pcd",
                              scan1);
    pcl::io::savePCDFileASCII("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resulting0PCL2.pcd",
                              final);

    std::ofstream myFile11;
    myFile11.open(
            "/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultingTransformation0.csv");

    Eigen::Quaterniond quatTMP(estimatedTransformation.block<3, 3>(0, 0));
    Eigen::Vector3d rpyTMP = generalHelpfulTools::getRollPitchYaw(quatTMP);

    myFile11 << estimatedTransformation(0, 3);
    myFile11 << "\n";
    myFile11 << estimatedTransformation(1, 3);
    myFile11 << "\n";
    myFile11 << estimatedTransformation(2, 3);
    myFile11 << "\n";

    myFile11 << rpyTMP.x();
    myFile11 << "\n";
    myFile11 << rpyTMP.y();
    myFile11 << "\n";
    myFile11 << rpyTMP.z();
    myFile11 << "\n";
    myFile11.close();


    //scanRegistrationObject.sofftRegistration(*scan1,*scan2,fitnessX,fitnessY,std::atan2(estimatedTransformation(1, 0), estimatedTransformation(0, 0)),true);

//        printf("Press [Enter] key to continue.\n");

//        while(getchar()!='\n'); // option TWO to clean stdin
//        getchar(); // wait for ENTER


//    }
//    double sum = std::accumulate(numberOfTimeItTakes.begin(), numberOfTimeItTakes.end(), 0.0);
//    double mean = sum / numberOfTimeItTakes.size();
//
//    std::vector<double> diff(numberOfTimeItTakes.size());
//    std::transform(numberOfTimeItTakes.begin(), numberOfTimeItTakes.end(), diff.begin(),
//                   std::bind2nd(std::minus<double>(), mean));
//    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
//    double stdev = std::sqrt(sq_sum / numberOfTimeItTakes.size());
//
//
//    std::cout << "mean: " << mean << std::endl;
//    std::cout << "stdev: " << stdev << std::endl;

    return (0);
}
