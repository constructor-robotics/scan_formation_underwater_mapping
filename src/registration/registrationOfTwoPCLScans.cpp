//
// Created by tim-linux on 26.03.22.
//

//
// Created by jurobotics on 13.09.21.
//


#include "generalHelpfulTools.h"
#include "slamToolsRos.h"
//#include "scanRegistrationClass.h"

int main(int argc, char **argv) {

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



    scanRegistrationClass scanRegistrationObject;

    pcl::PointCloud<pcl::PointXYZ> scan1;
    pcl::PointCloud<pcl::PointXYZ> scan2;


    pcl::io::loadPCDFile(
            all_args[0],
            scan1);
    pcl::io::loadPCDFile(
            all_args[1],
            scan2);

    double fitnessX,fitnessY;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    Eigen::Matrix4d estimatedTransformation = scanRegistrationObject.sofftRegistration2D(scan1, scan2, fitnessX,
                                                                                         fitnessY, -100, true);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();







    return (0);
}
