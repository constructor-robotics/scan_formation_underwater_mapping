cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CATKIN_ENABLE_TESTING OFF)



project(scan_forming_underwater_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages cv_bridge PATHS "/usr/local/"
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg roscpp ping360_sonar waterlinked_dvl message_generation commonbluerovmsg dynamic_reconfigure roslib)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 4.7 REQUIRED )
find_package(OpenGR REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})


catkin_package()

################################################################################
# 2D Peak Algorithm
################################################################################

add_library(findpeaks INTERFACE)
target_include_directories(findpeaks INTERFACE
        src/find-peaks/include
        )
add_library(UNION_FIND
        SHARED
        src/find-peaks/src/union_find.cpp
        )
target_include_directories(UNION_FIND PRIVATE
        src/find-peaks/include
        )
target_link_libraries(findpeaks INTERFACE UNION_FIND)

################################################################################
# general interface
################################################################################
add_library(generalinterface INTERFACE)
target_include_directories(generalinterface INTERFACE src/generalInterface/include)

################################################################################
# soft20, the registration FMS
################################################################################
add_library(soft20 src/soft20/src/lib1/rotate_so3_fftw.c
        src/soft20/src/lib1/rotate_so3_utils.c
        src/soft20/src/lib1/s2_cospmls.c
        src/soft20/src/lib1/s2_legendreTransforms.c
        src/soft20/src/lib1/s2_semi_fly.c
        src/soft20/src/lib1/s2_semi_memo.c
        src/soft20/src/lib1/so3_correlate_fftw.c
        src/soft20/src/lib1/soft_fftw.c
        src/soft20/src/lib1/soft_fftw_nt.c
        src/soft20/src/lib1/soft_fftw_pc.c
        src/soft20/src/lib1/utils_vec_cx.c
        src/soft20/src/lib1/wignerTransforms_fftw.c
        src/soft20/src/lib1/wrap_s2_rotate_fftw.c
        src/soft20/src/lib1/wrap_soft_fftw.c
        src/soft20/src/lib1/wrap_soft_fftw_cor2.c

        src/soft20/src/common/csecond.c
        src/soft20/src/common/makeweights.c
        src/soft20/src/common/makeWigner.c
        src/soft20/src/common/s2_primitive.c
        src/soft20/src/common/utils_so3.c
        src/soft20/src/common/wignerTransforms.c)

target_link_libraries(soft20 fftw3)
install(
        TARGETS soft20
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY src/soft20/include/soft20/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

target_include_directories( soft20 PUBLIC src/soft20/include)



################################################################################
# generalhelpfullib
################################################################################
add_library(generalhelpfullib SHARED src/generalHelpfulTools/src/generalHelpfulTools.cpp src/generalHelpfulTools/src/PeakFinder.cpp)
target_include_directories(generalhelpfullib PUBLIC ${catkin_INCLUDE_DIRS} src/generalHelpfulTools/include)


################################################################################
# posegraph
################################################################################

add_library(posegraph SHARED src/poseGraph/src/edge.cpp src/poseGraph/src/graphSlamSaveStructure.cpp src/poseGraph/src/vertex.cpp)
target_link_libraries(posegraph PUBLIC generalinterface generalhelpfullib ${Eigen_LIBRARIES})
target_include_directories(posegraph PUBLIC ${catkin_INCLUDE_DIRS} src/poseGraph/include)


################################################################################
# scan Registration
################################################################################

add_library(registrationlib SHARED src/registration/src/scanRegistrationClass.cpp src/registration/src/sofftCorrelationClass.cpp src/registration/src/softDescriptorRegistration.cpp)
target_link_libraries(registrationlib PUBLIC  ${Eigen_LIBRARIES} ${OpenCV_LIBS} generalhelpfullib gr::algo findpeaks  gmm_registration soft20)
target_include_directories(registrationlib PUBLIC src/registration/include ${catkin_INCLUDE_DIRS} )


################################################################################
# slamtools
################################################################################

add_library(slamtools SHARED src/slamTools/src/slamToolsRos.cpp)
target_link_libraries(slamtools PUBLIC  ${Eigen_LIBRARIES} posegraph registrationlib)
target_include_directories(slamtools PUBLIC ${catkin_INCLUDE_DIRS}  src/slamTools/include)


################################################################################
# Executables
################################################################################

add_executable(rosslamexp1 src/exampleSlamRun.cpp)
target_link_libraries(rosslamexp1 gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3  ${catkin_LIBRARIES} ${OpenCV_LIBS})









