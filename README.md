# scan_formation_underwater_mapping
ROS Package for paper: Synthetic Scan Formation for Underwater Mapping with Low-Cost Mechanical Scanning Sonars (MSS)
Currently under Construction(nearly done, already working)

Needs the following packages to be installed:
The messages for the example Bag File:
1. `https://github.com/Zarbokk/commonBlueROVMSG.git`
2. `https://github.com/GSO-soslab/waterlinked_dvl_ros`

Special Librarys necessary:
1. GTsam `https://github.com/borglab/gtsam.git`
2. OpenCV 4.7 `https://github.com/opencv/opencv.git`

All the other Libraries are either standard(Eigen, ROS, OpenGT etc.), or inside this package.(FMS registration, Peak detection)
An Example Bag, where the SLAM can be performed on is at included in this package. Look in CMakeLists.txt for other libraries if something is missing.

A Service is up for saving the current MAP + Graph. The String in the service is not used. The map is saved in the rosbag folder.



