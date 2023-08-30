//
// Created by tim-linux on 18.03.22.
//
#include <vector>
#define POINT_CLOUD_SAVED 0
#define INTEGRATED_POSE 1
#define FIRST_ENTRY 2
#define INTENSITY_SAVED 3
#define INTENSITY_BASED_GRAPH 4
#define POINT_CLOUD_BASED_GRAPH 5
#define ONLY_SIMPLE_GRAPH 6
#define INTENSITY_SAVED_AND_KEYFRAME 7
#define LOOP_CLOSURE 8

struct intensityMeasurement {
    double time;
    double angle;//in rad
    double increment;//step size of angle;
    double range;
    std::vector<double> intensities;
};




