// PeakFinder.h
#ifndef PEAKFINDER_H
#define PEAKFINDER_H

#include <vector>

struct indexPeak {
    int x;
    int y;
    double height;
    bool highestPeak;
};

namespace PeakFinder {
    const float EPS = 2.2204e-16f;

    /*
        Inputs
        x0: input signal        
        extrema: 1 if maxima are desired, -1 if minima are desired
        includeEndpoints - If true the endpoints will be included as possible extrema otherwise they will not be included
        Output
        peakInds: Indices of peaks in x0
    */
    void findPeaks(std::vector<float> x0, std::vector<int> &peakInds, bool includeEndpoints = true, float impactOfNoise=4.0, float extrema = 1);

    void findPeaks2D(double x0[], std::vector<indexPeak> &potentialPeaks, int dimSize);
}

#endif
