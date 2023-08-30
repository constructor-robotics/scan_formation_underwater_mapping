//
// Created by tim-external on 01.03.22.
//

#include "softDescriptorRegistration.h"

bool compareTwoAngleCorrelation(rotationPeak i1, rotationPeak i2) {
    return (i1.angle < i2.angle);
}

std::vector<double> linspace(double start_in, double end_in, int num_in) {
    if (num_in < 0) {
        std::cout << "number of linspace negative" << std::endl;
        exit(-1);
    }
    std::vector<double> linspaced;

    double start = start_in;
    double end = end_in;
    auto num = (double) num_in;

    if (num == 0) { return linspaced; }
    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);//stepSize

    for (int i = 0; i < num - 1; ++i) {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
    // are exactly the same as the input
    return linspaced;
}

bool compareTwoPeaks(indexPeak i1, indexPeak i2) {
    return (i1.height > i2.height);
}

double thetaIncrement(double index, int bandwidth) {
    return M_PI * (1 * index + 0) / (2.0 * bandwidth);
}

double phiIncrement(double index, int bandwidth) {
    return M_PI * index / bandwidth;
}

double angleDifference(double angle1, double angle2) {//gives angle 1 - angle 2
    return atan2(sin(angle1 - angle2), cos(angle1 - angle2));
}

double
softDescriptorRegistration::getSpectrumFromVoxelData2D(double voxelData[], double magnitude[], double phase[],
                                                       bool gaussianBlur) {

    double *voxelDataTMP;
    voxelDataTMP = (double *) malloc(sizeof(double) * N * N);
    for (int i = 0; i < this->N * this->N; i++) {
        voxelDataTMP[i] = voxelData[i];
    }
    if (gaussianBlur) {
        cv::Mat magTMP1(this->N, this->N, CV_64F, voxelDataTMP);
        //add gaussian blur
        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
//        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
//        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
    }



    //from voxel data to row and input for fftw
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < N; i++) {
            inputSpacialData[j + N * i][0] = voxelDataTMP[j + N * i]; // real part
            inputSpacialData[j + N * i][1] = 0; // imaginary part
        }
    }

    fftw_execute(planVoxelToFourier2D);

    //calc magnitude and phase
    double maximumMagnitude = 0;

    //get magnitude and find maximum
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < N; i++) {
            magnitude[j + N * i] = sqrt(
                    spectrumOut[j + N * i][0] *
                    spectrumOut[j + N * i][0] +
                    spectrumOut[j + N * i][1] *
                    spectrumOut[j + N * i][1]); // real part;
            if (maximumMagnitude < magnitude[j + N * i]) {
                maximumMagnitude = magnitude[j + N * i];
            }

            phase[j + N * i] = atan2(spectrumOut[j + N * i][1], spectrumOut[j + N * i][0]);

        }
    }

    free(voxelDataTMP);
    return maximumMagnitude;
}

double
softDescriptorRegistration::getSpectrumFromVoxelData2DCorrelation(double voxelData[], double magnitude[],
                                                                  double phase[],
                                                                  bool gaussianBlur, double normalizationFactor) {

    double *voxelDataTMP;
    voxelDataTMP = (double *) malloc(sizeof(double) * correlationN * correlationN);
    for (int i = 0; i < this->correlationN * this->correlationN; i++) {
        voxelDataTMP[i] = voxelData[i];
    }
    if (gaussianBlur) {
        cv::Mat magTMP1(this->correlationN, this->correlationN, CV_64F, voxelData);
        //add gaussian blur
        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
//        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
//        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
    }

    for (int i = 0; i < this->correlationN; i++) {
        inputSpacialDataCorrelation[i][0] = 0;
        inputSpacialDataCorrelation[i][1] = 0;
    }


    //from voxel data to row and input for fftw
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < N; i++) {
            inputSpacialDataCorrelation[(j + (int) (this->correlationN / 4)) +
                                        this->correlationN * (i + (int) (this->correlationN / 4))][0] =
                    normalizationFactor * voxelData[j +
                                                    N *
                                                    i]; // real part
//            inputSpacialDataCorrelation[j + N * i][1] = 0; // imaginary part
        }
    }
//    for (int j = 0; j < this->correlationN*this->correlationN; j++) {
//        std::cout << inputSpacialDataCorrelation[j][0] << std::endl;
//    }
    fftw_execute(planVoxelToFourier2DCorrelation);

    //calc magnitude and phase
    double maximumMagnitude = 0;

    //get magnitude and find maximum
    for (int j = 0; j < this->correlationN; j++) {
        for (int i = 0; i < this->correlationN; i++) {
            magnitude[j + this->correlationN * i] = sqrt(
                    spectrumOutCorrelation[j + this->correlationN * i][0] *
                    spectrumOutCorrelation[j + this->correlationN * i][0] +
                    spectrumOutCorrelation[j + this->correlationN * i][1] *
                    spectrumOutCorrelation[j + this->correlationN * i][1]); // real part;
            if (maximumMagnitude < magnitude[j + this->correlationN * i]) {
                maximumMagnitude = magnitude[j + this->correlationN * i];
            }

            phase[j + this->correlationN * i] = atan2(spectrumOutCorrelation[j + this->correlationN * i][1],
                                                      spectrumOutCorrelation[j + this->correlationN * i][0]);

        }
    }

    free(voxelDataTMP);
    return maximumMagnitude;
}


double
softDescriptorRegistration::sofftRegistrationVoxel2DRotationOnly(double voxelData1Input[], double voxelData2Input[],
                                                                 double goodGuessAlpha,double &covariance, bool debug) {

    std::vector<rotationPeak> allAnglesList = this->sofftRegistrationVoxel2DListOfPossibleRotations(voxelData1Input,
                                                                                                    voxelData2Input,
                                                                                                    debug);

    int indexCorrectAngle = 0;
    for (int i = 1; i < allAnglesList.size(); i++) {
        if (std::abs(angleDifference(allAnglesList[indexCorrectAngle].angle, goodGuessAlpha)) >
            std::abs(angleDifference(allAnglesList[i].angle, goodGuessAlpha))) {
            indexCorrectAngle = i;
        }
    }
    covariance = allAnglesList[indexCorrectAngle].covariance;
    return allAnglesList[indexCorrectAngle].angle;//this angle is from Pos1 to Pos 2
}

std::vector<rotationPeak>
softDescriptorRegistration::sofftRegistrationVoxel2DListOfPossibleRotations(double voxelData1Input[],
                                                                            double voxelData2Input[], bool debug,
                                                                            bool multipleRadii, bool useClahe,
                                                                            bool useHamming) {

    double maximumScan1Magnitude = this->getSpectrumFromVoxelData2D(voxelData1Input, this->magnitude1,
                                                                    this->phase1, false);
    double maximumScan2Magnitude = this->getSpectrumFromVoxelData2D(voxelData2Input, this->magnitude2,
                                                                    this->phase2, false);


    if (debug) {
        std::ofstream myFile1, myFile2, myFile3, myFile4, myFile5, myFile6;
        myFile1.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/magnitudeFFTW1.csv");
        myFile2.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/phaseFFTW1.csv");
        myFile3.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/voxelDataFFTW1.csv");
        myFile4.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/magnitudeFFTW2.csv");
        myFile5.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/phaseFFTW2.csv");
        myFile6.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/voxelDataFFTW2.csv");
        for (int j = 0; j < N; j++) {
            for (int i = 0; i < N; i++) {
                myFile1 << magnitude1[j + N * i]; // real part
                myFile1 << "\n";
                myFile2 << phase1[j + N * i]; // imaginary part
                myFile2 << "\n";
                myFile3 << voxelData1Input[j + N * i]; // imaginary part
                myFile3 << "\n";
                myFile4 << magnitude2[j + N * i]; // real part
                myFile4 << "\n";
                myFile5 << phase2[j + N * i]; // imaginary part
                myFile5 << "\n";
                myFile6 << voxelData2Input[j + N * i]; // imaginary part
                myFile6 << "\n";
            }
        }

        myFile1.close();
        myFile2.close();
        myFile3.close();
        myFile4.close();
        myFile5.close();
        myFile6.close();
    }

    double globalMaximumMagnitude;
    if (maximumScan2Magnitude < maximumScan1Magnitude) {
        globalMaximumMagnitude = maximumScan1Magnitude;
    } else {
        globalMaximumMagnitude = maximumScan2Magnitude;
    }

    //normalize and fftshift
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < N; i++) {
            int indexX = (N / 2 + i) % N;
            int indexY = (N / 2 + j) % N;

//            int indexX = i;
//            int indexY = j;

            magnitude1Shifted[indexY + N * indexX] =
                    magnitude1[j + N * i] / globalMaximumMagnitude;
            magnitude2Shifted[indexY + N * indexX] =
                    magnitude2[j + N * i] / globalMaximumMagnitude;
        }
    }


    //re-initialize to zero
    for (int i = 0; i < N * N; i++) {
        resampledMagnitudeSO3_1[i] = 0;
        resampledMagnitudeSO3_2[i] = 0;
        resampledMagnitudeSO3_1TMP[i] = 0;
        resampledMagnitudeSO3_2TMP[i] = 0;
    }

    int minRNumber = 1 + floor(N * 0.05);//was 4
    int maxRNumber = N / 2 - floor(N * 0.05);
    int bandwidth = N / 2;

    if(multipleRadii){
        minRNumber = maxRNumber - 1;
    }
    //CHANGE HERE HAPPEND TESTS
    for (int r = minRNumber; r < maxRNumber; r++) {
//    for (int r = minRNumber; r < maxRNumber; r++) {
        for (int j = 0; j < 2 * bandwidth; j++) {
            for (int k = 0; k < 2 * bandwidth; k++) {
                int xIndex = std::round((double) r * std::sin(thetaIncrement((double) j, bandwidth)) *
                                        std::cos(phiIncrement((double) k, bandwidth)) + bandwidth) - 1;
                int yIndex = std::round((double) r * std::sin(thetaIncrement((double) j, bandwidth)) *
                                        std::sin(phiIncrement((double) k, bandwidth)) + bandwidth) - 1;
//                int zIndex =
//                        std::round((double) r * std::cos(thetaIncrement((double) j + 1, bandwidth)) + bandwidth) - 1;
//                double hammingCoeff = 25.0/46.0-(1.0-25.0/46.0)*cos(2*M_PI*k/(2*bandwidth));
                double hammingCoeff = 1;
                resampledMagnitudeSO3_1TMP[k + j * bandwidth * 2] =
                        255 * magnitude1Shifted[yIndex + N * xIndex] * hammingCoeff;
                resampledMagnitudeSO3_2TMP[k + j * bandwidth * 2] =
                        255 * magnitude2Shifted[yIndex + N * xIndex] * hammingCoeff;
            }
        }
//        int removeLines = 20;
//        for (int j = 0; j < 2 * bandwidth; j++) {
//            for (int k = 0; k < 2 * bandwidth; k++) {
//                if(j<removeLines || j > 2 * bandwidth-removeLines){
//                    resampledMagnitudeSO3_1TMP[k + j * bandwidth * 2] = 0;
//                    resampledMagnitudeSO3_2TMP[k + j * bandwidth * 2] = 0;
//                }
//            }
//        }

//        for (int j = 0; j < 2 * bandwidth; j++) {
//            for (int k = 0; k < 2 * bandwidth; k++) {
////                double hammingCoeff = 1;
//                double hammingCoeff = 25.0 / 46.0 - (1.0 - 25.0 / 46.0) * cos(2 * M_PI * k / (2 * bandwidth));
//
//                resampledMagnitudeSO3_1TMP[j + k * bandwidth * 2] = resampledMagnitudeSO3_1TMP[j + k * bandwidth * 2] * hammingCoeff;
//                resampledMagnitudeSO3_2TMP[j + k * bandwidth * 2] = resampledMagnitudeSO3_2TMP[j + k * bandwidth * 2] * hammingCoeff;
//            }
//        }



        cv::Mat magTMP1(N, N, CV_64FC1, resampledMagnitudeSO3_1TMP);
        cv::Mat magTMP2(N, N, CV_64FC1, resampledMagnitudeSO3_2TMP);
        magTMP1.convertTo(magTMP1, CV_8UC1);
        magTMP2.convertTo(magTMP2, CV_8UC1);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(3);
        if (useClahe){
            clahe->apply(magTMP1, magTMP1);
            clahe->apply(magTMP2, magTMP2);
        }



        for (int j = 0; j < 2 * bandwidth; j++) {
            for (int k = 0; k < 2 * bandwidth; k++) {
//                double hammingCoeff = 1;

                double hammingCoeff = 1;
                if(useHamming){
                    hammingCoeff = 25.0 / 46.0 - (1.0 - 25.0 / 46.0) * cos(2 * M_PI * k / (2 * bandwidth));
                }

                resampledMagnitudeSO3_1[j + k * bandwidth * 2] = resampledMagnitudeSO3_1[j + k * bandwidth * 2] +
                                                                 ((double) magTMP1.data[j + k * bandwidth * 2]) /
                                                                 255.0 * hammingCoeff;
                resampledMagnitudeSO3_2[j + k * bandwidth * 2] = resampledMagnitudeSO3_2[j + k * bandwidth * 2] +
                                                                 ((double) magTMP2.data[j + k * bandwidth * 2]) /
                                                                 255.0 * hammingCoeff;
            }
        }
//        std::cout << resampledMagnitudeSO3_1[100 + 100 * bandwidth * 2] << std::endl;
//        std::cout << resampledMagnitudeSO3_1[100 + 100 * bandwidth * 2] << std::endl;
    }


    if (debug) {
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
    }

    //use soft descriptor to calculate the correlation
    this->sofftCorrelationObject.correlationOfTwoSignalsInSO3(resampledMagnitudeSO3_1, resampledMagnitudeSO3_2,
                                                              resultingCorrelationComplex);
    if (debug) {
        FILE *fp;
        fp = fopen(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultCorrelation3D.csv",
                "w");
        for (int i = 0; i < 8 * bwOut * bwOut * bwOut; i++)
            fprintf(fp, "%.16f\n", resultingCorrelationComplex[i][0]);
        fclose(fp);
    }


    if (debug) {
        double minimumCorrelation = INFINITY;
        double maximumCorrelation = 0;
        for (int i = 0; i < 8 * bwOut * bwOut * bwOut; i++) {
            if (minimumCorrelation > resultingCorrelationComplex[i][0]) {
                minimumCorrelation = resultingCorrelationComplex[i][0];
            }
            if (maximumCorrelation < resultingCorrelationComplex[i][0]) {
                maximumCorrelation = resultingCorrelationComplex[i][0];
            }
        }


        double correlationCurrent;
        long N_Long = N / 2;
        double *quaternionCorrelation = (double *) malloc(sizeof(double) * N_Long * N_Long * N_Long);
        int *quaternionCorrelationINT = (int *) malloc(sizeof(int) * N_Long * N_Long * N_Long);


        for (int i = 0; i < N_Long * N_Long * N_Long; i++) {
            quaternionCorrelation[i] = 0;
            quaternionCorrelationINT[i] = 0;
        }

        for (int j = 0; j < N; j++) {
            for (int i = 0; i < N; i++) {
                for (int k = 0; k < N; k++) {
                    correlationCurrent = (resultingCorrelationComplex[j + N * (i + N * k)][0] - minimumCorrelation) /
                                         (maximumCorrelation - minimumCorrelation);
                    Eigen::AngleAxisd rotation_vectorz1(j * 2 * 3.14159 / N, Eigen::Vector3d(0, 0, 1));
                    Eigen::AngleAxisd rotation_vectory(k * 3.14159 / N, Eigen::Vector3d(0, 1, 0));
                    Eigen::AngleAxisd rotation_vectorz2(i * 2 * 3.14159 / N, Eigen::Vector3d(0, 0, 1));


                    Eigen::Matrix3d tmpMatrix3d =
                            rotation_vectorz1.toRotationMatrix() * rotation_vectory.toRotationMatrix() *
                            rotation_vectorz2.toRotationMatrix();
                    Eigen::Quaterniond quaternionResult(tmpMatrix3d);
                    quaternionResult.normalize();
                    if (quaternionResult.w() < 0) {
                        Eigen::Quaterniond tmpQuad = quaternionResult;
                        quaternionResult.w() = -tmpQuad.w();
                        quaternionResult.x() = -tmpQuad.x();
                        quaternionResult.y() = -tmpQuad.y();
                        quaternionResult.z() = -tmpQuad.z();
                    }
//                    std::cout << quaternionResult.x() << " " << quaternionResult.y() <<" " << quaternionResult.z() <<" " << quaternionResult.w() << std::endl;
                    long xx = (long) ((quaternionResult.x() + 1) / 2.0 * (N_Long - 1));
                    long yy = (long) ((quaternionResult.y() + 1) / 2.0 * (N_Long - 1));
                    long zz = (long) ((quaternionResult.z() + 1) / 2.0 * (N_Long - 1));
                    long ww = (long) ((quaternionResult.w()) * (N_Long - 1));
//                    std::cout << xx << " " << yy <<" " << zz <<" " << ww << std::endl;
//                    std::cout << correlationCurrent << std::endl;
                    if (N_Long * N_Long * N_Long < xx + N_Long * (yy + N_Long * (zz))) {
                        std::cout << xx + N_Long * (yy + N_Long * (zz)) << std::endl;
                        std::cout << N_Long * N_Long * N_Long << std::endl;
                    }
                    quaternionCorrelation[xx + N_Long * (yy + N_Long * (zz))] += correlationCurrent * 10000;
                    quaternionCorrelationINT[xx + N_Long * (yy + N_Long * (zz))] += 1;
                }
            }
        }

        for (int i = 0; i < N_Long * N_Long * N_Long; i++) {
            if (quaternionCorrelationINT[i] > 0) {
                quaternionCorrelation[i] = quaternionCorrelation[i] / quaternionCorrelationINT[i];
            }
        }


        FILE *fp;
        fp = fopen(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultCorrelation4D.csv",
                "w");
        for (int i = 0; i < N_Long * N_Long * N_Long; i++)
            fprintf(fp, "%.16f\n", quaternionCorrelation[i]);
        fclose(fp);
        free(quaternionCorrelation);
        free(quaternionCorrelationINT);
    }



    //calcs the rotation angle around z axis for 2D scans
    double z1;
    double z2;
    double maxCorrelation = 0;
    std::vector<rotationPeak> correlationOfAngle;
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < N; i++) {
            z1 = j * 2.0 * M_PI / N;
            z2 = i * 2.0 * M_PI / N;
            //[i + N * j]
            rotationPeak tmpHolding;
            tmpHolding.peakCorrelation = resultingCorrelationComplex[j + N * (i + N * 0)][0]; // real part
            if (tmpHolding.peakCorrelation > maxCorrelation) {
                maxCorrelation = tmpHolding.peakCorrelation;
            }
            // test on dataset with N and N/2 and 0   first test + n/2
            tmpHolding.angle = std::fmod(-(z1 + z2) + 6 * M_PI + 0.0 * M_PI / (N),
                                         2 * M_PI);
            correlationOfAngle.push_back(tmpHolding);
        }
    }

    std::sort(correlationOfAngle.begin(), correlationOfAngle.end(), compareTwoAngleCorrelation);

    std::vector<float> correlationAveraged, angleList;
    float maximumCorrelation = 0;
    float minimumCorrelation = INFINITY;
    double currentAverageAngle = correlationOfAngle[0].angle;
    //angleList.push_back(currentAverageAngle);
    int numberOfAngles = 1;
    double averageCorrelation = correlationOfAngle[0].peakCorrelation;
    for (int i = 1; i < correlationOfAngle.size(); i++) {

        if (std::abs(currentAverageAngle - correlationOfAngle[i].angle) < 1.0 / N / 4.0) {
            numberOfAngles = numberOfAngles + 1;
            averageCorrelation = averageCorrelation + correlationOfAngle[i].peakCorrelation;
        } else {

            correlationAveraged.push_back((float) (averageCorrelation / numberOfAngles));
            angleList.push_back((float) currentAverageAngle);
            numberOfAngles = 1;
            averageCorrelation = correlationOfAngle[i].peakCorrelation;
            currentAverageAngle = correlationOfAngle[i].angle;
            if (minimumCorrelation > correlationAveraged.back()) {
                minimumCorrelation = correlationAveraged.back();
            }
            if (maximumCorrelation < correlationAveraged.back()) {
                maximumCorrelation = correlationAveraged.back();
            }

        }
    }
    correlationAveraged.push_back((float) (averageCorrelation / numberOfAngles));
    if (minimumCorrelation > correlationAveraged.back()) {
        minimumCorrelation = correlationAveraged.back();
    }
    if (maximumCorrelation < correlationAveraged.back()) {
        maximumCorrelation = correlationAveraged.back();
    }

    for (int i = 0; i < correlationAveraged.size(); i++) {
        correlationAveraged[i] =
                (correlationAveraged[i] - minimumCorrelation) / (maximumCorrelation - minimumCorrelation);
    }

    angleList.push_back((float) currentAverageAngle);
    if (debug) {
        std::ofstream myFile9;
        myFile9.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultingCorrelation1D.csv");

        for (int i = 0; i < correlationAveraged.size(); i++) {
            myFile9 << correlationAveraged[i]; // real part
            myFile9 << "\n";

        }
        myFile9.close();
    }

    auto minmax = std::min_element(correlationAveraged.begin(), correlationAveraged.end());
    long distanceToMinElement = std::distance(correlationAveraged.begin(), minmax);
    std::rotate(correlationAveraged.begin(), correlationAveraged.begin() + distanceToMinElement,
                correlationAveraged.end());

    std::vector<int> out;

    PeakFinder::findPeaks(correlationAveraged, out, true, 8.0);//was 4.0


//
//    size_t ourSize = correlationAveraged.size();
//    double *tmpDoubleArray = (double *) malloc(sizeof(double) * ourSize);
//    for(int i = 0 ; i<ourSize;i++){
//        tmpDoubleArray[i] = correlationAveraged[i];
//    }
//    findpeaks::image_t<double> image = {
//            ourSize, 1,
//            tmpDoubleArray
//    };
//
//    std::vector<findpeaks::peak_t<double>> peaks = findpeaks::persistance(image);
//
//    free(tmpDoubleArray);


    std::rotate(correlationAveraged.begin(),
                correlationAveraged.begin() + correlationAveraged.size() - distanceToMinElement,
                correlationAveraged.end());
    for (int i = 0; i < out.size(); ++i) {
        out[i] = out[i] + (int) distanceToMinElement;
        if (out[i] >= correlationAveraged.size()) {
            out[i] = out[i] - correlationAveraged.size();
        }
    }

    std::vector<rotationPeak> returnVectorWithPotentialAngles;

    for (int i = 0; i < out.size(); i++) {
        rotationPeak tmpPeak{};
        tmpPeak.angle = angleList[out[i]];
        tmpPeak.peakCorrelation = correlationAveraged[out[i]];
        tmpPeak.covariance = 0.05;
        returnVectorWithPotentialAngles.push_back(tmpPeak);
    }

    return returnVectorWithPotentialAngles;
}


//Eigen::Vector2d softDescriptorRegistration::sofftRegistrationVoxel2DTranslation(double voxelData1Input[],
//                                                                                double voxelData2Input[],
//                                                                                double &fitnessX, double &fitnessY,
//                                                                                double cellSize,
//                                                                                Eigen::Vector3d initialGuess,
//                                                                                bool useInitialGuess,
//                                                                                double &heightMaximumPeak, bool debug) {
//
//    //std::vector<double> xShiftList, yShiftList, heightPeakList, estimatedAngleList, heightPeakAngleList;
//
//    //i have to inverse initialGuess ( i think it is because the cross correlation gives translation from 2 -> 1 not from 1 ->2)
//    initialGuess = -initialGuess;
//    // create padding in translation voxelData
//
//
//
//
//    double maximumScan1 = this->getSpectrumFromVoxelData2DCorrelation(voxelData1Input, this->magnitude1Correlation,
//                                                                      this->phase1Correlation, false, 1);
//
////    if (debug) {
////        std::ofstream myFile1, myFile2, myFile3;
////        myFile1.open(
////                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/magnitudeFFTW1.csv");
////        myFile2.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/phaseFFTW1.csv");
////        myFile3.open(
////                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/voxelDataFFTW1.csv");
////
////        for (int j = 0; j < this->correlationN; j++) {
////            for (int i = 0; i < this->correlationN; i++) {
////                myFile1 << magnitude1Correlation[j + this->correlationN * i]; // real part
////                myFile1 << "\n";
////                myFile2 << phase1Correlation[j + this->correlationN * i]; // imaginary part
////                myFile2 << "\n";
////                myFile3 << inputSpacialDataCorrelation[j + this->correlationN * i][0]; // imaginary part
////                myFile3 << "\n";
////            }
////        }
////
////        myFile1.close();
////        myFile2.close();
////        myFile3.close();
////    }
//
//    double maximumScan2 = this->getSpectrumFromVoxelData2DCorrelation(voxelData2Input, this->magnitude2Correlation,
//                                                                      this->phase2Correlation, false, 1);
//
////    if (debug) {
////        std::ofstream myFile4, myFile5, myFile6;
////        myFile4.open(
////                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/magnitudeFFTW2.csv");
////        myFile5.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/phaseFFTW2.csv");
////        myFile6.open(
////                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/voxelDataFFTW2.csv");
////        for (int j = 0; j < this->correlationN; j++) {
////            for (int i = 0; i < this->correlationN; i++) {
////                myFile4 << magnitude2Correlation[j + this->correlationN * i]; // real part
////                myFile4 << "\n";
////                myFile5 << phase1Correlation[j + this->correlationN * i]; // imaginary part
////                myFile5 << "\n";
////                myFile6 << inputSpacialDataCorrelation[j + this->correlationN * i][0]; // real part
////                myFile6 << "\n";
////            }
////        }
////
////
////        myFile4.close();
////        myFile5.close();
////        myFile6.close();
////    }
//
//
//
//    //fftshift and calculate correlation of spectrums
//    for (int j = 0; j < this->correlationN; j++) {
//        for (int i = 0; i < this->correlationN; i++) {
//
////            int indexX = (this->correlationN / 2 + i) % this->correlationN;
////            int indexY = (this->correlationN / 2 + j) % this->correlationN;
//            int indexX = i;
//            int indexY = j;
//            //calculate the spectrum back
//            std::complex<double> tmpComplex1 =
//                    magnitude1Correlation[indexY + this->correlationN * indexX] *
//                    std::exp(std::complex<double>(0, phase1Correlation[indexY + this->correlationN * indexX]));
//            std::complex<double> tmpComplex2 =
//                    magnitude2Correlation[indexY + this->correlationN * indexX] *
//                    std::exp(std::complex<double>(0, phase2Correlation[indexY + this->correlationN * indexX]));
////                std::complex<double> tmpComplex1 = std::exp(std::complex<double>(0, phase1[indexY + N * indexX]));
////                std::complex<double> tmpComplex2 = std::exp(std::complex<double>(0, phase2[indexY + N * indexX]));
////                std::complex<double> tmpComplex;
////                tmpComplex.real(0);
////                tmpComplex.imag(phase1[indexY + N * indexX] - phase2[indexY + N * indexX]);
////                std::complex<double> resultCompexNumber = std::exp(tmpComplex);
////                resultingPhaseDiff2D[j + N * i][0] = resultCompexNumber.real();
////                resultingPhaseDiff2D[j + N * i][1] = resultCompexNumber.imag();
////            std::cout << tmpComplex1 << std::endl;
//            std::complex<double> resultComplex = ((tmpComplex1) * conj(tmpComplex2));
//            resultingPhaseDiff2DCorrelation[j + this->correlationN * i][0] = resultComplex.real();
//            resultingPhaseDiff2DCorrelation[j + this->correlationN * i][1] = resultComplex.imag();
//
//        }
//    }
//
//
//    fftw_execute(planFourierToVoxel2DCorrelation);
//
//
//
//    // fftshift and calc magnitude
//    //double meanCorrelation = 0;
//    int indexMaximumCorrelationI;
//    int indexMaximumCorrelationJ;
//    double maximumCorrelation = 0;
//    for (int j = 0; j < this->correlationN; j++) {
//        for (int i = 0; i < this->correlationN; i++) {
//            int indexX = (this->correlationN / 2 + i + this->correlationN) % this->correlationN;// changed j and i here
//            int indexY = (this->correlationN / 2 + j + this->correlationN) % this->correlationN;
////            int indexX = i;// changed j and i here
////            int indexY = j;
//            resultingCorrelationDouble[indexY + this->correlationN * indexX] = sqrt(
//                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][0] *
//                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][0] +
//                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][1] *
//                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][1]); // magnitude;
//            if (resultingCorrelationDouble[indexY + this->correlationN * indexX] < 10000) {
//                resultingCorrelationDouble[indexY + this->correlationN * indexX] = 0;
//            }
//            //meanCorrelation = meanCorrelation + resultingCorrelationDouble[indexY + N * indexX];
//            if (maximumCorrelation < resultingCorrelationDouble[indexY + this->correlationN * indexX]) {
//                maximumCorrelation = resultingCorrelationDouble[indexY + this->correlationN * indexX];
//                indexMaximumCorrelationI = indexX;
//                indexMaximumCorrelationJ = indexY;
//            }
//
//        }
//    }
//    ////////////////////////////////// HERE COMES THE NEW STUFFFF //////////////////////////////////
//    float impactOfNoise = 4.0;
//    std::vector<std::vector<int>> xPeaks, yPeaks;
//
//    for (int j = 0; j < this->correlationN; j++) {
//        std::vector<float> inputYLine;
//        for (int i = 0; i < this->correlationN; i++) {
//            inputYLine.push_back((float) resultingCorrelationDouble[j + this->correlationN * i]);
//        }
//        std::vector<int> out;
//        PeakFinder::findPeaks(inputYLine, out, false, impactOfNoise);
////        for(int i = 0 ; i < out.size();i++){
////            std::cout << out[i] << std::endl;
////        }
////        std::cout <<"next"<< std::endl;
//        yPeaks.push_back(out);
//    }
//
//    for (int i = 0; i < this->correlationN; i++) {
//        std::vector<float> inputXLine;
//        for (int j = 0; j < this->correlationN; j++) {
//            inputXLine.push_back((float) resultingCorrelationDouble[j + this->correlationN * i]);
//        }
//        std::vector<int> out;
//        PeakFinder::findPeaks(inputXLine, out, false, impactOfNoise);
////        for(int j = 0 ; j < out.size();j++){
////            std::cout << out[j] << std::endl;
////        }
////        std::cout <<"next"<< std::endl;
//        xPeaks.push_back(out);
//
//    }
////    for (int j = 0; j < this->correlationN; j++) {
////        for (int i = 0; i < this->correlationN; i++) {
////            auto iteratorX = std::find(xPeaks[i].begin(), xPeaks[i].end(), j);
////            if( iteratorX != xPeaks[i].end()){
////                auto iteratorY = std::find(yPeaks[j].begin(), yPeaks[j].end(), i);
////                if(iteratorY!= yPeaks[j].end()){
////                    std::cout << "found Peak:" << std::endl;
//////                    std::cout << *iteratorX << std::endl;
//////                    std::cout << *iteratorY  << std::endl;
////                    std::cout << i+1 << std::endl;
////                    std::cout << j+1  << std::endl;
////                }
////            }
////        }
////    }
//
//
////    for (int j = 0; j < this->correlationN*this->correlationN; j++) {
////        resultingCorrelationDouble[j]=sqrt(
////                resultingShiftPeaks2DCorrelation[j][0] *
////                resultingShiftPeaks2DCorrelation[j][0] +
////                resultingShiftPeaks2DCorrelation[j][1] *
////                resultingShiftPeaks2DCorrelation[j][1]);
////    }
//    if (debug) {
//        std::ofstream myFile10;
//        myFile10.open(
//                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultingCorrelationShift.csv");
//
//        for (int j = 0; j < this->correlationN; j++) {
//            for (int i = 0; i < this->correlationN; i++) {
//                myFile10 << resultingCorrelationDouble[j + this->correlationN * i];
//                myFile10 << "\n";
//            }
//        }
//        myFile10.close();
//    }
//
//    //89 131
//    if (useInitialGuess) {
//        //find local maximum in 2d array
//        int initialIndexX = (int) (initialGuess[0] / cellSize + this->correlationN / 2);
//        int initialIndexY = (int) (initialGuess[1] / cellSize + this->correlationN / 2);
//        int localMaxDiffX = 0;
//        int localMaxDiffY = 0;
//        do {
//            localMaxDiffX = 0;
//            localMaxDiffY = 0;
//
//            for (int i = -1; i < 2; i++) {
//                for (int j = -1; j < 2; j++) {
//                    if (resultingCorrelationDouble[(initialIndexY + localMaxDiffY) +
//                                                   this->correlationN * (initialIndexX + localMaxDiffX)] <
//                        resultingCorrelationDouble[(initialIndexY + j) + this->correlationN * (initialIndexX + i)]) {
//                        localMaxDiffX = i;
//                        localMaxDiffY = j;
//                    }
//                }
//            }
//            initialIndexY += localMaxDiffY;
//            initialIndexX += localMaxDiffX;
//        } while (localMaxDiffX != 0 || localMaxDiffY != 0);
//        indexMaximumCorrelationI = initialIndexX;
//        indexMaximumCorrelationJ = initialIndexY;
//    }
//    heightMaximumPeak = resultingCorrelationDouble[indexMaximumCorrelationJ +
//                                                   this->correlationN * indexMaximumCorrelationI];//Hope that is correct
//    // @TODO find SubPixel accuracy
//
//
//
////    std::cout << "estimated indexToStart:" << std::endl;
////    std::cout << indexMaximumCorrelationI<< std::endl;
////    std::cout << indexMaximumCorrelationJ << std::endl;
//    Eigen::Vector3d translationCalculated((indexMaximumCorrelationI - (int) (this->correlationN / 2.0)) * cellSize,
//                                          (indexMaximumCorrelationJ - (int) (this->correlationN / 2.0)) * cellSize, 0);
////    std::cout << "translationCalculated: "<< std::endl;
////    std::cout << translationCalculated << std::endl;
//
//
//    //currently these metrics are not used.
////        std::cout << "current angle: " << currentAngle << std::endl;
////        std::cout << "SNR peak/mean: " << heightPeakList[heightPeakList.size()-1]/meanCorrelation << std::endl;
////        std::cout << "SNR var/mean: " << variance/meanCorrelation << std::endl;
////        std::cout << "height of Peak: " << heightPeakList[heightPeakList.size() - 1] << std::endl;
////        std::cout << "index I: " << indexMaximumCorrelationI << std::endl;
////        std::cout << "index J: " << indexMaximumCorrelationJ << std::endl;
//
//
//
//
//    // calculate
//
//    // x calculation of C
//    double aParam = resultingCorrelationDouble[indexMaximumCorrelationJ +
//                                               this->correlationN * indexMaximumCorrelationI];
//    double bParam = indexMaximumCorrelationI;
//    double cParam = 0;
//    for (int i = 0; i < this->correlationN; i++) {
//        double xTMP = i;
//        double yTMP = resultingCorrelationDouble[indexMaximumCorrelationJ + this->correlationN * i];
//        double cTMP = abs((xTMP - bParam) / (sqrt(-2 * log(yTMP / aParam))));
//        if (xTMP != indexMaximumCorrelationI) {
//            cParam = cParam + cTMP;
//        }
//
//    }
//    fitnessX = cParam / (this->correlationN - 1) * cellSize;
//    //std::cout << "cParam X: " << fitnessX<<std::endl;
//
//
//    //aParam=resultingCorrelationDouble[indexMaximumCorrelationJ + N * indexMaximumCorrelationI];
//    bParam = indexMaximumCorrelationJ;
//    cParam = 0;
//    for (int i = 0; i < this->correlationN; i++) {
//        double xTMP = i;
//        double yTMP = resultingCorrelationDouble[i + this->correlationN * indexMaximumCorrelationI];
//        double cTMP = abs((xTMP - bParam) / (sqrt(-2 * log(yTMP / aParam))));
//        if (xTMP != indexMaximumCorrelationJ) {
//            cParam = cParam + cTMP;
//        }
//
//    }
//    fitnessY = cParam / (this->correlationN - 1) * cellSize;
//
//
//    //translationCalculated = generalHelpfulTools::getQuaternionFromRPY(0,0,M_PI).toRotationMatrix()*translationCalculated;
//    if (!isfinite(fitnessX)) {
//        fitnessX = 10;
//    }
//    if (!isfinite(fitnessY)) {
//        fitnessY = 10;
//    }
//    Eigen::Vector2d returnVector;
//    returnVector[0] = translationCalculated[0];
//    returnVector[1] = translationCalculated[1];
//    return -returnVector;//returning - because we want from 1 to 2 and not the other way around
//
//}

std::vector<translationPeak>
softDescriptorRegistration::sofftRegistrationVoxel2DTranslationAllPossibleSolutions(double voxelData1Input[],
                                                                                    double voxelData2Input[],
                                                                                    double cellSize,
                                                                                    double normalizationFactor,
                                                                                    bool debug,
                                                                                    int numberOfRotationForDebug,
                                                                                    double potentialNecessaryForPeak) {
    //copy and normalize voxelDataInput



    // create padding in translation voxelData
    double maximumScan1 = this->getSpectrumFromVoxelData2DCorrelation(voxelData1Input, this->magnitude1Correlation,
                                                                      this->phase1Correlation, false,
                                                                      normalizationFactor);


    double maximumScan2 = this->getSpectrumFromVoxelData2DCorrelation(voxelData2Input, this->magnitude2Correlation,
                                                                      this->phase2Correlation, false,
                                                                      normalizationFactor);


    //calculate correlation of spectrums
    for (int j = 0; j < this->correlationN; j++) {
        for (int i = 0; i < this->correlationN; i++) {

            int indexX = i;
            int indexY = j;
            //calculate the spectrum back
            std::complex<double> tmpComplex1 =
                    magnitude1Correlation[indexY + this->correlationN * indexX] *
                    std::exp(std::complex<double>(0, phase1Correlation[indexY + this->correlationN * indexX]));
            std::complex<double> tmpComplex2 =
                    magnitude2Correlation[indexY + this->correlationN * indexX] *
                    std::exp(std::complex<double>(0, phase2Correlation[indexY + this->correlationN * indexX]));
            std::complex<double> resultComplex = ((tmpComplex1) * conj(tmpComplex2));
            resultingPhaseDiff2DCorrelation[j + this->correlationN * i][0] = resultComplex.real();
            resultingPhaseDiff2DCorrelation[j + this->correlationN * i][1] = resultComplex.imag();

        }
    }

    // back fft
    fftw_execute(planFourierToVoxel2DCorrelation);


    // normalization Test:

//    std::cout << this->normalizationFactorCalculation(254,254)<< std::endl;
//    std::cout << this->normalizationFactorCalculation(255,255)<< std::endl;
//    std::cout << this->normalizationFactorCalculation(256,256)<< std::endl;
//    std::cout << this->normalizationFactorCalculation(255,255)/this->normalizationFactorCalculation(30,30)<< std::endl;

    // fftshift and calc magnitude
    //double meanCorrelation = 0;
    int indexMaximumCorrelationI;
    int indexMaximumCorrelationJ;
    double maximumCorrelation = 0;
    for (int j = 0; j < this->correlationN; j++) {
        for (int i = 0; i < this->correlationN; i++) {
            int indexX = (this->correlationN / 2 + i + this->correlationN) % this->correlationN;// changed j and i here
            int indexY = (this->correlationN / 2 + j + this->correlationN) % this->correlationN;
//            double normalizationFactorForCorrelation = sqrt(
//                    this->correlationN * this->correlationN / this->normalizationFactorCalculation(indexX, indexY));
            //maybe without sqrt, but for now thats fine
            double normalizationFactorForCorrelation =
                    1 / this->normalizationFactorCalculation(indexX, indexY);
//            double normalizationFactorForCorrelation = 1/this->normalizationFactorCalculation(indexX, indexY);
            normalizationFactorForCorrelation = sqrt(normalizationFactorForCorrelation);
//            normalizationFactorForCorrelation = 1;

//            double normalizationFactorForCorrelation = 25.0 / 46.0 - (1.0 - 25.0 / 46.0) *
//                                                                     cos(2 * M_PI * (i - this->correlationN / 2) *
//                                                                         (j - this->correlationN / 2) /
//                                                                         (pow(this->correlationN / 2, 2)));


            //            int indexX = i;// changed j and i here
//            int indexY = j;
            resultingCorrelationDouble[indexY + this->correlationN * indexX] = normalizationFactorForCorrelation * sqrt(
                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][0] *
                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][0] +
                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][1] *
                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][1]); // magnitude;
//            if(resultingCorrelationDouble[indexY + this->correlationN * indexX] < 10000){
//                resultingCorrelationDouble[indexY + this->correlationN * indexX] = 0;
//            }
            //meanCorrelation = meanCorrelation + resultingCorrelationDouble[indexY + N * indexX];
            if (maximumCorrelation < resultingCorrelationDouble[indexY + this->correlationN * indexX]) {
                maximumCorrelation = resultingCorrelationDouble[indexY + this->correlationN * indexX];
            }
        }
    }
    ////////////////////////////////// HERE COMES THE NEW STUFFFF //////////////////////////////////

    //add gaussian blur
//    if (true) {
//        cv::Mat magTMP1(this->correlationN, this->correlationN, CV_64F, resultingCorrelationDouble);
//        for (int i = 0; i < 5; i++) {
//            cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
//        }
//    }
//    peakDetectionOf2DCorrelationOpenCVHoughTransform(            maximumCorrelation,
//                                                                 cellSize,
//                                                                 registrationNoiseImpactFactor,
//                                                                 ignorePercentageFactor);

    std::vector<translationPeak> potentialTranslations = this->peakDetectionOf2DCorrelationFindPeaksLibrary(cellSize,
                                                                                                            potentialNecessaryForPeak);

    //function of 2D peak detection

//    std::vector<translationPeak> potentialTranslations = this->peakDetectionOf2DCorrelationSimpleDouble1D(
//            maximumCorrelation,
//            cellSize,
//            registrationNoiseImpactFactor,
//            ignorePercentageFactor);


    if (debug) {
        std::ofstream myFile10;
        myFile10.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultingCorrelationShift_" +
                std::to_string(numberOfRotationForDebug) + "_.csv");

        for (int j = 0; j < this->correlationN; j++) {
            for (int i = 0; i < this->correlationN; i++) {
                myFile10 << resultingCorrelationDouble[j + this->correlationN * i];
                myFile10 << "\n";
            }
        }
        myFile10.close();
    }



    // calculate for each maxima a covariance(my algorithm)
//    int definedRadiusVoxel = ceil(this->correlationN / 30);
//    double definedRadiusSI = cellSize * this->correlationN / 30.0;
//    for (auto &potentialTranslation: potentialTranslations) {
//        double resultingIntegral = 0;
//        double maximumIntegral = M_PI * definedRadiusSI * definedRadiusSI * potentialTranslation.peakHeight;
//        for (int i = -definedRadiusVoxel; i < definedRadiusVoxel + 1; i++) {
//            for (int j = -definedRadiusVoxel; j < definedRadiusVoxel + 1; j++) {
//                if (sqrt((double) (i * i + j * j)) *
//                    cellSize < definedRadiusSI) {
//                    resultingIntegral += resultingCorrelationDouble[(potentialTranslation.translationVoxel.y() + j) +
//                                                                    this->correlationN *
//                                                                    (potentialTranslation.translationVoxel.x() + i)];
//                }
//            }
//        }
//        potentialTranslation.covarianceX = resultingIntegral / maximumIntegral;
//        potentialTranslation.covarianceY = resultingIntegral / maximumIntegral;
//    }

// covariance calculation
    int definedRadiusVoxel = ceil(this->correlationN / 20);
//    double definedRadiusSI = cellSize * this->correlationN / 30.0;
    for (auto &potentialTranslation: potentialTranslations) {
        // calculate a distribution dependent on peak height
        std::vector<Eigen::Vector2d> listOfPoints;
        for (int i = -definedRadiusVoxel; i < definedRadiusVoxel + 1; i++) {
            for (int j = -definedRadiusVoxel; j < definedRadiusVoxel + 1; j++) {
                double currentPeakHeight = resultingCorrelationDouble[(potentialTranslation.translationVoxel.y() + j) +
                                                                      this->correlationN *
                                                                      (potentialTranslation.translationVoxel.x() + i)];
                int numberOfdatasetPoints = ceil(currentPeakHeight * currentPeakHeight / 1 * 200);
                for (int k = 0; k < numberOfdatasetPoints; k++) {
                    Eigen::Vector2d tmpVector(i * cellSize, j * cellSize);
                    listOfPoints.push_back(tmpVector);
                }
            }
        }

        //calculate covariance matrix based on this distribution
        double meanVar1 = 0;
        double meanVar2 = 0;
//        for (int i = 0; i < listOfPoints.size(); i++) {
//            meanVar1 += listOfPoints[i].x();
//            meanVar2 += listOfPoints[i].y();
//        }
//        meanVar1 = meanVar1 / listOfPoints.size();
//        meanVar2 = meanVar2 / listOfPoints.size();

//        meanVar1 = 0;
//        meanVar2 = 0;
        double cov1 = 0, cov2 = 0, var12 = 0;
        for (int i = 0; i < listOfPoints.size(); i++) {
            cov1 += pow((listOfPoints[i].x() - meanVar1), 2);
            cov2 += pow((listOfPoints[i].y() - meanVar2), 2);
            var12 += (listOfPoints[i].x() - meanVar1) * (listOfPoints[i].y() - meanVar2);
        }
        cov1 = cov1 / (listOfPoints.size() - 1);
        cov2 = cov2 / (listOfPoints.size() - 1);
        var12 = var12 / (listOfPoints.size() - 1);
        Eigen::Matrix2d tmpCovariance;
        tmpCovariance(0, 0) = cov1 * 10;
        tmpCovariance(1, 1) = cov2 * 10;
        tmpCovariance(0, 1) = var12 * 10;
        tmpCovariance(1, 0) = var12 * 10;
        potentialTranslation.covariance = tmpCovariance;
//        std::cout << tmpCovariance << std::endl;

        cov1 = 0, cov2 = 0, var12 = 0;
        for (int i = -definedRadiusVoxel; i < definedRadiusVoxel + 1; i++) {
            for (int j = -definedRadiusVoxel; j < definedRadiusVoxel + 1; j++) {
                double currentPeakHeight = resultingCorrelationDouble[(potentialTranslation.translationVoxel.y() + j) +
                                                                      this->correlationN *
                                                                      (potentialTranslation.translationVoxel.x() + i)];
                cov1 += pow((i * currentPeakHeight - meanVar1), 2);
                cov2 += pow((j * currentPeakHeight - meanVar2), 2);
                var12 += (i * currentPeakHeight - meanVar1) * (j * currentPeakHeight - meanVar2);

            }
        }
        cov1 = cov1 / (listOfPoints.size() - 1);
        cov2 = cov2 / (listOfPoints.size() - 1);
        var12 = var12 / (listOfPoints.size() - 1);


        tmpCovariance(0, 0) = cov1 * 500;
        tmpCovariance(1, 1) = cov2 * 500;
        tmpCovariance(0, 1) = var12 * 500;
        tmpCovariance(1, 0) = var12 * 500;
//        std::cout << tmpCovariance << std::endl;
//        std::cout << "end" << std::endl;
    }
    return potentialTranslations;
}

Eigen::Matrix4d softDescriptorRegistration::registrationOfTwoVoxelsSOFFTFast(double voxelData1Input[],
                                                                             double voxelData2Input[],
                                                                             Eigen::Matrix4d &initialGuess,
                                                                             Eigen::Matrix3d &covarianceMatrix,
                                                                             bool useInitialAngle,
                                                                             bool useInitialTranslation,
                                                                             double cellSize,
                                                                             bool useGauss,
                                                                             bool debug,double potentialNecessaryForPeak) {
    if(!useInitialAngle || !useInitialTranslation){
        std::cout << "this function has to be used with initial guess = true" << std::endl;
        exit(-1);
    }

    double goodGuessAlpha = std::atan2(initialGuess(1, 0),
                                       initialGuess(0, 0));


    std::vector<translationPeak> listOfTranslations;
    std::vector<Eigen::Matrix4d> listOfTransformations;

//   std::vector<double> maximumHeightPeakList;
    std::vector<rotationPeak> estimatedAngles;
    double angleCovariance;
    double angleTMP = this->sofftRegistrationVoxel2DRotationOnly(voxelData1Input, voxelData2Input, goodGuessAlpha,
                                                                 angleCovariance,debug);

    rotationPeak rotationPeakTMP;
    rotationPeakTMP.angle = angleTMP;
    rotationPeakTMP.covariance = angleCovariance;
    estimatedAngles.push_back(rotationPeakTMP);



//    std::cout << "number of possible solutions: " << estimatedAngles.size() << std::endl;

    int angleIndex = 0;
    for (auto &estimatedAngle: estimatedAngles) {

        //copy data
        for (int i = 0; i < N * N; i++) {
            this->voxelData1[i] = voxelData1Input[i];
            this->voxelData2[i] = voxelData2Input[i];
        }

        cv::Mat magTMP1(this->N, this->N, CV_64F, voxelData1);
        cv::Mat magTMP2(this->N, this->N, CV_64F, voxelData2);
        //add gaussian blur
        if (useGauss) {
            for (int i = 0; i < 2; i++) {
                cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
                cv::GaussianBlur(magTMP2, magTMP2, cv::Size(9, 9), 0);
            }
        }

        cv::Point2f pc(magTMP1.cols / 2., magTMP1.rows / 2.);
        //positive values mean COUNTER CLOCK WISE (open cv description) threfore negative rotation
        cv::Mat r = cv::getRotationMatrix2D(pc, estimatedAngle.angle * 180.0 / M_PI, 1.0);
        cv::warpAffine(magTMP1, magTMP1, r, magTMP1.size()); // what size I should use?

        std::vector<translationPeak> potentialTranslations = this->sofftRegistrationVoxel2DTranslationAllPossibleSolutions(
                voxelData1, voxelData2,
                cellSize,
                1.0,
                debug, angleIndex,potentialNecessaryForPeak);
        Eigen::Matrix4d estimatedRotationScans = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd rotation_vectorTMP(estimatedAngle.angle, Eigen::Vector3d(0, 0, 1));
        Eigen::Matrix3d tmpRotMatrix3d = rotation_vectorTMP.toRotationMatrix();
        estimatedRotationScans.block<3, 3>(0, 0) = tmpRotMatrix3d;
        translationPeak bestFitTranslation;
        if (useInitialTranslation) {
            double distance = 100000;
            for (auto &potentialTranslation: potentialTranslations) {
                double diffX = potentialTranslation.translationSI.x() - initialGuess(0, 3);
                double diffY = potentialTranslation.translationSI.y() - initialGuess(1, 3);
                if (distance > sqrt(diffX * diffX + diffY * diffY)) {

                    bestFitTranslation = potentialTranslation;
                    distance = sqrt(diffX * diffX + diffY * diffY);

                }
            }
        } else {
            double highestPeak = 0;
            int indexHighestPeak = 0;
            for (int i = 0; i < potentialTranslations.size(); i++) {
                if (potentialTranslations[i].peakHeight > highestPeak) {
                    indexHighestPeak = i;
                    highestPeak = potentialTranslations[i].peakHeight;
                }
            }
            bestFitTranslation = potentialTranslations[indexHighestPeak];
        }

        estimatedRotationScans(0, 3) = bestFitTranslation.translationSI.x();
        estimatedRotationScans(1, 3) = bestFitTranslation.translationSI.y();
        estimatedRotationScans(2, 3) = 0;

        listOfTransformations.push_back(estimatedRotationScans);
        listOfTranslations.push_back(bestFitTranslation);

        angleIndex++;
    }

    //find maximum peak:
    double highestPeak = 0;
    int indexHighestPeak = 0;
    for (int i = 0; i < listOfTransformations.size(); i++) {
        if (highestPeak < listOfTranslations[i].peakHeight) {
            highestPeak = listOfTranslations[i].peakHeight;
            indexHighestPeak = i;
        }
    }

    covarianceMatrix.block<2, 2>(0,
                                 0) = listOfTranslations[indexHighestPeak].covariance;
    covarianceMatrix(2, 2) = angleCovariance;

    return listOfTransformations[indexHighestPeak];//should be the transformation matrix from 1 to 2
}

std::vector<transformationPeak>
softDescriptorRegistration::registrationOfTwoVoxelsSOFFTAllSoluations(double voxelData1Input[],
                                                                      double voxelData2Input[],
                                                                      double cellSize,
                                                                      bool useGauss,
                                                                      bool debug, double potentialNecessaryForPeak,bool multipleRadii,
                                                                      bool useClahe,
                                                                      bool useHamming) {

    double timeToCalculate;


    std::vector<transformationPeak> listOfTransformations;
    std::vector<double> maximumHeightPeakList;
    std::vector<rotationPeak> estimatedAnglePeak;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    estimatedAnglePeak = this->sofftRegistrationVoxel2DListOfPossibleRotations(voxelData1Input, voxelData2Input,
                                                                               debug,multipleRadii,useClahe,useHamming);
//    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//    std::cout << "1: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;

//    std::cout << "number of possible solutions: " << estimatedAnglePeak.size() << std::endl;
//
//    for (auto &estimatedAngle: estimatedAnglePeak) {
//        std::cout << estimatedAngle.angle << std::endl;
//    }

    int angleIndex = 0;
    for (auto &estimatedAngle: estimatedAnglePeak) {

        begin = std::chrono::steady_clock::now();

        //copy data
        for (int i = 0; i < N * N; i++) {
            this->voxelData1[i] = voxelData1Input[i];
            this->voxelData2[i] = voxelData2Input[i];
        }

        cv::Mat magTMP1(this->N, this->N, CV_64F, voxelData1);
        cv::Mat magTMP2(this->N, this->N, CV_64F, voxelData2);
        //add gaussian blur
        if (useGauss) {
            for (int i = 0; i < 2; i++) {
                cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
                cv::GaussianBlur(magTMP2, magTMP2, cv::Size(9, 9), 0);
            }
        }

        cv::Point2f pc(magTMP1.cols / 2., magTMP1.rows / 2.);
        //positive values mean COUNTER CLOCK WISE (open cv description) threfore negative rotation
        cv::Mat r = cv::getRotationMatrix2D(pc, estimatedAngle.angle * 180.0 / M_PI, 1.0);
        cv::warpAffine(magTMP1, magTMP1, r, magTMP1.size()); // what size I should use?

//        end = std::chrono::steady_clock::now();
//        std::cout << "2: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;
//        begin = std::chrono::steady_clock::now();

        std::vector<translationPeak> potentialTranslations = this->sofftRegistrationVoxel2DTranslationAllPossibleSolutions(
                voxelData1, voxelData2,
                cellSize,
                1.0,
                debug, angleIndex, potentialNecessaryForPeak);
//        end = std::chrono::steady_clock::now();
//        std::cout << "3: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;
        transformationPeak transformationPeakTMP;
        transformationPeakTMP.potentialRotation = estimatedAngle;
        transformationPeakTMP.potentialTranslations = potentialTranslations;

        listOfTransformations.push_back(transformationPeakTMP);
        angleIndex++;
    }

    // save list of transformations
    if (debug) {

        std::ofstream myFile12;
        myFile12.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/dataForReadIn.csv");

        myFile12 << listOfTransformations.size();//number of possible rotations
        myFile12 << "\n";
        for (auto &listOfTransformation: listOfTransformations) {
            myFile12 << listOfTransformation.potentialTranslations.size();//numberOfPossibleTranslations
            myFile12 << "\n";
        }
        myFile12 << cellSize;//numberOfPossibleTranslations
        myFile12 << "\n";
        myFile12.close();

    }
    //save every transformation in files.
    if (debug) {

        int numberOfTransformations = 0;
        for (auto &listOfTransformation: listOfTransformations) {
            Eigen::Matrix4d currentMatrix = Eigen::Matrix4d::Identity();
            //rotation
            currentMatrix.block<3, 3>(0, 0) = generalHelpfulTools::getQuaternionFromRPY(0, 0,
                                                                                        listOfTransformation.potentialRotation.angle).toRotationMatrix();
            for (auto &potentialTranslation: listOfTransformation.potentialTranslations) {
                //translation
                currentMatrix.block<3, 1>(0, 3) = Eigen::Vector3d(potentialTranslation.translationSI.x(),
                                                                  potentialTranslation.translationSI.y(), 0);

                std::ofstream myFile12;
                myFile12.open(
                        "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/potentialTransformation" +
                        std::to_string(numberOfTransformations) + ".csv");
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 4; j++) {
                        myFile12 << currentMatrix(i, j) << " ";//number of possible rotations
                    }
                    myFile12 << "\n";
                }
                myFile12 << "\n";
                myFile12 << potentialTranslation.peakHeight;
                myFile12 << "\n";

                for (int i = 0; i < 2; i++) {
                    for (int j = 0; j < 2; j++) {
                        myFile12 << potentialTranslation.covariance(i, j) << " ";//number of possible rotations
                    }
                    myFile12 << "\n";
                }
                myFile12.close();
                numberOfTransformations++;
            }

        }


    }
    return listOfTransformations;
}

std::vector<translationPeak>
softDescriptorRegistration::peakDetectionOf2DCorrelationSimpleDouble1D(double maximumCorrelation, double cellSize,
                                                                       int impactOfNoiseFactor,
                                                                       double percentageOfMaxCorrelationIgnored) {

    float impactOfNoise = std::pow(2, impactOfNoiseFactor);

    std::vector<std::vector<int>> xPeaks, yPeaks;

    for (int j = 0; j < this->correlationN; j++) {
        std::vector<float> inputYLine;
        for (int i = 0; i < this->correlationN; i++) {
            inputYLine.push_back((float) resultingCorrelationDouble[j + this->correlationN * i]);
        }
        std::vector<int> out;
        PeakFinder::findPeaks(inputYLine, out, false, impactOfNoise);
//        for(int i = 0 ; i < out.size();i++){
//            std::cout << out[i] << std::endl;
//        }
//        std::cout <<"next"<< std::endl;
        yPeaks.push_back(out);
    }

    for (int i = 0; i < this->correlationN; i++) {
        std::vector<float> inputXLine;
        for (int j = 0; j < this->correlationN; j++) {
            inputXLine.push_back((float) resultingCorrelationDouble[j + this->correlationN * i]);
        }
        std::vector<int> out;
        PeakFinder::findPeaks(inputXLine, out, false, impactOfNoise);
//        for(int j = 0 ; j < out.size();j++){
//            std::cout << out[j] << std::endl;
//        }
//        std::cout <<"next"<< std::endl;
        xPeaks.push_back(out);

    }
    std::vector<translationPeak> potentialTranslations;
    for (int j = 0; j < this->correlationN; j++) {
        for (int i = 0; i < this->correlationN; i++) {
            auto iteratorX = std::find(xPeaks[i].begin(), xPeaks[i].end(), j);
            if (iteratorX != xPeaks[i].end()) {
                auto iteratorY = std::find(yPeaks[j].begin(), yPeaks[j].end(), i);
                if (iteratorY != yPeaks[j].end()) {
//                    std::cout << "found Peak:" << std::endl;
//                    std::cout << *iteratorX << std::endl;
//                    std::cout << *iteratorY  << std::endl;

//                    std::cout << i << std::endl;
//                    std::cout << j  << std::endl;
                    if (maximumCorrelation * percentageOfMaxCorrelationIgnored <
                        resultingCorrelationDouble[j + this->correlationN * i]) {
                        translationPeak tmpTranslationPeak;
                        tmpTranslationPeak.translationSI.x() = -((i - (int) (this->correlationN / 2.0)) * cellSize);
                        tmpTranslationPeak.translationSI.y() = -((j - (int) (this->correlationN / 2.0)) * cellSize);
                        tmpTranslationPeak.translationVoxel.x() = i;
                        tmpTranslationPeak.translationVoxel.y() = j;
                        tmpTranslationPeak.peakHeight = resultingCorrelationDouble[j + this->correlationN * i];
                        potentialTranslations.push_back(tmpTranslationPeak);
                    }

                }
            }
        }
    }
    return potentialTranslations;
}

std::vector<translationPeak>
softDescriptorRegistration::peakDetectionOf2DCorrelationOpenCVHoughTransform(double maximumCorrelation, double cellSize,
                                                                             int impactOfNoiseFactor,
                                                                             double percentageOfMaxCorrelationIgnored) {

    float impactOfNoise = std::pow(2, impactOfNoiseFactor);

    double *current2DCorrelation;

    current2DCorrelation = (double *) malloc(sizeof(double) * this->correlationN * this->correlationN);
    double maxValue = 0;
    //copy data
    for (int j = 0; j < this->correlationN; j++) {
        for (int i = 0; i < this->correlationN; i++) {
            current2DCorrelation[j + this->correlationN * i] = this->resultingCorrelationDouble[j +
                                                                                                this->correlationN * i];
            if (current2DCorrelation[j + this->correlationN * i] > maxValue) {
                maxValue = current2DCorrelation[j + this->correlationN * i];
            }
        }
    }

    for (int j = 0; j < this->correlationN; j++) {
        for (int i = 0; i < this->correlationN; i++) {
            current2DCorrelation[j + this->correlationN * i] =
                    current2DCorrelation[j + this->correlationN * i] / maxValue;
        }
    }


    cv::Mat magTMP1(this->correlationN, this->correlationN, CV_64F, current2DCorrelation);
    std::cout << magTMP1 << std::endl;
//    for (int j = 0; j < this->correlationN; j++) {
//        for (int i = 0; i < this->correlationN; i++) {
//            std::cout << magTMP1.at<double>(i,j) << std::endl;
//        }
//    }



//    cv::imshow("Display window", magTMP1);
//    int k = cv::waitKey(0); // Wait for a keystroke in the window

//    int morph_elem = 0;
//    int morph_size = 0;
//    cv::Mat element = getStructuringElement( morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
                                            cv::Size(ceil(0.02 * this->correlationN), ceil(0.02 * this->correlationN)));
    cv::morphologyEx(magTMP1, magTMP1, cv::MORPH_TOPHAT, element);
    std::cout << magTMP1 << std::endl;
    for (int j = 0; j < this->correlationN * this->correlationN; j++) {

//        newMatrix.data[j] = ((int)((int)newMatrix.data[j]/255.0));
        std::cout << (double) magTMP1.data[j] << std::endl;
    }
    std::cout << "here1" << std::endl;
    cv::imshow("Display window", magTMP1);
    int k = cv::waitKey(0); // Wait for a keystroke in the window

//    cv::Mat newMatrix = softDescriptorRegistration::imregionalmax(magTMP1);
    cv::Mat newMatrix = softDescriptorRegistration::opencv_imextendedmax(magTMP1, 0.03);
    std::cout << newMatrix.rows << " " << newMatrix.cols << std::endl;
    std::cout << newMatrix.type() << std::endl;
    std::cout << newMatrix << std::endl;

    cv::imshow("Display window", newMatrix);
    k = cv::waitKey(0); // Wait for a keystroke in the window
    cv::Mat dst = newMatrix;
    dst = dst * 255;
    dst.convertTo(dst, CV_8U);
    std::cout << "contours: " << std::endl;
    cv::imshow("Display window", dst);
    k = cv::waitKey(0); // Wait for a keystroke in the window
    std::vector<std::vector<cv::Point> > contours_1;
    std::vector<cv::Vec4i> hierarchy_1;
    cv::findContours(dst, contours_1, hierarchy_1, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    double maxLabelSize = (dst.rows / 4.0) * (dst.cols / 6.0);
    double minLabelSize = ((dst.rows / 40.0) * (dst.cols / 60.0));

    std::vector<std::vector<cv::Point> > goodContours;
    for (int i = 0; i < contours_1.size(); i++) {
        double size = cv::contourArea(contours_1[i]);
        if (size < maxLabelSize && size > minLabelSize) {
            goodContours.push_back(contours_1[i]);
        }
    }

    cv::Mat filterContours = cv::Mat::zeros(dst.size(), CV_8UC3);
    for (int i = 0; i < goodContours.size(); i++) {
        cv::RNG rng(12345);
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(filterContours, goodContours, i, color, 2, 8, hierarchy_1, 0, cv::Point());
    }

    cv::imshow("Contours", filterContours);
    cv::waitKey(0);

    free(current2DCorrelation);
    std::vector<translationPeak> potentialTranslations;
    return potentialTranslations;
}

bool softDescriptorRegistration::isPeak(cv::Mat mx[], std::vector<cv::Point> &conn_points) {
    cv::Point poi_point = conn_points.back();
    int row = poi_point.y;
    int col = poi_point.x;
    float poi_val = mx[0].at<float>(poi_point);
    bool isPeakEle = true;
    for (int mask_row = row - 1; mask_row <= row + 1; mask_row++) {
        for (int mask_col = col - 1; mask_col <= col + 1; mask_col++) {
            if (mask_row == row && mask_col == col) {
                continue;
            }
            float conn_pt_val = mx[0].at<float>(mask_row, mask_col);
            if (poi_val < conn_pt_val) {
                isPeakEle = false;
                break;
            }
            if (poi_val == conn_pt_val) {
                int Peak_status = mx[1].at<uchar>(mask_row, mask_col);
                if (Peak_status == 0) {
                    isPeakEle = false;
                    break;
                } else if (Peak_status == 1) {
                    isPeakEle = true;
                    break;
                } else {
                    cv::Point p(mask_col, mask_row);
                    std::vector<cv::Point>::iterator it;
                    it = std::find(conn_points.begin(), conn_points.end(), p);
                    if (it == conn_points.end()) {
                        conn_points.push_back(p);
                        isPeakEle = isPeakEle && isPeak(mx, conn_points);
                    }
                }
            }
        }
        if (isPeakEle == false) {
            break;
        }
    }
    return isPeakEle;
}

void softDescriptorRegistration::imextendedmax_imreconstruct(cv::Mat g, cv::Mat f, cv::Mat &dest) {

    cv::Mat m0, m1, m;
    m1 = f;
    do {
        m0 = m1.clone();
        cv::dilate(m0, m, cv::Mat());
        cv::min(g, m, m1);
    } while (cv::countNonZero(m1 != m0) != 0);
    dest = m1.clone();
}

cv::Mat softDescriptorRegistration::opencv_imextendedmax(cv::Mat &inputMatrix, double hParam) {

    std::cout << inputMatrix << std::endl;
//    cv::imshow("Display window", inputMatrix);
//    int k = cv::waitKey(0); // Wait for a keystroke in the window
    double h = hParam;
    cv::Mat gray_image, dst, fin_image, m, m2;

//    cvtColor(inputMatrix, gray_image, cv::COLOR_BGR2GRAY);
    cv::max((inputMatrix - h), 0, m);
    std::cout << m << std::endl;
//    cv::imshow("Display window", m);
//    k = cv::waitKey(0); // Wait for a keystroke in the window

    softDescriptorRegistration::imextendedmax_imreconstruct(inputMatrix, m, dst);
    std::cout << dst << std::endl;
    std::cout << "here4" << std::endl;
    cv::imshow("Display window", dst);
    int k = cv::waitKey(0); // Wait for a keystroke in the window
    cv::subtract(dst, 0.5, m2);
    std::cout << m2 << std::endl;
//    cv::imshow("Display window", m2);
//    k = cv::waitKey(0); // Wait for a keystroke in the window
    softDescriptorRegistration::imextendedmax_imreconstruct(dst, m2, m);
    std::cout << m << std::endl;
//    cv::imshow("Display window", m);
//    k = cv::waitKey(0); // Wait for a keystroke in the window
    cv::subtract(dst, m, m2);
    std::cout << m2 << std::endl;
    std::cout << "here5" << std::endl;
    cv::imshow("Display window", m2);
    k = cv::waitKey(0); // Wait for a keystroke in the window

    fin_image = m2;

    return fin_image;

}


cv::Mat softDescriptorRegistration::imregionalmax(cv::Mat &src) {
    cv::Mat padded;
    cv::copyMakeBorder(src, padded, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar::all(-1));
    cv::Mat mx_ch1(padded.rows, padded.cols, CV_8UC1, cv::Scalar(
            2)); //Peak elements will be represented by 1, others by 0, initializing Mat with 2 for differentiation
    cv::Mat mx[2] = {padded, mx_ch1}; //mx[0] is padded image, mx[1] is regional maxima matrix
    int mx_rows = mx[0].rows;
    int mx_cols = mx[0].cols;
    cv::Mat dest(mx[0].size(), CV_8UC1);

    //Check each pixel for local max
    for (int row = 1; row < mx_rows - 1; row++) {
        for (int col = 1; col < mx_cols - 1; col++) {
            std::vector<cv::Point> conn_points; //this vector holds all connected points for candidate pixel
            cv::Point p(col, row);
            conn_points.push_back(p);
            bool isPartOfPeak = isPeak(mx, conn_points);
            if (isPartOfPeak) {
                mx[1].at<uchar>(row, col) = 1;
            } else {
                mx[1].at<uchar>(row, col) = 0;
            }
        }
    }
    dest = mx[1](cv::Rect(1, 1, src.cols, src.rows));
    return dest;
}

double softDescriptorRegistration::normalizationFactorCalculation(int x, int y) {

    double tmpCalculation = 0;
//    tmpCalculation = abs(1.0/((x-this->correlationN/2)*(y-this->correlationN/2)));
//    tmpCalculation = this->correlationN * this->correlationN * (this->correlationN - (x + 1) + 1);
//    tmpCalculation = tmpCalculation * (this->correlationN - (y + 1) + 1);
    if (x < ceil(this->correlationN / 2)) {
        tmpCalculation = (x + 1);
    } else {
        tmpCalculation = (this->correlationN - x);
    }

    if (y < ceil(this->correlationN / 2)) {
        tmpCalculation = tmpCalculation * (y + 1);
    } else {
        tmpCalculation = tmpCalculation * (this->correlationN - y);
    }

    return (tmpCalculation);
}

std::vector<translationPeak> softDescriptorRegistration::peakDetectionOf2DCorrelationFindPeaksLibrary(double cellSize,
                                                                                                      double potentialNecessaryForPeak,
                                                                                                      double ignoreSidesPercentage) {

    double *current2DCorrelation;
    current2DCorrelation = (double *) malloc(sizeof(double) * this->correlationN * this->correlationN);

    double maxValue = 0;
    //copy data
    for (int j = 0; j < this->correlationN; j++) {
        for (int i = 0; i < this->correlationN; i++) {
            current2DCorrelation[j + this->correlationN * i] = this->resultingCorrelationDouble[j +
                                                                                                this->correlationN * i];
            if (current2DCorrelation[j + this->correlationN * i] > maxValue) {
                maxValue = current2DCorrelation[j + this->correlationN * i];
            }
        }
    }
    //normalize data
    for (int j = 0; j < this->correlationN; j++) {
        for (int i = 0; i < this->correlationN; i++) {
            current2DCorrelation[j + this->correlationN * i] =
                    current2DCorrelation[j + this->correlationN * i] / maxValue;
            this->resultingCorrelationDouble[j + this->correlationN * i] =
                    this->resultingCorrelationDouble[j + this->correlationN * i] / maxValue;
        }
    }
    cv::Mat magTMP1(this->correlationN, this->correlationN, CV_64F, this->resultingCorrelationDouble);
//    cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);

//    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
//                                            cv::Size(ceil(0.02 * this->correlationN), ceil(0.02 * this->correlationN)));
//    cv::morphologyEx(magTMP1, magTMP1, cv::MORPH_TOPHAT, element);

    size_t ourSize = this->correlationN;
    findpeaks::image_t<double> image = {
            ourSize, ourSize,
            this->resultingCorrelationDouble
    };

    std::vector<findpeaks::peak_t<double>> peaks = findpeaks::persistance(image);
    std::vector<translationPeak> tmpTranslations;
    for (const auto &p: peaks) {
        //calculation of level, that is a potential translation
        double levelPotential = p.persistence * sqrt(p.birth_level) *
                                Eigen::Vector2d((double) ((int) p.birth_position.x - (int) p.death_position.x),
                                                (double) ((int) p.birth_position.y - (int) p.death_position.y)).norm() *
                                511.0 / this->correlationN;
//        std::cout << p.persistence<< std::endl;
//        std::cout << Eigen::Vector2d((double) ((int) p.birth_position.x - (int) p.death_position.x),
//                                     (double) ((int) p.birth_position.y - (int) p.death_position.y)).norm() << std::endl;

//        if (p.persistence > 0.05  && p.birth_level>0.1) {
//
//        }

        bool inInterestingArea = true;
        if ((int) p.birth_position.x<ignoreSidesPercentage * this->correlationN || (int) p.birth_position.x>(
                1 - ignoreSidesPercentage) * this->correlationN ||
            (int) p.birth_position.y<ignoreSidesPercentage * this->correlationN || (int) p.birth_position.y>(
                    1 - ignoreSidesPercentage) * this->correlationN) {
            inInterestingArea = false;
        }


        if (p.birth_level > 0.1 && levelPotential > potentialNecessaryForPeak && inInterestingArea) {
//            std::cout << levelPotential << std::endl;
//            std::cout << "(" << p.birth_position.x << ", " << p.birth_position.y << ")\t"
//                      << p.birth_level << "  " << p.persistence << "  " << levelPotential
//                      << "\t(" << p.death_position.x << ", " << p.death_position.y << ")\n";
            translationPeak tmpTranslationPeak;
            tmpTranslationPeak.translationSI.x() = -(((int) p.birth_position.x - (int) (this->correlationN / 2.0)) *
                                                     cellSize);
            tmpTranslationPeak.translationSI.y() = -(((int) p.birth_position.y - (int) (this->correlationN / 2.0)) *
                                                     cellSize);
            tmpTranslationPeak.translationVoxel.x() = (int) p.birth_position.x;
            tmpTranslationPeak.translationVoxel.y() = (int) p.birth_position.y;
            tmpTranslationPeak.peakHeight = resultingCorrelationDouble[p.birth_position.y +
                                                                       this->correlationN * p.birth_position.x] *
                                            maxValue;
            tmpTranslations.push_back(tmpTranslationPeak);

//            std::cout << tmpTranslationPeak.translationSI.x() << "  " << tmpTranslationPeak.translationSI.y()
//                      << std::endl;
        }
    }
    free(current2DCorrelation);
    return (tmpTranslations);

}