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

                double hammingCoeff = 1;
                resampledMagnitudeSO3_1TMP[k + j * bandwidth * 2] =
                        255 * magnitude1Shifted[yIndex + N * xIndex] * hammingCoeff;
                resampledMagnitudeSO3_2TMP[k + j * bandwidth * 2] =
                        255 * magnitude2Shifted[yIndex + N * xIndex] * hammingCoeff;
            }
        }




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
    }




    //use soft descriptor to calculate the correlation
    this->sofftCorrelationObject.correlationOfTwoSignalsInSO3(resampledMagnitudeSO3_1, resampledMagnitudeSO3_2,
                                                              resultingCorrelationComplex);




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


    auto minmax = std::min_element(correlationAveraged.begin(), correlationAveraged.end());
    long distanceToMinElement = std::distance(correlationAveraged.begin(), minmax);
    std::rotate(correlationAveraged.begin(), correlationAveraged.begin() + distanceToMinElement,
                correlationAveraged.end());

    std::vector<int> out;

    PeakFinder::findPeaks(correlationAveraged, out, true, 8.0);//was 4.0


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

    //double meanCorrelation = 0;
    int indexMaximumCorrelationI;
    int indexMaximumCorrelationJ;
    double maximumCorrelation = 0;
    for (int j = 0; j < this->correlationN; j++) {
        for (int i = 0; i < this->correlationN; i++) {
            int indexX = (this->correlationN / 2 + i + this->correlationN) % this->correlationN;// changed j and i here
            int indexY = (this->correlationN / 2 + j + this->correlationN) % this->correlationN;
            //maybe without sqrt, but for now thats fine
            double normalizationFactorForCorrelation =
                    1 / this->normalizationFactorCalculation(indexX, indexY);
//            double normalizationFactorForCorrelation = 1/this->normalizationFactorCalculation(indexX, indexY);
            normalizationFactorForCorrelation = sqrt(normalizationFactorForCorrelation);

            resultingCorrelationDouble[indexY + this->correlationN * indexX] = normalizationFactorForCorrelation * sqrt(
                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][0] *
                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][0] +
                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][1] *
                    resultingShiftPeaks2DCorrelation[j + this->correlationN * i][1]); // magnitude;

            if (maximumCorrelation < resultingCorrelationDouble[indexY + this->correlationN * indexX]) {
                maximumCorrelation = resultingCorrelationDouble[indexY + this->correlationN * indexX];
            }
        }
    }

    std::vector<translationPeak> potentialTranslations = this->peakDetectionOf2DCorrelationFindPeaksLibrary(cellSize,
                                                                                                            potentialNecessaryForPeak);

    // covariance calculation
    int definedRadiusVoxel = ceil(this->correlationN / 20);
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



double softDescriptorRegistration::normalizationFactorCalculation(int x, int y) {

    double tmpCalculation = 0;
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

        bool inInterestingArea = true;
        if ((int) p.birth_position.x<ignoreSidesPercentage * this->correlationN || (int) p.birth_position.x>(
                1 - ignoreSidesPercentage) * this->correlationN ||
            (int) p.birth_position.y<ignoreSidesPercentage * this->correlationN || (int) p.birth_position.y>(
                    1 - ignoreSidesPercentage) * this->correlationN) {
            inInterestingArea = false;
        }


        if (p.birth_level > 0.1 && levelPotential > potentialNecessaryForPeak && inInterestingArea) {
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


        }
    }
    free(current2DCorrelation);
    return (tmpTranslations);

}