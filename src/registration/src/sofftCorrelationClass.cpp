//
// Created by tim-linux on 01.03.22.
//


#include "sofftCorrelationClass.h"



void sofftCorrelationClass::correlationOfTwoSignalsInSO3(double resampledMagnitude1[], double resampledMagnitude2[], fftw_complex so3SigReturn[]){
    //so3SigReturn is of size (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * (8 * bwOut * bwOut * bwOut))

    for (i = 0; i < N * N; i++) {
        sigR[i] = resampledMagnitude1[i];
        sigI[i] = 0;
    }
    FST_semi_memo(sigR, sigI,
                  sigCoefR, sigCoefI,
                  bwIn, seminaive_naive_table,
                  (double *) workspace2, 0, bwIn,
                  &dctPlan, &fftPlan,
                  weights);

    /* read in SIGNAL samples */
    /* first the signal */
    for (i = 0; i < N * N; i++) {
        sigR[i] = resampledMagnitude2[i];
        sigI[i] = 0;
    }

    FST_semi_memo(sigR, sigI,
                  patCoefR, patCoefI,
                  bwIn, seminaive_naive_table,
                  (double *) workspace2, 0, bwIn,
                  &dctPlan, &fftPlan,
                  weights);


    so3CombineCoef_fftw(bwIn, bwOut, degLim,
                        sigCoefR, sigCoefI,
                        patCoefR, patCoefI,
                        so3Coef);




    /* now inverse so(3) */
    Inverse_SO3_Naive_fftw(bwOut,
                           so3Coef,
                           so3Sig,
                           workspace1,
                           workspace2,
                           workspace3,
                           &p1,
                           0);

    /* now find max value */
//    maxval = 0.0;
//    maxloc = 0;
//    for (i = 0; i < 8 * bwOut * bwOut * bwOut; i++) {
//        /*
//      if (resultingCorrelation[i][0] >= maxval)
//      {
//      maxval = resultingCorrelation[i][0];
//      maxloc = i ;
//      }
//        */
//        tmpval = NORM(so3Sig[i]);
//        if (tmpval > maxval) {
//            maxval = tmpval;
//            maxloc = i;
//        }
//
//    }
//
//    ii = floor(maxloc / (4. * bwOut * bwOut));
//    tmp = maxloc - (ii * 4. * bwOut * bwOut);
//    jj = floor(tmp / (2. * bwOut));
//    tmp = maxloc - (ii * 4 * bwOut * bwOut) - jj * (2 * bwOut);
//    kk = tmp;

//    printf("ii = %d\tjj = %d\tkk = %d\n", ii, jj, kk);
//
//    printf("alpha = %f\nbeta = %f\ngamma = %f\n",
//           M_PI * jj / ((double) bwOut),
//           M_PI * (2 * ii + 1) / (4. * bwOut),
//           M_PI * kk / ((double) bwOut));


    //this is the correlation:  return: resultingCorrelation

    for(i = 0 ; i< (8 * bwOut * bwOut * bwOut);i++){
        so3SigReturn[i][0] = so3Sig[i][0];
        so3SigReturn[i][1] = so3Sig[i][1];
    }

}