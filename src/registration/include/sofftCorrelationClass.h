//
// Created by tim-linux on 01.03.22.
//

#ifndef UNDERWATERSLAM_SOFFTCORRELATIONCLASS_H
#define UNDERWATERSLAM_SOFFTCORRELATIONCLASS_H
#include "fftw3.h"
#include "soft20/s2_cospmls.h"
#include "soft20/s2_semi_memo.h"
#include "soft20/makeweights.h"
#include "soft20/so3_correlate_fftw.h"
#include "soft20/soft_fftw.h"
#include <cmath>



#define NORM(x) ( (x[0])*(x[0]) + (x[1])*(x[1]) )

class sofftCorrelationClass {
public:
    sofftCorrelationClass(int N,int bwOut,int bwIn,int degLim){
        this->N = N;
        this->bwOut = bwOut;
        this->bwIn = bwIn;
        this->degLim = degLim;

//        bwIn = this->N / 2;
//        bwOut = this->N / 2;
//        degLim = bwOut - 1;

        sigR = (double *) calloc(this->N *this->N, sizeof(double));
        sigI = (double *) calloc(this->N *this->N, sizeof(double));
        so3Sig = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * (8 * bwOut * bwOut * bwOut));
        workspace1 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * (8 * bwOut * bwOut * bwOut));
        workspace2 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * ((14 * bwIn * bwIn) + (48 * bwIn)));
        workspace3 = (double *) malloc(sizeof(double) * (12 *this->N +this->N * bwIn));
        sigCoefR = (double *) malloc(sizeof(double) * bwIn * bwIn);
        sigCoefI = (double *) malloc(sizeof(double) * bwIn * bwIn);
        patCoefR = (double *) malloc(sizeof(double) * bwIn * bwIn);
        patCoefI = (double *) malloc(sizeof(double) * bwIn * bwIn);
        so3Coef = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * ((4 * bwOut * bwOut * bwOut - bwOut) / 3));


        seminaive_naive_tablespace =
                (double *) malloc(sizeof(double) *
                                  (Reduced_Naive_TableSize(bwIn, bwIn) +
                                   Reduced_SpharmonicTableSize(bwIn, bwIn)));

        weights = (double *) malloc(sizeof(double) * (4 * bwIn));

        /****
             At this point, check to see if all the memory has been
             allocated. If it has not, there's no point in going further.
        ****/

        /* create fftw plans for the S^2 transforms */
        /* first for the dct */
        dctPlan = fftw_plan_r2r_1d(2 * bwIn, weights, workspace3,
                                   FFTW_REDFT10, FFTW_ESTIMATE);

        /* now for the fft */
        /*
           IMPORTANT NOTE!!! READ THIS!!!

           Now to make the fft plans.

           Please note that the planning-rigor flag *must be* FFTW_ESTIMATE!
           Why? Well, to try to keep things simple. I am using some of the
           pointers to arrays in rotateFct's arguments in the fftw-planning
           routines. If the planning-rigor is *not* FFTW_ESTIMATE, then
           the arrays will be written over during the planning stage.

           Therefore, unless you are really really sure you know what
           you're doing, keep the rigor as FFTW_ESTIMATE !!!
        */

        /*
          fftw "preamble" ;
          note  that this places in the transposed array
        */

        rank = 1;
        dims[0].n = 2 * bwIn;
        dims[0].is = 1;
        dims[0].os = 2 * bwIn;
        howmany_rank = 1;
        howmany_dims[0].n = 2 * bwIn;
        howmany_dims[0].is = 2 * bwIn;
        howmany_dims[0].os = 1;

        fftPlan = fftw_plan_guru_split_dft(rank, dims,
                                           howmany_rank, howmany_dims,
                                           sigR, sigI,
                                           (double *) workspace2,
                                           (double *) workspace2 + (this->N *this->N),
                                           FFTW_ESTIMATE);

        this->N = 2 * bwOut;
        howmany =this->N *this->N;
        idist =this->N;
        odist =this->N;
        rank = 2;
        inembed[0] =this->N;
        inembed[1] =this->N *this->N;
        onembed[0] =this->N;
        onembed[1] =this->N *this->N;
        istride = 1;
        ostride = 1;
        na[0] = 1;
        na[1] =this->N;

        p1 = fftw_plan_many_dft(rank, na, howmany,
                                workspace1, inembed,
                                istride, idist,
                                so3Sig, onembed,
                                ostride, odist,
                                FFTW_FORWARD, FFTW_ESTIMATE);


        fprintf(stdout, "Generating seminaive_naive tables...\n");
        seminaive_naive_table = SemiNaive_Naive_Pml_Table(bwIn, bwIn,
                                                          seminaive_naive_tablespace,
                                                          (double *) workspace2);


        /* make quadrature weights for the S^2 transform */
        makeweights(bwIn, weights);

        this->N = 2 * bwIn;


    }
    ~sofftCorrelationClass (){
//        free(sigR);
//        free(sigI);
//        free(so3Sig);
//        free(workspace1);
//        free(workspace2);
//        free(workspace3);
//        free(sigCoefR);
//        free(sigCoefI);
//        free(patCoefR);
//        free(patCoefI);
//        free(so3Coef);
//
//        free(seminaive_naive_tablespace);
//        free(weights);
    }




    void correlationOfTwoSignalsInSO3(double resampledMagnitude1[], double resampledMagnitude2[], fftw_complex so3SigReturn[]);

private:
    int N;//describes the size of the overall voxel system

    int i;
    int bwIn, bwOut, degLim;

    fftw_complex *workspace1, *workspace2;
    double *workspace3;
    double *sigR, *sigI;
    double *sigCoefR, *sigCoefI;
    double *patCoefR, *patCoefI;
    fftw_complex *so3Sig, *so3Coef;
    fftw_plan p1;
    int na[2], inembed[2], onembed[2];
    int rank, howmany, istride, idist, ostride, odist;
    int tmp, maxloc, ii, jj, kk;
    double maxval, tmpval;
    double *weights;
    double *seminaive_naive_tablespace;
    double **seminaive_naive_table;
    fftw_plan dctPlan, fftPlan;
    int howmany_rank;
    fftw_iodim dims[1], howmany_dims[1];

};


#endif //UNDERWATERSLAM_SOFFTCORRELATIONCLASS_H
