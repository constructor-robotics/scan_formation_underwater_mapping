/***************************************************************************
  **************************************************************************
  
  SOFT: SO(3) Fourier Transforms
  Version 2.0

  Copyright (c) 2003, 2004, 2007 Peter Kostelec, Dan Rockmore
  
  This file is part of SOFT.

  SOFT is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  SOFT is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  See the accompanying LICENSE file for details.
  
  ************************************************************************
  ************************************************************************/

/*

 a somewhat memory-friendly test routine to rotate a spherical function
 by massaging its S^2 Fourier coefficients with Wigner-D functions

 bwIn = bandwidth of input signal
 bwOut = bandwidth of output signal (can up- or down-sample)
 degOut = max degree of spherical harmonic you want to use ( < bwOut )
 alpha, beta, gamma -> the three Euler angles

             0 <= alpha, gamma < 2*pi
             0 <= beta <= pi

 inputSamples -> filename of input samples in INTERLEAVED format
 outputSamples -> filename of output (rotated) samples in INTERLEAVED format

 Here are order of rotation events:
  1) rotate by gamma about the z-axis
  2) rotate by beta about the y-axis
  3) rotate by alpha about the z-axis.

 example: test_s2_rotate_fftw bwIn bwOut degOut alpha beta gamma inputSamples outputSamples

 example: test_s2_rotate_fftw 32 16 15 0.37 2.32 4.37 fctIn.dat fctOut.dat



 NOTE: Sometimes there is a segmentation fault *after* all the rotating and
 writing out of the output file is complete. I haven't tracked this down yet,
 but I believe it has to do with freeing up the memory associated with doing
 the S^2 transforms ... my array of double pointers are not pointing in the
 right places when I try to free memory. However, the rotation itself is
 correct.

*/

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "csecond.h"
#include "s2_cospmls.h"
#include "rotate_so3_fftw.h"


/* #define max(A, B) ((A) > (B) ? (A) : (B)) */

/**************************************************************/
/**************************************************************/


int main(int argc, char **argv)
{
  FILE *fp ;
  int i ;
  int bwIn, bwOut, degOut ;
  double alpha, beta, gamma ;
  double *sigInR, *sigInI, *sigOutR, *sigOutI ;
  double *scratch ;
  double tstart, tstop;
  double *seminaive_naive_tablespace, *trans_seminaive_naive_tablespace2;
  double *seminaive_naive_tablespace2 ;
  double **seminaive_naive_table2,**seminaive_naive_table ;
  double **trans_seminaive_naive_table2;

  if (argc < 9)
    {
      fprintf(stdout, "Usage: test_s2_rotate_fftw bwIn bwOut degOut ");
      fprintf(stdout, "alpha beta gamma ");
      fprintf(stdout, "input_filename output_filename\n");
      exit(0);
    }


  bwIn = atoi( argv[ 1 ] );
  bwOut = atoi( argv[ 2 ] );
  degOut = atoi( argv[ 3 ] );
  alpha = (double) atof( argv[ 4 ] );
  beta = (double) atof( argv[ 5 ] );
  gamma = (double) atof( argv[ 6 ] );

  sigInR = (double *) malloc(sizeof(double)*(4*bwIn*bwIn));
  sigInI = (double *) malloc(sizeof(double)*(4*bwIn*bwIn));
  sigOutR = (double *) malloc(sizeof(double)*(4*bwOut*bwOut));
  sigOutI = (double *) malloc(sizeof(double)*(4*bwOut*bwOut));

  if ( bwOut > bwIn )
    scratch = (double *) malloc(sizeof(double)*((14*bwOut*bwOut) + (52*bwOut) + (2*bwIn)));
  else
    scratch = (double *) malloc(sizeof(double)*((14*bwIn*bwIn) + (52*bwIn) + (2*bwIn)));

  seminaive_naive_tablespace =
    (double *) malloc(sizeof(double) *
		      (Reduced_Naive_TableSize(bwIn,bwIn) +
		       Reduced_SpharmonicTableSize(bwIn,bwIn)));

  trans_seminaive_naive_tablespace2 =
    (double *) malloc(sizeof(double) *
		      (Reduced_Naive_TableSize(bwOut,bwOut) +
		       Reduced_SpharmonicTableSize(bwOut,bwOut)));

  seminaive_naive_tablespace2 =
    (double *) malloc(sizeof(double) *
		      (Reduced_Naive_TableSize(bwOut,bwOut) +
		       Reduced_SpharmonicTableSize(bwOut,bwOut)));

  

  /****
       At this point, check to see if all the memory has been
       allocated. If it has not, there's no point in going further.
  ****/

  if ( (scratch == NULL) || 
       (sigInR == NULL ) || (sigInI == NULL ) ||
       (sigOutR == NULL ) || (sigOutI == NULL ) ||
       (seminaive_naive_tablespace == NULL) ||
       (trans_seminaive_naive_tablespace2 == NULL) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }
  

  fprintf(stdout,"Generating seminaive_naive tables...\n");
  seminaive_naive_table = SemiNaive_Naive_Pml_Table(bwIn, bwIn,
						    seminaive_naive_tablespace,
						    scratch);


  fprintf(stdout,"Generating seminaive_naive tables...\n");
  seminaive_naive_table2 = SemiNaive_Naive_Pml_Table(bwOut, bwOut,
						    seminaive_naive_tablespace2,
						    scratch);


  fprintf(stdout,"Generating trans_seminaive_naive tables...\n");
  trans_seminaive_naive_table2 =
    Transpose_SemiNaive_Naive_Pml_Table(seminaive_naive_table2,
					bwOut, bwOut,
					trans_seminaive_naive_tablespace2,
					scratch);

  fprintf(stdout,"reading in signal ...\n");

  /* read in signal */
  fp = fopen(argv[7], "r");
  for ( i = 0 ; i < (4*bwIn*bwIn) ; i ++ )
    {
      fscanf(fp,"%lf",sigInR+i);
      fscanf(fp,"%lf",sigInI+i);
    }
  fclose( fp ) ;

  fprintf(stdout,"about to rotate ...\n");
  tstart = csecond();

  rotateFctFFTW( bwIn, bwOut, degOut,
		 sigInR, sigInI,
		 sigOutR, sigOutI,
		 alpha, beta, gamma,
		 scratch,
		 seminaive_naive_table,
		 trans_seminaive_naive_table2 ) ;
  
  tstop = csecond();
  fprintf(stdout,"finished rotating ...\n");
  fprintf(stdout,"rotation time \t = %.4e\n", tstop - tstart);

  /* write out rotated signal */
  fp = fopen(argv[8], "w");
  for ( i = 0 ; i < (4*bwOut*bwOut) ; i ++ )
    {
      fprintf(fp,"%.15f\n%.15f\n",sigOutR[i],sigOutI[i]);
    }
  fclose( fp ) ;
 
  fprintf(stdout,"finished writing ...\n");
 
  free(trans_seminaive_naive_table2);
  free(seminaive_naive_table2);
  free(seminaive_naive_table);
  free(seminaive_naive_tablespace2);
  free(trans_seminaive_naive_tablespace2);
  free(seminaive_naive_tablespace);
 
  free(scratch);
  free(sigOutI);
  free(sigOutR);
  free(sigInI);
  free(sigInR);
 
  return 0 ;
 
}


