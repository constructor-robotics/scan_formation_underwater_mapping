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

 bw = bandwidth of input signal
 degOut = max degree of spherical harmonic you want to use ( < bw )
 alpha, beta, gamma -> the three Euler angles

             0 <= alpha, gamma < 2*pi
             0 <= beta <= pi

 inputSamples -> filename of input samples in INTERLEAVED format
 outputSamples -> filename of output (rotated) samples in INTERLEAVED format

 Here are order of rotation events:
  1) rotate by gamma about the z-axis
  2) rotate by beta about the y-axis
  3) rotate by alpha about the z-axis.

 example: test_s2_rotate_mem bw degOut alpha beta gamma inputSamples outputSamples

 example: test_s2_rotate_mem 32 31 0.37 2.32 4.37 fctIn.dat fctOut.dat


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

#include "FST_semi_memo.h"
#include "csecond.h"
#include "cospmls.h"
#include "s2_primitive.h"
#include "legendreTransforms.h"
#include "rotate_so3_mem.h"


/* #define max(A, B) ((A) > (B) ? (A) : (B)) */

/**************************************************************/
/**************************************************************/


int main(int argc, char **argv)
{
  FILE *fp ;
  int i ;
  int bw, degOut ;
  double alpha, beta, gamma ;
  double *sigR, *sigI ;
  double *scratch ;
  double tstart, tstop ;
  double *seminaive_naive_tablespace ;
  double *trans_seminaive_naive_tablespace;
  double **seminaive_naive_table ;
  double **trans_seminaive_naive_table;

  if (argc < 3)
    {
      fprintf(stdout, "Usage: test_s2_rotate_mem bw degOut ");
      fprintf(stdout, "alpha beta gamma  ");
      fprintf(stdout, "input_filename output_filename\n");
      exit(0);
    }


  bw = atoi( argv[ 1 ] );
  degOut = atoi( argv[ 2 ] );
  alpha = (double) atof( argv[ 3 ] );
  beta = (double) atof( argv[ 4 ] );
  gamma = (double) atof( argv[ 5 ] );

  sigR = (double *) malloc(sizeof(double)*(4*bw*bw));
  sigI = (double *) malloc(sizeof(double)*(4*bw*bw));
  scratch = (double *) malloc(sizeof(double)*((10*bw*bw) + (48 * bw)));


  seminaive_naive_tablespace =
    (double *) malloc(sizeof(double) *
		      (Reduced_Naive_TableSize(bw,bw) +
		       Reduced_SpharmonicTableSize(bw,bw)));

  trans_seminaive_naive_tablespace =
    (double *) malloc(sizeof(double) *
		      (Reduced_Naive_TableSize(bw,bw) +
		       Reduced_SpharmonicTableSize(bw,bw)));

  /****
       At this point, check to see if all the memory has been
       allocated. If it has not, there's no point in going further.
  ****/

  if ( (scratch == NULL) || 
       (sigR == NULL ) || (sigI == NULL ) ||
       (seminaive_naive_tablespace == NULL) ||
       (trans_seminaive_naive_tablespace == NULL) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }
  

  fprintf(stdout,"Generating seminaive_naive tables...\n");
  seminaive_naive_table = SemiNaive_Naive_Pml_Table(bw, bw,
						    seminaive_naive_tablespace,
						    scratch);


  fprintf(stdout,"Generating trans_seminaive_naive tables...\n");
  trans_seminaive_naive_table =
    Transpose_SemiNaive_Naive_Pml_Table(seminaive_naive_table,
					bw, bw,
					trans_seminaive_naive_tablespace,
					scratch);


  fprintf(stdout,"reading in signal ...\n");

  /* read in signal */
  fp = fopen(argv[6], "r");
  for ( i = 0 ; i < (4*bw*bw) ; i ++ )
    {
      fscanf(fp,"%lf",sigR+i);
      fscanf(fp,"%lf",sigI+i);
    }
  fclose( fp ) ;

  fprintf(stdout,"about to rotate ...\n");
  tstart = csecond() ;

  rotateFct_mem( bw, degOut,
		 sigR, sigI,
		 alpha, beta, gamma,
		 scratch,
		 seminaive_naive_table,
		 trans_seminaive_naive_table ) ;

  tstop = csecond();
  fprintf(stdout,"finished rotating ...\n");
  fprintf(stdout,"rotation time \t = %.4e\n", tstop - tstart);
 
  /* write out rotated signal */
  fp = fopen(argv[7], "w");
  for ( i = 0 ; i < (4*bw*bw) ; i ++ )
    {
      fprintf(fp,"%.15f\n%.15f\n",sigR[i],sigI[i]);
    }
  fclose( fp ) ;
 
  fprintf(stdout,"finished writing ...\n");
 
  /* clean up */
  free(trans_seminaive_naive_table);
  free(seminaive_naive_table);
  free(trans_seminaive_naive_tablespace);
  free(seminaive_naive_tablespace);
 
  free(scratch);
  free(sigI);
  free(sigR);
 
  return 0 ;
 
}


