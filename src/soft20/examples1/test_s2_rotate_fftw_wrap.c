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

 inputSamples -> filename of input samples
 outputSamples -> filename of output (rotated) samples

 isReal = 1: samples are strictly real (so no imaginary parts)
 isReal = 0: samples are complex (so in interleaved format)


 Here are order of rotation events:
  1) rotate by gamma about the z-axis
  2) rotate by beta about the y-axis
  3) rotate by alpha about the z-axis.

 example: test_s2_rotate_fftw_wrap bw alpha beta gamma inputSamples outputSamples isReal

 example: test_s2_rotate_fftw_wrap 8 0.37 2.32 4.37 randomS2sig_bw8.dat yyy.dat 0


 NOTE: Sometimes there is a segmentation fault *after* all the rotating and
 writing out of the output file is complete. I haven't tracked this down yet,
 but I believe it has to do with freeing up the memory associated with doing
 the S^2 transforms ... my array of double pointers are not pointing in the
 right places when I try to free memory. However, the rotation itself is
 correct.

*/

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "wrap_fftw.h"
#include "csecond.h"

/**************************************************************/
/**************************************************************/


int main(int argc, char **argv)
{
  FILE *fp ;
  int bw, i, isReal ;
  double alpha, beta, gamma ;
  double *sigIn, *sigOut ;
  double tstart, tstop ;

  if (argc < 7)
    {
      fprintf(stdout, "Usage: test_s2_rotate_fftw_wrap bw ");
      fprintf(stdout, "alpha beta gamma ");
      fprintf(stdout, "input_filename output_filename isReal\n");
      fprintf(stdout, " isReal = 1: signal and pattern strictly real (no interleaved)\n");
      fprintf(stdout, " isReal = 0: signal and pattern complex (interleaved)\n");
      exit(0);
    }


  bw = atoi( argv[ 1 ] );
  alpha = (double) atof( argv[ 2 ] );
  beta = (double) atof( argv[ 3 ] );
  gamma = (double) atof( argv[ 4 ] );
  isReal = atoi(argv[ 7 ] );

  if ( isReal )
    {
      sigIn = (double *) malloc(sizeof(double)*(4*bw*bw));
      sigOut = (double *) malloc(sizeof(double)*(4*bw*bw));
    }
  else
    {
      sigIn = (double *) malloc(sizeof(double)*(2*4*bw*bw));
      sigOut = (double *) malloc(sizeof(double)*(2*4*bw*bw));
    }

  /****
       At this point, check to see if all the memory has been
       allocated. If it has not, there's no point in going further.
  ****/

  if ( (sigIn == NULL ) || (sigOut == NULL ) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }

  fprintf(stdout,"reading in signal ...\n");

  /* read in signal */
  fp = fopen(argv[5], "r");
  if ( isReal )
    {
      for ( i = 0 ; i < (4*bw*bw) ; i ++ )
	{
	  fscanf(fp,"%lf",sigIn+i);
	}
    }
  else
    {
      for ( i = 0 ; i < (4*bw*bw) ; i ++ )
	{
	  fscanf(fp,"%lf",sigIn+2*i);
	  fscanf(fp,"%lf",sigIn+2*i+1);
	}
    }
  fclose( fp ) ;

  fprintf(stdout,"about to rotate ...\n");
  tstart = csecond();

  s2RotateFFTW( bw,
		sigIn,
		sigOut,
		alpha, beta, gamma,
		isReal );
		  
  tstop = csecond();
  fprintf(stdout,"finished rotating ...\n");
  fprintf(stdout,"rotation time \t = %.4e\n", tstop - tstart);

  /* write out rotated signal */
  fp = fopen(argv[6], "w");
  if ( isReal )
    {
      for ( i = 0 ; i < (4*bw*bw) ; i ++ )
	{
	  fprintf(fp,"%.15f\n",sigOut[i]);
	}
    }
  else
    {
      for ( i = 0 ; i < (4*bw*bw) ; i ++ )
	{
	  fprintf(fp,"%.15f\n%.15f\n",sigOut[2*i],sigOut[2*i+1]);
	}
    }



  fclose( fp ) ;
 
  fprintf(stdout,"finished writing ...\n");
 
  free(sigOut);
  free(sigIn);
 
  return 0 ;
 
}


