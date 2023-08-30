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

 test program to see if the FORWARD full SO3 transform actually works.

  - uses FFTW and Wigner-d symmetries

  input: - bandwidth bw 
         - input file name of real and imaginary parts of sample values ...
	   should be (2*bw)^3 many samples
	 - output file for real and imaginary parts of coefficients
	 - isReal -> 1: input is real
                  -> 0: input is complex
         - output format:
              0 -> ordered as the INVERSE TRANSFORM would expect the
                   coefficients
	      1 -> ordered (and labeled) by degree, i.e.

                      for l = 0 : bw - 1
		       for m1 = -l : l
                        for m2 = -l : l
                         coefficient of degree l, orders m1, m2
   

  example: test_soft_fftw_for bw InputSamples OutputCoefs isReal outputFormat

  example: test_soft_fftw_for 16 samples.dat coeffs.dat 1 1


*/


#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "wrap_fftw.h"
#include "utils_so3.h"  /* for totalCoeffs_so3() */
#include "csecond.h"

int main( int argc,
	  char **argv )
{
  int i, l, m1, m2, bw, n, n3 ;
  int dummy, format, isReal, tmpInt ;
  double tmpbuf[2] ;
  fftw_complex *signal, *coeffs ;
  double tstartF, tstopF, runtimeF ;
  FILE *fp ;
  
  if (argc < 6)
    {
      fprintf(stdout, "Usage: test_soft_fftw_for bw inputFile ");
      fprintf(stdout, "coef_file isReal order_flag\n");
      exit(0);
    }

  bw = atoi( argv[1] );
  isReal = atoi( argv[4] );
  format = atoi( argv[5] );

  n = 2 * bw ;
  n3 = n * n * n ;

  /* signal */
  signal = fftw_malloc( sizeof( fftw_complex ) * n3 ) ;

  /* coefficients totalCoeffs_so3( bw) amount of space */
  coeffs = fftw_malloc(sizeof( fftw_complex ) * totalCoeffs_so3( bw ) ) ;

  /* check if any problems allocating memory */
  if ( ( signal == NULL) || ( coeffs == NULL ) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }

  /* read in samples */
  fp = fopen( argv[2], "r" );
  for ( i = 0 ; i < n3 ; i ++ )
    {
      /* first the real part */
      fscanf(fp, "%lf", tmpbuf) ;

      /* now deal with imaginary part */
      if ( isReal )
	tmpbuf[1] = 0.0 ; /* provide trivial imaginary part */
      else
	fscanf(fp, "%lf", tmpbuf+1);  /* honest to goodness imaginary part */

      signal[i][0] = tmpbuf[0];
      signal[i][1] = tmpbuf[1];
    }
  fclose ( fp ) ;

  /* initialize time */
  runtimeF = 0.0 ;

  /* turn on stopwatch */
  tstartF = csecond( ) ;

  Forward_SO3_Naive_fftw_W( bw,
			    signal,
			    coeffs,
			    isReal ) ;

  /* turn off stopwatch */
  tstopF = csecond( ) ;
  runtimeF += tstopF - tstartF ;
  fprintf(stderr,"forward time \t = %.4e\n", tstopF - tstartF);

  /* write out coefficients to disk */
  /* ordered as the inverse transform expects the coefficients */
  if ( format == 0 )
    {
      tmpInt = totalCoeffs_so3( bw ) ;
      fp = fopen( argv[ 3 ], "w" );
      for ( i = 0 ; i < tmpInt ; i ++ )
	fprintf(fp, "%.16f\n%.16f\n",
		coeffs[ i ][0], coeffs[ i ][1] ) ;
      fclose( fp ) ;
    }
  else /* ordered in a more human friendly way */
    {
      fp = fopen( argv[ 3 ], "w" );
      for ( l = 0 ; l < bw ; l ++ )
	for ( m1 = -l ; m1 < l + 1 ; m1 ++ )
	  for ( m2 = -l ; m2 < l + 1 ; m2 ++ )
	    {
	      dummy = so3CoefLoc( m1, m2, l, bw ) ;
	      fprintf(fp, "l = %d m1 = %d m2 = %d\t%.16f\t%.16f\n",
		      l, m1, m2,
		      coeffs[ dummy ][0], coeffs[ dummy ][1] ) ;
	    }
      fclose( fp ) ;
    }

  fftw_free( coeffs );
  fftw_free( signal );

  return 0 ;
}
