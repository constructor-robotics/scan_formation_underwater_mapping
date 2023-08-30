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

 test program to see if the INVERSE full SO3 transform actually works.

  - uses FFTW and Wigner-d symmetries

  input: - bandwidth bw 
         - input file name containing real and imaginary parts of
	   coefficients ... should be 1/3 bw (4 bw^2-1) values
	   (ordered as given by forward transform -> NOT human!)
	 - output file for real and imaginary parts of samples;
	   there are (2 bw)^3 many samples

	 - isReal -> 1: coefficients are for a strictly REAL function 
                  -> 0: coefficients are for a complex-valued function
   

  example: test_soft_fftw_inv bw InputCoeffs OutputSamples isReal

  example: test_soft_fftw_inv 16 coeffs.dat samples.dat 1


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
  int i, bw, n, n3 ;
  int isReal ;
  int tmpInt ;
  // int l, m1, m2 ;
  // int dummy, format, isReal ;
  double tmpbuf[2] ;
  fftw_complex *signal, *coeffs ;
  double tstartI, tstopI, runtimeI ;
  FILE *fp ;
  
  if (argc < 5)
    {
      fprintf(stdout, "Usage: test_soft_fftw_inv bw coefFile ");
      fprintf(stdout, "sample_file isReal\n");
      exit(0);
    }

  bw = atoi( argv[1] );
  isReal = atoi( argv[4] );

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

  /* read in coeffs */
  tmpInt = totalCoeffs_so3( bw ) ;
  fp = fopen( argv[2], "r" );
  for ( i = 0 ; i < tmpInt ; i ++ )
    {
      /* first the real part */
      fscanf(fp, "%lf", tmpbuf) ;

      /* now the imaginary part */
      fscanf(fp, "%lf", tmpbuf+1);

      coeffs[i][0] = tmpbuf[0];
      coeffs[i][1] = tmpbuf[1];
    }
  fclose ( fp ) ;

  /* initialize time */
  runtimeI = 0.0 ;

  /* turn on stopwatch */
  tstartI = csecond( ) ;

  Inverse_SO3_Naive_fftw_W( bw,
			    coeffs,
			    signal,
			    isReal ) ;

  /* turn off stopwatch */
  tstopI = csecond( ) ;
  runtimeI += tstopI - tstartI ;
  fprintf(stderr,"inverse time \t = %.4e\n", tstopI - tstartI);

  /* write out samples to disk */
  fp = fopen( argv[ 3 ], "w" );
  if ( isReal ) /* strictly real */
    for ( i = 0 ; i < n3 ; i ++ )
      fprintf(fp, "%.16f\n",
	      signal[ i ][0] ) ;
  else /* complex samples */
    for ( i = 0 ; i < n3 ; i ++ )
      fprintf(fp, "%.16f\n%.16f\n",
	      signal[ i ][0], signal[ i ][1] ) ;

  fclose( fp ) ;


  fftw_free( coeffs );
  fftw_free( signal );

  return 0 ;
}
