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

  This program will test the accuracy and speed of the routines
  by doing the following

  1) generate random wigner coefficients
  2) do a Wigner synthesis to get sample values
  3) do a Wigner analysis to get new coefficients
  4) compare the new coefficients with the old


  input: orders m1, m2
         bandwidth bw
	 number of loops
	 output file (optional) - to write the differences
         between the original coeffs and new ones

  example: test_Wigner_Naive m1 m2 bw loops output_file

  example: test_Wigner_Naive 3 4 16 coeffs.dat splat.dat


*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "utils_so3.h"
#include "makeWigner.h"
#include "makeweights.h"
#include "wignerTransforms.h"
#include "csecond.h"

int main ( int argc ,
	   char **argv )
{
  int i, j, m1, m2, bw, n ;
  int loops, m ;
  long int seed ;
  double *coeffs, *signal, *newcoeffs;
  double *wigners, *wignersTrans ;
  double *workspace, *scratch ;
  double *weights ;
  double *sinPts, *cosPts ;
  double *sinPts2, *cosPts2 ;
  double tmp_error, sum_error;
  double tmp_relerror, sum_relerror;
  double tstartA, tstopA, runtimeA ;
  double tstartB, tstopB, runtimeB ;
  double *relerror, *curmax;
  double ave_error, ave_relerror, stddev_error, stddev_relerror ;
  FILE *fp ;

  
  if (argc < 5)
    {
      fprintf(stdout,"Usage: test_Wigner_Naive m1 m2 bw loops [output_file]\n");
      exit(0);
    }

  m1 = atoi( argv[1] );
  m2 = atoi( argv[2] );
  bw = atoi( argv[3] );
  loops = atoi( argv[4] ) ;
  m = MAX( ABS( m1 ) , ABS( m2 ) ) ;
  n = 2 * bw ;
  runtimeA = 0.0 ;
  runtimeB = 0.0 ;

  weights = ( double * ) malloc(sizeof( double ) * (2*bw) ) ;
  coeffs = ( double * ) malloc(sizeof( double ) * (bw - m) ) ;
  newcoeffs = ( double * ) malloc(sizeof( double ) * (bw - m) ) ;
  signal = ( double * ) malloc(sizeof( double ) * n ) ;
  wigners = ( double * ) malloc( sizeof( double ) * ( bw - m ) * n ) ;
  wignersTrans = ( double * ) malloc( sizeof( double ) * ( bw - m ) * n ) ;
  workspace = (double *) malloc(sizeof( double ) * (4 + 6) * n ) ;
  sinPts = workspace ;
  cosPts = sinPts + n ;
  sinPts2 = cosPts + n ;
  cosPts2 = sinPts2 + n ;
  scratch = cosPts2 + n ; /* scratch needs to be of size 6*n */

  /* note that the definition of wigSpec requires that instead of
     evaluating at beta, I need to evaluate at beta/2; ergo I call
     SinEvalPts2 instead of SinEvalPts, etc etc
  */


  /* generate seed for random number generator */
  time ( &seed ) ;
  srand48( seed ) ;

  /* precompute sines and cosines appropriate for making the
     wigners */

  SinEvalPts( n, sinPts ) ;
  CosEvalPts( n, cosPts ) ;
  SinEvalPts2( n, sinPts2 ) ;
  CosEvalPts2( n, cosPts2 ) ;

  /* make quadrature weights */
  makeweights2( bw, weights );

  /* make the wigners */
  genWig_L2( m1, m2, bw,
	     sinPts, cosPts,
	     sinPts2, cosPts2,
	     wigners, scratch ) ;

  /* now make the wigners - transpose version! */
  genWigTrans_L2( m1, m2, bw,
		  sinPts, cosPts,
		  sinPts2, cosPts2,
		  wignersTrans, scratch ) ;

  /** space for errors **/
  relerror = (double *) malloc(sizeof(double) * loops);
  curmax = (double *) malloc(sizeof(double) * loops);

  sum_error = 0.0 ;
  sum_relerror = 0.0 ;

  for ( i = 0 ; i < loops ; i ++ )
    {
      /* generate random coeffs */
      for( j = 0 ; j < (bw - m) ; j++ )
	coeffs[ j ] = drand48() ;
      
      /* turn on stop watch */
      tstartA = csecond () ;

      /* now synthesize */
      wigNaiveSynthesis( m1, m2, bw, coeffs,
			 wignersTrans, signal,
			 scratch ) ;
      tstopA = csecond () ;

      runtimeA += (tstopA - tstartA);

      tstartB = csecond () ;

      /* now analyze */
      wigNaiveAnalysis( m1, m2, bw, signal,
			wigners, weights,
			newcoeffs,
			scratch ) ;

      /* turn off stop watch */
      tstopB = csecond () ;

      runtimeB += (tstopB - tstartB);

      relerror[ i ] = 0.0 ;
      curmax[ i ] = 0.0 ;
      /* now figure out errors */
      for( j = 0 ; j < bw - m ; j ++ )
	{
	  tmp_error = fabs( coeffs[j] - newcoeffs[j] );
	  tmp_relerror = tmp_error / ( fabs( coeffs[j] ) +
				       pow( 10.0, -50.0 ) );
	  curmax[ i ] = MAX( curmax[ i ], tmp_error );
	  relerror[ i ] = MAX( relerror[ i ], tmp_relerror );
	}
      sum_error += curmax[ i ] ;
      sum_relerror += relerror[ i ] ;
    }


  ave_error = sum_error / ( (double) loops );
  ave_relerror = sum_relerror / ( (double) loops );
  stddev_error = 0.0 ; stddev_relerror = 0.0;
  for( i = 0 ; i < loops ; i ++ )
    {
      stddev_error += pow( ave_error - curmax[ i ] , 2.0 );
      stddev_relerror += pow( ave_relerror - relerror[ i ] , 2.0 );
    }
  /*** this won't work if loops == 1 ***/
  if( loops != 1 )
    {
      stddev_error = sqrt(stddev_error / ( (double) (loops - 1) ) );
      stddev_relerror = sqrt(stddev_relerror / ( (double) (loops - 1) ) );
    }


  fprintf(stderr,"bw = %d\tm1 = %d\tm2 = %d\n",bw, m1, m2);
  fprintf(stderr,"total runtime: %.4e seconds\n", runtimeA+runtimeB);

  fprintf(stderr,"average forward runtime: %.4e seconds per iteration\n",
	  runtimeB/((double) loops));
  fprintf(stderr,"average inverse runtime: %.4e seconds per iteration\n",
	  runtimeA/((double) loops));


  fprintf(stderr,"Average r-o error:\t\t %.4e\t",
	  sum_error/((double) loops));
  fprintf(stderr,"std dev: %.4e\n",stddev_error);
  fprintf(stderr,"Average (r-o)/o error:\t\t %.4e\t",
	  sum_relerror/((double) loops));
  fprintf(stderr,"std dev: %.4e\n\n",stddev_relerror);



  if ( argc == 6 )
    {
      fp = fopen(argv[5], "w");
      for ( i = 0 ; i < bw - m ; i ++ )
	fprintf(fp,"%.16f\n", coeffs[i] - newcoeffs[i]);
    }


  free( curmax ) ;
  free( relerror ) ;
  free( workspace ) ;
  free( wignersTrans ) ;
  free( wigners ) ;
  free( signal ) ;
  free( newcoeffs ) ;
  free( coeffs ) ;
  free( weights ) ;

  return 0 ;
}
