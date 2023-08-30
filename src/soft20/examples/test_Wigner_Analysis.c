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

  a routine to see if the C function wigNaiveAnalysis actually works.

  for orders m1, m2, let l = max(|m1|, |m2|)
  let bw = bandwidth

  this routine should project a signal of length 2*bw onto
  all the Wigner functions

  d_{m1,m2}^l, d_{m1,m2}^{l+1}, ..., d_{m1,m2}^{bw-1}

  In other words, it will determine the coefficients
  < signal, d_{m1,m2}^{j} > for
  j = l ... bw - 1


  input: orders m1, m2
         bandwidth bw,
	 name of file containing sample values (length 2*bw)
	 name of output file (to write out the coefficients)


  example: test_Wigner_Analysis 3 4 16 signal.dat splat.dat


*/

#include <stdio.h>
#include <stdlib.h>
#include "utils_so3.h"
#include "makeWigner.h"
#include "makeweights.h"
#include "wignerTransforms.h"


int main ( int argc ,
	   char **argv )
{
  int i, m1, m2, bw, n ;
  int m ;
  double *signal, *result, *wigners ;
  double *workspace, *scratch ;
  double *weights ;
  double *sinPts, *cosPts ;
  double *sinPts2, *cosPts2 ;
  FILE *fp ;

  
  if (argc < 6)
    {
      fprintf(stdout,"Usage: test_Wigner_Analysis m1 m2 bw input_file output_file\n");
      exit(0);
    }

  m1 = atoi( argv[1] );
  m2 = atoi( argv[2] );
  bw = atoi( argv[3] );

  m = MAX( ABS( m1 ) , ABS( m2 ) ) ;

  n = 2 * bw ;

  weights = (double * ) malloc(sizeof(double) * n ) ;
  signal = ( double * ) malloc(sizeof( double ) * n ) ;
  result = ( double * ) malloc(sizeof( double ) * ( bw - m ) ) ;
  wigners = ( double * ) malloc( sizeof( double ) * ( bw - m ) * n ) ;
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


  /* read in sample values */
  fp = fopen( argv[4] , "r") ;
  for ( i = 0 ; i < n ; i ++ )
    fscanf( fp, "%lf", signal + i );
  fclose( fp ) ;


  /* precompute sines and cosines appropriate for making the
     wigners */

  SinEvalPts( n, sinPts ) ;
  CosEvalPts( n, cosPts ) ;
  SinEvalPts2( n, sinPts2 ) ;
  CosEvalPts2( n, cosPts2 ) ;

  /* make quadrature weights */
  makeweights2( bw, weights );

  /* now make the wigners */
  genWig_L2( m1, m2, bw,
	     sinPts, cosPts,
	     sinPts2, cosPts2,
	     wigners, scratch ) ;


  /* now analyze */
  wigNaiveAnalysis( m1, m2, bw, signal,
		    wigners, weights,
		    result,
		    scratch ) ;

  fp = fopen( argv[5], "w" );
  for ( i = 0 ; i <  (bw-m) ; i++ )
    fprintf(fp, "%.15f\n", result[i]);
  fclose( fp ) ;

  free( workspace ) ;
  free( wigners ) ;
  free( result ) ;
  free( signal ) ;
  free( weights ) ;

  return 0 ;
}
