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

  a routine to see if the C function wigNaiveSynthesis actually works.

  for orders m1, m2, let l = max(|m1|, |m2|)
  let bw = bandwidth

  this routine should perform the inverse Wigner transform.

  Let l = Max(|m1|, |m2|). In matrix-lingo, wigNaiveAnalysis may be
  written as:
  
  c = P W f
  
  where f is the data vector, W is the quadrature matrix (i.e. weights),
  P is the (bw-l) x n matrix of sample values of the wigners
  d_{m1,m2}^l d_{m1,m2}^{l+1} ... d_{m1,m2}^{bw-1}, and c is the
  wigner series representation (i.e. coefficients) of f (c is
  a vector of length bw-l).
  
  So wigNaiveSynthesis can be written as
  
  f = Transpose(P) c
  
  No quadrature matrix is necessary.

  input: orders m1, m2
         bandwidth bw,
	 name of file containing coefficients (length bw-l)
	 name of output file (to write out the n=2*bw sample values)


  example: test_Wigner_Synthesis 3 4 16 coeffs.dat splat.dat


*/

#include <stdio.h>
#include <stdlib.h>
#include "utils_so3.h"
#include "makeWigner.h"
#include "wignerTransforms.h"


int main ( int argc ,
	   char **argv )
{
  int i, m1, m2, bw, n ;
  int m ;
  double *coeffs, *result, *wignersTrans ;
  double *workspace, *scratch ;
  double *sinPts, *cosPts ;
  double *sinPts2, *cosPts2 ;
  FILE *fp ;

  
  if (argc < 6)
    {
      fprintf(stdout,"Usage: test_Wigner_Synthesis m1 m2 bw input_file output_file\n");
      exit(0);
    }

  m1 = atoi( argv[1] );
  m2 = atoi( argv[2] );
  bw = atoi( argv[3] );

  m = MAX( ABS( m1 ) , ABS( m2 ) ) ;

  n = 2 * bw ;


  coeffs = ( double * ) malloc(sizeof( double ) * (bw - m) ) ;
  result = ( double * ) malloc(sizeof( double ) * n ) ;
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


  /* read in coefficients */
  fp = fopen( argv[4] , "r") ;
  for ( i = 0 ; i < (bw - m) ; i ++ )
    fscanf( fp, "%lf", coeffs + i );
  fclose( fp ) ;


  /* precompute sines and cosines appropriate for making the
     wigners */

  SinEvalPts( n, sinPts ) ;
  CosEvalPts( n, cosPts ) ;
  SinEvalPts2( n, sinPts2 ) ;
  CosEvalPts2( n, cosPts2 ) ;

  /* now make the wigners - transpose version! */
  genWigTrans_L2( m1, m2, bw,
		  sinPts, cosPts,
		  sinPts2, cosPts2,
		  wignersTrans, scratch ) ;


  /* now analyze */
  wigNaiveSynthesis( m1, m2, bw, coeffs,
		     wignersTrans, result,
		     scratch ) ;

  fp = fopen( argv[5], "w" );
  for ( i = 0 ; i <  n ; i++ )
    fprintf(fp, "%.15f\n", result[i]);
  fclose( fp ) ;

  free( workspace ) ;
  free( wignersTrans ) ;
  free( result ) ;
  free( coeffs ) ;

  return 0 ;
}
