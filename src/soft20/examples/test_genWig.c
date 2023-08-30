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

  a routine to see if the C function genWig_L2 actually works.

  for orders m1, m2, let l = max(|m1|, |m2|)
  let bw = bandwidth

  this routine should generate all the Wigner functions

  d_{m1,m2}^l, d_{m1,m2}^{l+1}, ..., d_{m1,m2}^{bw-1}

  evaluated at the n = 2*bw many points

  pi*(2*[0..n-1] + 1)/(2 n)


  input: orders m1, m2
         bandwidth bw,
	 name of output file (to write function values)

  example: test_genWig m1 m2 bw outputFileName

  example: test_genWig 3 4 16 d34.dat


*/

#include <stdio.h>
#include <stdlib.h>
#include "utils_so3.h"
#include "makeWigner.h"

int main ( int argc ,
	   char **argv )
{
  int i, m1, m2, bw, n ;
  int m ;
  double *workspace, *scratch ;
  double *sinPts, *cosPts, *result ;
  double *sinPts2, *cosPts2 ;
  FILE *fp ;

  
  if (argc < 5)
    {
      fprintf(stdout,"Usage: test_genWig m1 m2 bw output_file_name\n");
      exit(0);
    }

  m1 = atoi( argv[1] );
  m2 = atoi( argv[2] );
  bw = atoi( argv[3] );

  m = MAX( ABS( m1 ) , ABS( m2 ) ) ;

  n = 2 * bw ;

  result = ( double * ) malloc(sizeof( double ) * n * ( bw - m ) ) ;
  workspace = (double *) malloc(sizeof( double ) * (4 + 6) * n ) ;
  sinPts = workspace ;
  cosPts = sinPts + n ;
  sinPts2 = cosPts + n ;
  cosPts2 = sinPts2 + n ;
  scratch = cosPts2 + n ; /* scratch needs to be of size 6*n */

  /* 
     Compute appropriate sines and cosines at Chebyshev points
     (or their slight variants)

     note that the definition of wigSpec requires that instead of
     evaluating at beta, I need to evaluate at beta/2; ergo I call
     SinEvalPts2 instead of SinEvalPts, etc etc
  */

  SinEvalPts( n, sinPts ) ;
  CosEvalPts( n, cosPts ) ;
  SinEvalPts2( n, sinPts2 ) ;
  CosEvalPts2( n, cosPts2 ) ;


  genWig_L2( m1, m2, bw,
	     sinPts, cosPts,
	     sinPts2, cosPts2,
	     result, scratch ) ;

  fp = fopen( argv[4], "w" );
  for ( i = 0 ; i < n*(bw-m) ; i++ )
    fprintf(fp, "%.15f\n", result[i]);
  fclose( fp ) ;

  free( workspace ) ;
  free( result ) ;

  return 0 ;
}
