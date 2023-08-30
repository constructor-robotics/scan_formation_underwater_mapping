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

  a routine to see if the C function genWig_L2_U actually works.

  for orders m1, m2, let l = max(|m1|, |m2|)
  let bw = bandwidth

  this routine should generate all the Wigner functions

  d_{m1,m2}^l, d_{m1,m2}^{l+1}, ..., d_{m1,m2}^{bw-1}

  evaluated at the angles (in RADIANS) a0, a1, ..., aN

  input: angles a0, a1, a2, ..., aN (in RADIANS)
         orders m1, m2
         bandwidth bw,
         flag: = 0 -> NOT L2-normalized Wigners
               = 1 -> L2-normalized Wigners
	 name of output file (to write function values)

  example: test_Wigner_angle m1 m2 bw flag outputFileName a0 a1 a2 ... aN

  example: test_Wigner_angle 3 4 16 1 d34.dat 1.57079632679490 0.5 0.1

  Format of output file:

     Let l = max(abs(m1), abs(m2))
     Let the number of angles = n+1: a0, a1, a2, a3, ..., aN
     The output file will have (bw-m)-many lines.
     Each line will consist of (n+1)-many numbers.

     The first line will consist of
     d_{m1,m2}^l(a0) d_{m1,m2}^l(a1) d_{m1,m2}^l(a2) ... d_{m1,m2}^l(aN)

     The second line will consist of
     d_{m1,m2}^{l+1}(a0) d_{m1,m2}^{l+1}(a1) d_{m1,m2}^{l+1}(a2) ... d_{m1,m2}^{l+1}(aN)

     ...

     The last line will consist of
     d_{m1,m2}^{bw-1}(a0) d_{m1,m2}^{bw-1}(a1) d_{m1,m2}^{bw-1}(a2) ... d_{m1,m2}^{bw-1}(aN)


*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "utils_so3.h"
#include "makeWigner.h"

int main ( int argc ,
	   char **argv )
{
  int i, j ;
  int m1, m2, bw ;
  int flag, n, m ;
  double *workspace, *scratch ;
  double *sinPts, *cosPts, *result ;
  double *sinPts2, *cosPts2 ;
  FILE *fp ;
  double *ang, fudge ;
  
  if (argc < 7)
    {
      fprintf(stdout,"Usage: test_Wigner_angle m1 m2 bw flag output_file_name a0 [ a1 ... aN ]\n");
      exit(0);
    }

  m1 = atoi( argv[1] );
  m2 = atoi( argv[2] );
  bw = atoi( argv[3] );
  flag = atoi( argv[4] );

  m = MAX( ABS( m1 ) , ABS( m2 ) ) ;
  n  = argc - 6 ;  // n = number of angles to evaluate at

  ang = (double *) malloc(sizeof( double ) * n );
  result = ( double * ) malloc(sizeof( double ) * n * ( bw - m ) ) ;
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

  /* read in angles from command line */
  for ( i = 0 ; i < n ; i ++ )
    ang[i] = atof( argv[6+i] );

  /*
    evaluate trig functions at ang0, ang1, ...
  */
  for( i = 0 ; i < n ; i++ )
    {
      sinPts[i] = sin(ang[i]);
      cosPts[i] = cos(ang[i]);
      sinPts2[i] = sin(0.5*ang[i]);
      cosPts2[i] = cos(0.5*ang[i]);
    }

  genWig_L2_U( m1, m2, bw, n,
	       sinPts, cosPts,
	       sinPts2, cosPts2,
	       result, scratch ) ;

  /* write  d_{m1,m2}^l, d_{m1,m2}^{l+1}, ..., d_{m1,m2}^{bw-1},
     evaluated at angs, to disk */

  fp = fopen( argv[5], "w" );
  for ( i = 0 ; i < (bw-m) ; i++ )
    {
      fudge = 1/sqrt((double)(2*(m+i)+1)/2.0);
      for ( j = 0 ; j < n ; j ++ )
	{
	  if ( result[i*n+j] < 0 )
	    {
	      if ( flag )
		fprintf(fp,"%.15f  ", result[i*n+j]);
	      else
		fprintf(fp,"%.15f  ", result[i*n+j]*fudge);
	    }
	  else
	    {
	      if ( flag )
		fprintf(fp," %.15f  ", result[i*n+j]);
	      else
		fprintf(fp," %.15f  ", result[i*n+j]*fudge);
	    }
	}
      fprintf(fp,"\n") ;
    }
  fclose( fp ) ;

  free( workspace ) ;
  free( result ) ;
  free( ang ) ;

  return 0 ;
}
