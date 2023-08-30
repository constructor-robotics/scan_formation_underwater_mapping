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

  a routine to see if the C function wigSpec actually works.

  input: orders m1, m2
         bandwidth bw,
	 name of output file (to write function values)


  Generates the wigner-d's necessary to initialize the 3-term
  recurrence at orders m1, m2, and bandwidth

  example: test_wigSpec m1 m2 bw fileName

  example: test_wigSpec 3 4 16 d34.dat


*/

#include <stdio.h>
#include <stdlib.h>
#include "utils_so3.h"
#include "makeWigner.h"


int main ( int argc ,
	   char **argv )
{
  int i, m1, m2, bw, n ;
  double *workspace ;
  double *sinPts, *cosPts, *result ;
  FILE *fp ;

  
  if (argc < 5)
    {
      fprintf(stdout,"Usage: test_wigSpec m1 m2 bw output_file_name\n");
      exit(0);
    }

  m1 = atoi( argv[1] );
  m2 = atoi( argv[2] );
  bw = atoi( argv[3] );

  n = 2 * bw ;

  workspace = (double *) malloc(sizeof( double ) * 3 * n ) ;
  sinPts = workspace ;
  cosPts = sinPts + n ;
  result = cosPts + n ;

  /* note that the definition of wigSpec requires that instead of
     evaluating at beta, I need to evaluate at beta/2; ergo I call
     SinEvalPts2 instead of SinEvalPts, etc etc
  */

  SinEvalPts2( n, sinPts ) ;
  CosEvalPts2( n, cosPts ) ;

  wigSpec_L2( m1, m2, sinPts, cosPts, n, result ) ;

  fp = fopen( argv[4], "w" );
  for ( i = 0 ; i < n ; i++ )
    fprintf(fp, "%.15f\n", result[i]);
  fclose( fp ) ;

  free( workspace ) ;

  return 0 ;
}
