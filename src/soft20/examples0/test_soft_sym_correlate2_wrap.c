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
  to test the correlation routines

  - uses the Wigner-d symmetries
  - uses part of SpharmonicKit
  - STRICTLY double SAMPLES of signal and pattern files
  - [result] -> optional -> filename of all the correlation values
                (if you want all of them)
  - bw -> bw of spherical signals
  - isReal - whether data is strictly real (flag = 1), or interleaved ( 0 )

  example: test_soft_sym_correlate2_wrap signalFile patternFile bw isReal

*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "wrap.h"

int main ( int argc,
	   char **argv )
{
  FILE *fp ;
  int i, isReal ;
  int n, bw ;
  double *signal, *pattern ;
  double alpha, beta, gamma ;

  if (argc < 5 )
    {
      printf("test_soft_sym_correlate2_wrap signalFile patternFile bw isReal\n ");
      exit(0) ;
    }

  bw = atoi( argv[3] );
  n = 2 * bw ;
  isReal = atoi( argv[4] );

  /* allocate space to hold signal, pattern */
  if ( isReal )
    {
      signal = (double *) malloc( sizeof(double) * (n * n) );
      pattern = (double *) malloc( sizeof(double) * (n * n) );
    }
  else
    {
      signal = (double *) malloc( sizeof(double) * (2 * n * n) );
      pattern = (double *) malloc( sizeof(double) * (2 * n * n) );
    }
 
  /****
       At this point, check to see if all the memory has been
       allocated. If it has not, there's no point in going further.
  ****/

  if ( (signal == NULL) || (pattern == NULL) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }

  printf("Reading in signal file\n");
  /* read in SIGNAL samples */
  fp = fopen(argv[1],"r");
  if ( isReal )
    {
      for ( i = 0 ; i < n * n ; i ++ )
	fscanf(fp,"%lf", signal + i);
    }
  else
    {
      for ( i = 0 ; i < 2 * n * n ; i ++ )
	fscanf(fp,"%lf", signal + i);
    }

  fclose( fp );

  printf("Reading in pattern file\n");
  /* read in PATTERN samples */
  fp = fopen(argv[2],"r");
  if ( isReal )
    {
      for ( i = 0 ; i < n * n ; i ++ )
	fscanf(fp,"%lf", pattern + i);
    }
  else
    {
      for ( i = 0 ; i < 2 * n * n ; i ++ )
	fscanf(fp,"%lf", pattern + i);
    }

  fclose( fp );

  /* now correlate */
  softSymCor2( bw,
	       signal,
	       pattern,
	       &alpha, &beta, &gamma,
	       isReal ) ;

  /* print results */
  printf("alpha = %f\nbeta = %f\ngamma = %f\n",
	 alpha, beta, gamma );

  /* clean up */
  free( pattern );
  free( signal ) ;

  return 0 ;

}
