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

  test program to loop through inverse-forward SO3 transforms
  lots of times to run error checks.

  - uses routines that write over inputs

  - uses FFTW and wigner-d symmetries

  spectral - spatial - spectral

  input: - bandwidth bw
         - loops
  	 - output file containing real and imaginary parts of errors

  example: test_soft_fftw_nt bw loops errors

  example: test_soft_fftw_nt 16 10 error.dat



*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "fftw3.h"
#include "makeweights.h"
#include "utils_so3.h"
#include "soft_fftw_nt.h"
#include "csecond.h"

int main( int argc,
	  char **argv )

{
  int l, k, j, bw, n, n3 ;
  int loops ;
  fftw_complex *signal ;
  fftw_complex *coeffs ;
  fftw_complex *coeffsCopy ;
  fftw_complex *workspace_cx ;
  fftw_plan planF, planI ;
  double *weights ;
  int m1, m2 ;
  // int cl, cl2 ;
  // double fudge ;
  double *workspace_re ;
  double tstartF, tstopF, runtimeF ;
  double tstartI, tstopI, runtimeI ;
  double total_time ;
  double ave_error;
  double ave_relerror;
  double stddev_error, stddev_relerror;
  double *relerror;
  double *curmax ;
  double granderror, grandrelerror;
  double realtmp, imagtmp ;
  double origmag, tmpmag ;
  fftw_iodim dims[2], howmany_dims[2];
  int howmany_rank, rankY;
  int tmpInt ;
  long int seed ;
  FILE *fp ;
  
  if (argc < 3)
    {
      fprintf(stdout, "Usage: test_soft_fftw_nt bw loops ");
      fprintf(stdout, "[error_file]\n");
      exit(0);
    }

  bw = atoi( argv[1] );
  loops = atoi( argv[2] );
  n = 2 * bw ;
  n3 = n * n * n ;

  /* signal */
  signal = fftw_malloc( sizeof( fftw_complex ) * n3 ) ;

  /* coefficients totalCoeffs( bw ) amount of space */
  coeffs = fftw_malloc(sizeof( fftw_complex ) * totalCoeffs_so3( bw ) ) ;

  /* another coefficients array, to copy the original before
     the inverse transform writes over it - need a copy for
     determining the error */
  coeffsCopy = fftw_malloc(sizeof( fftw_complex ) * totalCoeffs_so3( bw ) ) ;

  /* now for workspace */
  workspace_cx = fftw_malloc(sizeof( fftw_complex ) * n3 ) ;
  workspace_re = ( double * ) malloc(sizeof( double ) *
				     ( 24 * bw + 2 * bw * bw) );
  /** space for errors **/
  relerror = (double *) malloc(sizeof(double) * loops);
  curmax = (double *) malloc(sizeof(double) * loops);

  /* space for weights */
  weights = (double *) malloc(sizeof(double) * (2*bw));


  /* check if any problems allocating memory */
  if ( ( signal == NULL ) || ( coeffs == NULL ) ||
       ( coeffsCopy == NULL ) || ( workspace_cx == NULL ) ||
       ( workspace_re == NULL ) ||
       ( relerror == NULL ) || ( curmax == NULL ) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }
  
  /* create plans */
  /* plan for FORWARD SO(3) transform */
  howmany_rank = 1 ;
  howmany_dims[0].n =  n ;
  howmany_dims[0].is = n*n ;
  howmany_dims[0].os = 1 ;
  
  rankY = 2 ;  
  dims[0].n = n ;
  dims[0].is = n ;
  dims[0].os = n*n ;
  
  dims[1].n = n ;
  dims[1].is = 1 ;
  dims[1].os = n ;
  
  planF = fftw_plan_guru_dft( rankY, dims,
			      howmany_rank, howmany_dims,
			      signal, workspace_cx,
			      1, FFTW_MEASURE ); 
  
  /* plan for INVERSE SO(3) transform */ 
  howmany_rank = 1 ;
  howmany_dims[0].n =  n ;
  howmany_dims[0].is = 1 ;
  howmany_dims[0].os = n*n ;
  
  rankY = 2 ;  
  dims[0].n = n ;
  dims[0].is = n*n ;
  dims[0].os = n ;
  
  dims[1].n = n ;
  dims[1].is = n ;
  dims[1].os = 1 ;
  
  /* NOTE: you can make the fft_plan for the inverse SO(3) transform
     to be IN_PLACE, i.e. replace "workspace_cx" with "signal" immediately
     below */
  planI = fftw_plan_guru_dft( rankY, dims,
 			      howmany_rank, howmany_dims,
 			      signal, workspace_cx,
 			      -1, FFTW_MEASURE ); 
 
  /* make weights */
  makeweights2( bw, weights );

  /* generate seed for random number generator */
  time ( &seed ) ;
  srand48( seed ) ;

  /* initialize error */
  granderror = 0.0 ;
  grandrelerror = 0.0 ;

  /* initialize time */
  runtimeF = 0.0 ;
  runtimeI = 0.0 ;

  fprintf(stderr,"About to enter for-loop\n");

  for( k = 0 ; k < loops ;  k ++ )
    {
      /* generate random coefficients */
      tmpInt = totalCoeffs_so3( bw ) ;
      for( l = 0 ; l < tmpInt ; l++ )
	{
	  coeffs[ l ][0] = 2.0 * ( drand48() - 0.5 ) ;
	  coeffs[ l ][1] = 2.0 * ( drand48() - 0.5 ) ;
	}

      /*
	generate random coefficients so that
	the signal is double
      */
      /*
	for ( l = 0 ; l < bw ; l ++ )
	{
	for ( m1 = -l ; m1 < l + 1 ; m1 ++ )
	for ( m2 = -l ; m2 < 1 ; m2 ++ )
	{
	cl = so3CoefLoc( m1, m2, l, bw );
	coeffs[ cl ][0] =  2.0 * ( drand48() - 0.5 ) ;
	coeffs[ cl ][1] =  2.0 * ( drand48() - 0.5 ) ;

	if ( ((ABS(m1)+ABS(m2)) % 2) == 0 )
	fudge = 1. ;
	else
	fudge = -1. ;
		
	cl2 = so3CoefLoc( -m1, -m2, l, bw );
	coeffs[ cl2 ][0] = fudge*coeffs[ cl ][0]  ;
	coeffs[ cl2 ][1] = -fudge*coeffs[ cl ][1]  ;
	}
	cl = so3CoefLoc(0, 0, l, bw);
	coeffs[ cl ][0] = 2.0 * ( drand48() - 0.5 ) ;
	coeffs[ cl ][1] = 0 ;
	}
      */

      /* now copy the coefficients (need the originals so that
	 can compute the error) */
      memcpy(coeffsCopy, coeffs, sizeof(fftw_complex)*totalCoeffs_so3(bw));


      /*
	Whew! Generated the coefficients. Can now do the transforming.
      */

      /* turn on stopwatch */
      tstartI = csecond( ) ;

      /* now do inverse transform */
      Inverse_SO3_Naive_fftw_nt( bw,
				 coeffs,
				 signal,
				 workspace_cx,
				 workspace_re,
				 &planI,
				 0 ) ;

      /* turn off stopwatch */
      tstopI = csecond( ) ;
      runtimeI += tstopI - tstartI ; 
      fprintf(stderr,"inv time \t = %.4e\n", tstopI - tstartI);

      /* turn on stopwatch */
      tstartF = csecond( ) ;

      /* now do the forward transform */
      Forward_SO3_Naive_fftw_nt( bw,
				 signal,
				 coeffs,
				 workspace_cx,
				 workspace_re,
				 weights,
				 &planF,
				 0 );
      
      /* turn off stopwatch */
      tstopF = csecond( ) ;
      runtimeF += tstopF - tstartF ;
      fprintf(stderr,"for time \t = %.4e\n", tstopF - tstartF);

      relerror[ k ] = 0.0 ;
      curmax[ k ] = 0.0 ;
      /* now figure out errors */
      for( j = 0 ; j < tmpInt ; j ++ )
	{
	  realtmp = coeffsCopy[ j ][0] - coeffs[ j ][0] ;
	  imagtmp = coeffsCopy[ j ][1] - coeffs[ j ][1] ;
	  origmag = sqrt((coeffsCopy[ j ][0]*coeffsCopy[ j ][0]) +
			 (coeffsCopy[ j ][1]*coeffsCopy[ j ][1]));
	  tmpmag = sqrt((realtmp*realtmp) +
			(imagtmp*imagtmp));
	  relerror[ k ] =
	    MAX(relerror[ k ] , tmpmag/(origmag + pow(10.0, -50.0)));
	  curmax[ k ] = MAX( curmax[ k ], tmpmag );
	}

    
      fprintf(stderr,"r-o error\t = %.12f\n", curmax[ k ]);
      fprintf(stderr,"(r-o)/o error\t = %.12f\n\n", relerror[ k ]);
      
      granderror += curmax[ k ];
      grandrelerror += relerror[ k ];
    }


  total_time = runtimeF + runtimeI ;

  ave_error = granderror / ( (double) loops );
  ave_relerror = grandrelerror / ( (double) loops );
  stddev_error = 0.0 ; stddev_relerror = 0.0;
  for( k = 0 ; k < loops ; k ++ )
    {
      stddev_error += pow( ave_error - curmax[ k ] , 2.0 );
      stddev_relerror += pow( ave_relerror - relerror[ k ] , 2.0 );
    }
  /*** this won't work if loops == 1 ***/
  if( loops != 1 )
    {
      stddev_error = sqrt(stddev_error / ( (double) (loops - 1) ) );
      stddev_relerror = sqrt(stddev_relerror / ( (double) (loops - 1) ) );
    }


  fprintf(stderr,"Program: test_soft_fftw_nt\n");
  fprintf(stderr,"Bandwidth = %d\n", bw);

#ifndef WALLCLOCK
  fprintf(stderr,"Total elapsed cpu time :\t\t %.4e seconds.\n",
	  total_time);
  fprintf(stderr,"Average cpu forward per iteration:\t %.4e seconds.\n",
	  runtimeF/((double) loops));  
  fprintf(stderr,"Average cpu inverse per iteration:\t %.4e seconds.\n",
	  runtimeI/((double) loops));
#else
  fprintf(stderr,"Total elapsed wall time :\t\t %.4e seconds.\n",
	  total_time);
  fprintf(stderr,"Average wall forward per iteration:\t %.4e seconds.\n",
	  runtimeF/((double) loops));  
  fprintf(stderr,"Average wall inverse per iteration:\t %.4e seconds.\n",
	  runtimeI/((double) loops));
#endif

  fprintf(stderr,"Average r-o error:\t\t %.4e\t",
	  granderror/((double) loops));
  fprintf(stderr,"std dev: %.4e\n",stddev_error);
  fprintf(stderr,"Average (r-o)/o error:\t\t %.4e\t",
	  grandrelerror/((double) loops));
  fprintf(stderr,"std dev: %.4e\n\n",stddev_relerror);


  /* if an error file is asked for (probably just after one loop) */
  if( argc > 3 )
    {
      fp = fopen( argv[ 3 ] , "w" );
      for ( l = 0 ; l < bw ; l ++ )
	for ( m1 = -l ; m1 < l + 1 ; m1 ++ )
	  for ( m2 = -l ; m2 < l + 1 ; m2 ++ )
	    {
	      k = so3CoefLoc( m1, m2, l, bw ) ;
	      fprintf( fp, "l = %d  m1 = %d  m2 = %d  %.15f\t%.15f\n",
		       l, m1, m2,
		       coeffsCopy[ k ][0] - coeffs[ k ][0],
		       coeffsCopy[ k ][1] - coeffs[ k ][1] );
	    }
      fclose( fp ) ;
    }

  /* free up memory (and there's lots of it) */
  fftw_destroy_plan( planI );
  fftw_destroy_plan( planF );

  free( weights );
  free( curmax );
  free( relerror );

  free( workspace_re );

  fftw_free( workspace_cx );
  fftw_free( coeffsCopy );
  fftw_free( coeffs );
  fftw_free( signal );

  return 0 ;
}
