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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>



#include "so3_correlate_sym.h"
#include "soft_sym.h"

#include "FST_semi_memo.h"
#include "cospmls.h"
#include "s2_primitive.h"
#include "legendreTransforms.h"

#define NORM( x ) ( (x[0])*(x[0]) + (x[1])*(x[1]) )

/****************************************

 softSymCor2: simple wrapper for correlating two STRICTLY double-VALUED
              functions defined on the sphere; does NOT use fftw;
              if efficiency is important to you, or want more control,
              e.g. want to correlate lots and lots of times without having
	      reallocate tmp workspace, or change the bandwidth
	      you want to correlate at, or correlate complex-valued
              functions, you should look at

	      test_soft_sym_correlate2.c

	      as an example of how to do it. softSymCor2() is basically
	      test_soft_sym_correlate2.c turned into a wrapper, with
	      some simplifying assumptions.
	      
  bw: bandwidth of signal and pattern


  isReal: int defining whether or not the signal and pattern are
          strictly real, or interleaved (complex)
          = 1 -> strictly real
          = 0 -> complex/interleaved

  sig: double ptr to SIGNAL function samples;
       for bandwidth bw, then, is a pointer to a
       double array of size (2*bw)^3 + (isReal*(2*bw)^3)

  pat: double ptr to PATTERN function samples
       for bandwidth bw, then, is a pointer to a
       double array of size (2*bw)^3 + (isReal*(2*bw)^3)

  alpha, beta, gamma: ptrs to doubles; at the end of the routine,
               will "contain" the angles alpha, beta, and gamma needed
	       in order to rotate the SIGNAL to match the PATTERN; the
	       order of rotation is:

                   1) rotate by gamma about the z-axis
                   2) rotate by beta about the y-axis
                   3) rotate by alpha about the z-axis.
		   
	       where
             
	           0 <= alpha, gamma < 2*pi
	           0 <= beta <= pi


***********************************/


void softSymCor2( int bw,
		  double *sig,
		  double *pat,
		  double *alpha, double *beta, double *gamma,
		  int isReal )
{
  int i ;
  int n, bwIn, bwOut, degLim ;
  double *tmpR, *tmpI ;
  double *workspace1, *workspace2  ;
  double *sigCoefR, *sigCoefI ;
  double *patCoefR, *patCoefI ;
  double *so3SigR, *so3SigI ;
  double *so3CoefR, *so3CoefI ;
  int tmp, maxloc, ii, jj, kk ;
  double tmpval, maxval ;
  double *seminaive_naive_tablespace  ;
  double **seminaive_naive_table ;

  bwIn = bw ;
  bwOut = bw ;
  degLim = bw - 1 ;
  n = 2 * bwIn ;

  tmpR = (double *) malloc( sizeof(double) * ( n * n ) );
  tmpI = (double *) malloc( sizeof(double) * ( n * n ) );
  so3SigR = (double *) malloc( sizeof(double) * (8*bwOut*bwOut*bwOut) );
  so3SigI = (double *) malloc( sizeof(double) * (8*bwOut*bwOut*bwOut) );
  workspace1 = (double *) malloc( sizeof(double) * (16*bwOut*bwOut*bwOut) );
  workspace2 = (double *) malloc( sizeof(double) * ((14*bwIn*bwIn) + (48 * bwIn)));
  sigCoefR = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
  sigCoefI = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
  patCoefR = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
  patCoefI = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
  so3CoefR = (double *) malloc( sizeof(double) * ((4*bwOut*bwOut*bwOut-bwOut)/3) ) ;
  so3CoefI = (double *) malloc( sizeof(double) * ((4*bwOut*bwOut*bwOut-bwOut)/3) ) ;

  seminaive_naive_tablespace =
    (double *) malloc(sizeof(double) *
		      (Reduced_Naive_TableSize(bwIn,bwIn) +
		       Reduced_SpharmonicTableSize(bwIn,bwIn)));

  /****
       At this point, check to see if all the memory has been
       allocated. If it has not, there's no point in going further.
  ****/

  if ( (seminaive_naive_tablespace == NULL) ||
       (tmpR == NULL) || ( tmpI == NULL) ||
       (so3CoefR == NULL) || (so3CoefI == NULL) ||
       (workspace1 == NULL) || (workspace2 == NULL) ||
       (sigCoefR == NULL) || (sigCoefI == NULL) ||
       (patCoefR == NULL) || (patCoefI == NULL) ||
       (so3CoefR == NULL) || (so3CoefI == NULL) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }

  seminaive_naive_table = SemiNaive_Naive_Pml_Table(bwIn, bwIn,
						    seminaive_naive_tablespace,
						    workspace2);

  /* load SIGNAL samples into temp array */
  if ( isReal )
    {
      for (i = 0 ; i < n * n ; i ++ )
	{
	  tmpR[i] = sig[i] ;
	  tmpI[i] = 0. ;
	}
    }
  else
   {
      for (i = 0 ; i < n * n ; i ++ )
	{
	  tmpR[i] = sig[2*i] ;
	  tmpI[i] = sig[2*i+1] ;
	}
    }

  /* spherical transform of SIGNAL */
  FST_semi_memo( tmpR, tmpI,
		 sigCoefR, sigCoefI,
		 n, seminaive_naive_table,
		 workspace2, isReal, bwIn ) ;

  /* load PATTERN samples into temp array */
  if ( isReal )
    {
      for (i = 0 ; i < n * n ; i ++ )
	{
	  tmpR[i] = pat[i] ;
	  tmpI[i] = 0. ;
	}
    }
  else
   {
      for (i = 0 ; i < n * n ; i ++ )
	{
	  tmpR[i] = pat[2*i] ;
	  tmpI[i] = pat[2*i+1] ;
	}
    }

  /* spherical transform of PATTERN */
  FST_semi_memo( tmpR, tmpI,
		 patCoefR, patCoefI,
		 n, seminaive_naive_table,
		 workspace2, isReal, bwIn ) ;

  /* all done with the spherical transform, so free up
     some memory before continuing */
  free( seminaive_naive_table ) ;
  free( seminaive_naive_tablespace ) ;


  /* combine coefficients */
  so3CombineCoef( bwIn, bwOut, degLim,
		  sigCoefR, sigCoefI,
		  patCoefR, patCoefI,
		  so3CoefR, so3CoefI ) ;

  /* now inverse so(3) */
  Inverse_SO3_Naive_sym( bwOut,
			 so3CoefR, so3CoefI,
			 so3SigR, so3SigI,
			 workspace1, workspace2,
			 isReal );

  /* now find max value */
  maxval = 0.0 ;
  maxloc = 0 ;
  for ( i = 0 ; i < 8*bwOut*bwOut*bwOut ; i ++ )
    {
      tmpval = (so3SigR[i]*so3SigR[i]) +
	(so3SigI[i]*so3SigI[i]);

      if (tmpval > maxval)
	{
	  maxval = tmpval ;
	  maxloc = i ;
	}
    }
  
  ii = floor( maxloc / (4.*bwOut*bwOut) );
  tmp = maxloc - (ii*4.*bwOut*bwOut);
  jj = floor( tmp / (2.*bwOut) );
  tmp = maxloc - (ii *4*bwOut*bwOut) - jj*(2*bwOut);
  kk = tmp ;

  *alpha = M_PI*jj/((double) bwOut) ;
  *beta =  M_PI*(2*ii+1)/(4.*bwOut) ;
  *gamma = M_PI*kk/((double) bwOut) ;

  free( so3CoefI );
  free( so3CoefR );
  free( patCoefI );
  free( patCoefR );
  free( sigCoefI );
  free( sigCoefR );
  free( workspace2 );
  free( workspace1 );
  free( so3SigI ) ;
  free( so3SigR ) ;
  free( tmpI );
  free( tmpR );

}
