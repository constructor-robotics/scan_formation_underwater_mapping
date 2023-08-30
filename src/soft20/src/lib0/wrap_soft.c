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

#include <stdio.h>
#include <stdlib.h>
#include "soft.h"


/***********

 Forward_SO3_Naive_W: wrapped version of Forward_SO3_Naive

 - given an input signal, will compute its SO(3) Fourier coefficients

 bw - bandwidth of input - MUST BE A POWER OF 2

 rsignal, isignal: double ptrs to real/imaginary parts of the input;
                   for bandwidth bw, then, each is a pointer to a
		   double array of size (2*bw)^3

 rcoeffs, icoeffs: double ptrs to arrays, each of size
                   (4*bw^3 - bw)/3; will contain the real/imaginary
		   parts of the computed Fourier coefficients

 - read soft_fx.pdf (included within this distribution) for how
   the function samples and coefficients are arranged

*************/

void Forward_SO3_Naive_W( int bw,
			  double *rsignal, double *isignal,
			  double *rcoeffs, double *icoeffs )
{
  int n, n3 ;
  double *workspace1, *workspace2 ;
  
  n = 2 * bw ;
  n3 = n * n * n ;

  /* LOTS OF workspace */
  workspace1 = ( double * ) malloc(sizeof( double ) * 4 * n3 ) ;
  workspace2 = ( double * ) malloc(sizeof( double ) * (26*bw + 2*bw*bw) );

  /* check if any problems allocating memory */
  if ( ( workspace1 == NULL ) || ( workspace2 == NULL ) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }
  
  /* now do the forward transform */
  Forward_SO3_Naive( bw,
		     rsignal, isignal,
		     rcoeffs, icoeffs,
		     workspace1, workspace2 ) ;
  
  /* free up memory (and there's lots of it) */
  free( workspace2 );
  free( workspace1 );

}

/*********************************************************/
/*********************************************************/

/***********

 Inverse_SO3_Naive_W: wrapped version of Inverse_SO3_Naive

 - given the Fourier coefficients of a function defined on
   SO(3), perform the inverse transform to obtain the samples

 bw - bandwidth

 rcoeffs, icoeffs: double ptrs to input arrays, each of size
                   (4*bw^3 - bw)/3, containing the real/imaginary
		   parts of the Fourier coefficients

 rsignal, isignal: double ptrs to arrays, each of size
                   (2*bw)^3; will contain the real/imaginary
		   parts of the computed function samples

 - read soft_fx.pdf (included within this distribution) for how
   the function samples and coefficients are arranged

*************/

void Inverse_SO3_Naive_W( int bw,
			  double *rcoeffs, double *icoeffs,
			  double *rsignal, double *isignal )
{
  int n, n3 ;
  double *workspace1, *workspace2 ;
  
  n = 2 * bw ;
  n3 = n * n * n ;

  /* LOTS OF workspace */
  workspace1 = ( double * ) malloc(sizeof( double ) * 2 * n3 ) ;
  workspace2 = ( double * ) malloc(sizeof( double ) * (24*bw + 2*bw*bw) );

  /* check if any problems allocating memory */
  if ( ( workspace1 == NULL ) || ( workspace2 == NULL ) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }
  

  /* now do the inverse transform */
  Inverse_SO3_Naive( bw,
		     rcoeffs, icoeffs,
		     rsignal, isignal,
		     workspace1, workspace2 ) ;
  
  /* free up memory (and there's lots of it) */
  free( workspace2 );
  free( workspace1 );

}
