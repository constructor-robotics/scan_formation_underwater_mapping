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

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "FST_semi_memo.h"
#include "cospmls.h"
#include "s2_primitive.h"
#include "legendreTransforms.h"
#include "rotate_so3.h"


/***********************

 s2Rotate: wrapper function version of

           test_s2_rotate.c

 will rotate a possibly complex-valued function defined on the sphere by
 the Euler angles alpha, beta, gamma, where

	           0 <= alpha, gamma < 2*pi
	           0 <= beta <= pi

 and in the following order

                   1) rotate by gamma about the z-axis
                   2) rotate by beta about the y-axis
                   3) rotate by alpha about the z-axis.
		   
 bw: bandwidth of function - MUST BE A POWER OF 2



 sigIn : ptr to the input signal; if complex, then it's an interleaved
        array of length 2*(2*bw)^2; if strictly real, then it's an
        array of length (2*bw)^2

 sigOut : ptr to the rotated signal; if complex, then it's an interleaved
          array of length 2*(2*bw)^2; if strictly real, then it's an
          array of length (2*bw)^2

 alpha, beta, gamma: doubles specifying the Euler angles.

 isReal: = 1 data is strictly real
         = 0 data is complex (interleaved format)

 alpha, beta, gamma: doubles specifying the Euler angles.


 NOTE: Sometimes there is a segmentation fault *after* all the rotating and
 writing out of the output file is complete. I haven't tracked this down yet,
 but I believe it has to do with freeing up the memory associated with doing
 the S^2 transforms ... my array of double pointers are not pointing in the
 right places when I try to free memory. However, the rotation itself is
 correct.

****************/


void s2Rotate( int bw,
	       double *sigIn,
	       double *sigOut,
	       double alpha, double beta, double gamma,
	       int isReal )
{
  int i, splat ;
  double *scratch ;
  double *tmpInR, *tmpInI, *tmpOutR, *tmpOutI;
  double *seminaive_naive_tablespace;
  double *trans_seminaive_naive_tablespace;
  double **seminaive_naive_table ;
  double **trans_seminaive_naive_table;

  tmpInR = (double *) malloc(sizeof(double)*(4*bw*bw));
  tmpInI = (double *) malloc(sizeof(double)*(4*bw*bw));
  tmpOutR = (double *) malloc(sizeof(double)*(4*bw*bw));
  tmpOutI = (double *) malloc(sizeof(double)*(4*bw*bw));

  scratch = (double *) malloc(sizeof(double)*((14*bw*bw) + (48 * bw)));

  splat = 0 ;

  seminaive_naive_tablespace =
    (double *) malloc(sizeof(double) *
		      (Reduced_Naive_TableSize(bw,splat) +
		       Reduced_SpharmonicTableSize(bw,splat)));
  
  trans_seminaive_naive_tablespace =
    (double *) malloc(sizeof(double) *
		      (Reduced_Naive_TableSize(bw,splat) +
		       Reduced_SpharmonicTableSize(bw,splat)));
  
  /****
       At this point, check to see if all the memory has been
       allocated. If it has not, there's no point in going further.
  ****/
  
  if ( (scratch == NULL) || 
       (tmpInR == NULL) || (tmpInI == NULL) ||
       (tmpOutR == NULL) || (tmpOutI == NULL) ||
       (seminaive_naive_tablespace == NULL) ||
       (trans_seminaive_naive_tablespace == NULL) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }

  /* precompute for S^2 transform */
  seminaive_naive_table = SemiNaive_Naive_Pml_Table(bw, splat,
						    seminaive_naive_tablespace,
						    scratch);

  trans_seminaive_naive_table =
    Transpose_SemiNaive_Naive_Pml_Table(seminaive_naive_table,
					bw, splat,
					trans_seminaive_naive_tablespace,
					scratch);

  if ( isReal )
    {
      for(i = 0 ; i < 4*bw*bw ; i ++ )
	{
	  tmpInR[i] = sigIn[i];
	  tmpInI[i] = 0. ;
	}
    }
  else
    {
      for(i = 0 ; i < 4*bw*bw ; i ++ )
	{
	  tmpInR[i] = sigIn[2*i];
	  tmpInI[i] = sigIn[2*i+1];
	}
    }
  
  /* now rotate */
  rotateFct( bw, bw, bw - 1,
	     tmpInR, tmpInI,
	     tmpOutR, tmpOutI,
	     alpha, beta, gamma,
	     scratch,
	     seminaive_naive_table,
	     trans_seminaive_naive_table ) ;
  if ( 0 )
    for(i=0;i<5;i++)
      printf("%d\t%f\t%f\n",i,tmpOutR[i],tmpOutI[i]);

  if ( isReal )
    {
      for(i = 0 ; i < 4*bw*bw ; i ++ )
	{
	  sigOut[i] = tmpOutR[i];
	}
    }
  else
    {
      for(i = 0 ; i < 4*bw*bw ; i ++ )
	{
	  sigOut[2*i] = tmpOutR[i];
	  sigOut[2*i+1] = tmpOutI[i];
	}
    }

  /* clean up */ 
  free(trans_seminaive_naive_table);
  free(seminaive_naive_table);
  free(trans_seminaive_naive_tablespace); 
  free(seminaive_naive_tablespace); 
  free(scratch);
  free(tmpOutI);
  free(tmpOutR);
  free(tmpInI);
  free(tmpInR);


}


