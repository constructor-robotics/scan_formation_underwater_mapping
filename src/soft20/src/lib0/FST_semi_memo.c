/***************************************************************************
  **************************************************************************
    
  Spherical Harmonic Transform Kit 2.7
    
  Copyright 1997-2003  Sean Moore, Dennis Healy,
                       Dan Rockmore, Peter Kostelec
  Copyright 2004  Peter Kostelec, Dan Rockmore

  This file is part of SpharmonicKit.

  SpharmonicKit is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  SpharmonicKit is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  See the accompanying LICENSE file for details.

  ************************************************************************
  ************************************************************************/

/********************************************************************

  FST_semi_memo.c - forward and inverse FSTs


  The primary functions in this package are

  1) FST_semi_memo() - computes the spherical harmonic expansion.
  2) InvFST_semi_memo() - computes the inverse spherical harmonic transform.

  For descriptions on calling these functions, see the documentation
  preceding each function.

*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "s2_primitive.h"
#include "cospmls.h"
#include "fft_grids.h"
#include "legendreTransforms.h"


/************************************************************************/


/************************************************************************/
/* performs a spherical harmonic transform using the semi-naive
   and naive algorithms */
/* size is the dimension of the input array (size x size) and it is
   expected that size=2*bw.  The inputs rdata and idata are expected
   to be pointers to size x size arrays.  rcoeffs and icoeffs are expected
   to be pointers to bw x bw arrays, and will contain the harmonic
   coefficients in a "linearized" form.

   spharmonic_pml_table should be a (double **) pointer to
   the result of a call to Spharmonic_Pml_Table.  Because this
   table is re-used in the inverse transform, and because for
   timing purposes the computation of the table is not included,
   it is passed in as an argument.  Also, at some point this
   code may be used as par of a series of convolutions, so
   reducing repetitive computation is prioritized.

   spharmonic_pml_table will be an array of (double *) pointers
   the array being of length TableSize(m,bw)

   workspace needs to be a double pointer to an array of size
   (8 * bw^2) + (29 * bw).

   cutoff -> what order to switch from semi-naive to naive
             algorithm.

*/

/* 
   Output Ordering of coeffs f(m,l) is
   f(0,0) f(0,1) f(0,2) ... f(0,bw-1)
          f(1,1) f(1,2) ... f(1,bw-1)
          etc.
                 f(bw-2,bw-2), f(bw-2,bw-1)
		               f(bw-1,bw-1)
			       f(-(bw-1),bw-1)
		 f(-(bw-2),bw-2) f(-(bw-2),bw-1)
	  etc.
	          f(-2,2) ... f(-2,bw-1)
	  f(-1,1) f(-1,2) ... f(-1,bw-1)
    
   This only requires an array of size (bw*bw).  If zero-padding
   is used to make the indexing nice, then you need a an
   (2bw-1) * bw array - but that is not done here.
   Because of the amount of space necessary for doing
   large transforms, it is important not to use any
   more than necessary.

*/

/*      isReal =0 -> samples are complex, =1 -> samples real */

void FST_semi_memo(double *rdata, double *idata,
		   double *rcoeffs, double *icoeffs, 
		   int size, double **seminaive_naive_table,
		   double *workspace,
		   int isReal,
		   int cutoff)
{
  int bw, m, i, j;
  double *rres, *ires;
  double *rdataptr, *idataptr;
  double *fltres, *scratchpad;
  double *eval_pts;
  int tmpindex[2];
  double pow_one;
  double *cos_even;
  double tmpA ;

  bw = size/2;

  /* assign space */

  cos_even = (double *) malloc(sizeof(double) * bw);

  rres = workspace;  /* needs (size * size) = (4 * bw^2) */
  ires = rres + (size * size); /* needs (size * size) = (4 * bw^2) */ 
  fltres = ires + (size * size); /* needs bw  */
  eval_pts = fltres + bw; /* needs (2*bw)  */
  scratchpad = eval_pts + (2*bw); /* needs (24 * bw)  */
 

  /* total workspace is (8 * bw^2) + (29 * bw) */


  /* do the FFTs along phi */
  grid_fourier(rdata, idata, rres, ires, size, scratchpad);

  /* scale intermediary coefficients */
  tmpA = sqrt(2. * PI) ;
  for ( i = 0 ; i < size*size ; i ++ )
    {
      rres[ i ] *= tmpA ;
      ires[ i ] *= tmpA ;
    }

  /* point to start of output data buffers */
  rdataptr = rcoeffs;
  idataptr = icoeffs;
  
  for (m=0; m<bw; m++) {
    /*
      fprintf(stderr,"m = %d\n",m);
      */
    
    /*** test to see if before cutoff or after ***/
    if (m < cutoff){
      
      /* do the real part */
      SemiNaiveReduced(rres+(m*size), 
		       bw, 
		       m, 
		       fltres, 
		       seminaive_naive_table[m],
		       scratchpad,
		       cos_even);
      
      /* now load real part of coefficients into output space */  
      memcpy(rdataptr, fltres, sizeof(double) * (bw - m));
      
      rdataptr += bw-m;
      
      /* do imaginary part */
      SemiNaiveReduced(ires+(m*size),
		       bw, 
		       m, 
		       fltres, 
		       seminaive_naive_table[m],
		       scratchpad,
		       cos_even);

      /* now load imaginary part of coefficients into output space */  
      memcpy(idataptr, fltres, sizeof(double) * (bw - m));
      
      idataptr += bw-m;
      
    }
    else{
      /* do real part */
      
      Naive_AnalysisX(rres+(m*size),
		      bw,
		      m,
		      fltres,
		      seminaive_naive_table[m],
		      workspace);
      memcpy(rdataptr, fltres, sizeof(double) * (bw - m));
      rdataptr += bw-m;
      
      /* do imaginary part */
      Naive_AnalysisX(ires+(m*size),
		      bw,
		      m,
		      fltres,
		      seminaive_naive_table[m],
		      workspace);
      memcpy(idataptr, fltres, sizeof(double) * (bw - m));
      idataptr += bw-m;
    }
    
    
  }
  
  /*** now do upper coefficients ****/
  
  /* now if the data is real, we don't have to compute the
     coefficients whose order is less than 0, i.e. since
     the data is real, we know that
     f-hat(l,-m) = (-1)^m * conjugate(f-hat(l,m)),
     so use that to get the rest of the coefficients
     
     isReal =0 -> samples are complex, =1 -> samples real
     
     */
  
  if( isReal == 0 ){
    
    /* note that m is greater than bw here, but this is for
       purposes of indexing the input data arrays.  
       The "true" value of m as a parameter for Pml is
       size - m  */
    /*   fprintf(stderr,"\n now the higher order terms \n\n"); */
    
    for (m=bw+1; m<size; m++) {
      /*
	fprintf(stderr,"m = %d\n",-(size-m));
	*/

      if ( size - m < cutoff )
	{
	  /* do real part */
	  SemiNaiveReduced(rres+(m*size), 
			   bw, 
			   size-m, 
			   fltres, 
			   seminaive_naive_table[size-m],
			   scratchpad,
			   cos_even);
      
	  /* now load real part of coefficients into output space */  
	  if ((m % 2) != 0) {
	    for (i=0; i<m-bw; i++)
	      rdataptr[i] = -fltres[i];
	  }
	  else {
	    memcpy(rdataptr, fltres, sizeof(double) * (m - bw));
	  }
	  rdataptr += m-bw;
      
	  /* do imaginary part */
	  SemiNaiveReduced(ires+(m*size), 
			   bw, 
			   size-m, 
			   fltres, 
			   seminaive_naive_table[size-m],
			   scratchpad,
			   cos_even);
      
	  /* now load real part of coefficients into output space */  
	  if ((m % 2) != 0) {
	    for (i=0; i<m-bw; i++)
	      idataptr[i] = -fltres[i];
	  }
	  else {
	    memcpy(idataptr, fltres, sizeof(double) * (m - bw));
	  }
	  idataptr += m-bw;
	}
      else
	{
	  /* do real part */
      
	  Naive_AnalysisX(rres+(m*size),
			  bw,
			  size-m,
			  fltres,
			  seminaive_naive_table[size-m],
			  workspace);
	  /* now load real part of coefficients into output space */  
	  if ((m % 2) != 0) {
	    for (i=0; i<m-bw; i++)
	      rdataptr[i] = -fltres[i];
	  }
	  else {
	    memcpy(rdataptr, fltres, sizeof(double) * (m - bw));
	  }
	  rdataptr += m-bw;
      
	  /* do imaginary part */
	  Naive_AnalysisX(ires+(m*size),
			  bw,
			  size-m,
			  fltres,
			  seminaive_naive_table[size-m],
			  workspace);

	  /* now load real part of coefficients into output space */  
	  if ((m % 2) != 0) {
	    for (i=0; i<m-bw; i++)
	      idataptr[i] = -fltres[i];
	  }
	  else {
	    memcpy(idataptr, fltres, sizeof(double) * (m - bw));
	  }
	  idataptr += m-bw;

	}

    }
  }
  else        /**** if the data is real ****/
    {            
      pow_one = 1.0;
      for(i = 1; i < bw; i++){
	pow_one *= -1.0;
	for( j = i; j < bw; j++){	
	  seanindex2(i, j, bw, tmpindex);
	  rcoeffs[tmpindex[1]] =
	    pow_one * rcoeffs[tmpindex[0]];
	  icoeffs[tmpindex[1]] =
	    -1.0 * pow_one * icoeffs[tmpindex[0]];
	}
      }
    }
  
  free(cos_even);

}

/************************************************************************/
/* Inverse spherical harmonic transform.  Inputs rcoeffs and icoeffs
   are harmonic coefficients stored in (bw * bw) arrays in the order
   spec'ed above.  rdata and idata are (size x size) arrays with
   the transformed result.  size is expected to be 2 * bw.
   transpose_spharmonic_pml_table should be the (double **) result of a call
   to Transpose_Spharmonic_Pml_Table()

   workspace is (8 * bw^2) + (32 * bw)

*/

/*      isReal =0 -> samples are complex, =1 -> samples real */

void InvFST_semi_memo(double *rcoeffs, double *icoeffs, 
		      double *rdata, double *idata, 
		      int size, 
		      double **transpose_seminaive_naive_table,
		      double *workspace,
		      int isReal,
		      int cutoff)
{
  int bw, m, i, n;
  double *rdataptr, *idataptr;
  double *rfourdata, *ifourdata;
  double *rinvfltres, *iminvfltres, *scratchpad;
  double *sin_values, *eval_pts;
  // int l , dummy ;
  double tmpA ; // , tmpB ;
  //  FILE *fp ;

  bw = size/2;

  /* allocate space */

  rfourdata = workspace;                  /* needs (size * size) */
  ifourdata = rfourdata + (size * size);  /* needs (size * size) */
  rinvfltres = ifourdata + (size * size); /* needs (2 * bw) */
  iminvfltres = rinvfltres + (2 * bw);    /* needs (2 * bw) */
  sin_values = iminvfltres + (2 * bw);    /* needs (2 * bw) */
  eval_pts = sin_values + (2 * bw);       /* needs (2 * bw) */
  scratchpad = eval_pts + (2 * bw);       /* needs (24 * bw) */
  
  /* total workspace = (8 * bw^2) + (32 * bw) */

  /* load up the sin_values array */
  n = 2*bw;

  ArcCosEvalPts(n, eval_pts);
  for (i=0; i<n; i++)
    sin_values[i] = sin(eval_pts[i]);


  /* Now do all of the inverse Legendre transforms */
  rdataptr = rcoeffs;
  idataptr = icoeffs;
  for (m=0; m<bw; m++)
    {
      if(m < cutoff)
	{
      
	  /* do real part first */ 
	  InvSemiNaiveReduced(rdataptr,
			      bw,
			      m,
			      rinvfltres,
			      transpose_seminaive_naive_table[m],
			      sin_values,
			      scratchpad);
      
	  /* now do imaginary part */
      
	  InvSemiNaiveReduced(idataptr,
			      bw,
			      m,
			      iminvfltres,
			      transpose_seminaive_naive_table[m],
			      sin_values,
			      scratchpad);
      
	  /* will store normal, then tranpose before doing inverse fft */
	  memcpy(rfourdata+(m*size), rinvfltres, sizeof(double) * size);
	  memcpy(ifourdata+(m*size), iminvfltres, sizeof(double) * size);
      
	  /* move to next set of coeffs */
      
	  rdataptr += bw-m;
	  idataptr += bw-m;
      
	}
      else
	{
	  /* first do the real part */
	
	  Naive_SynthesizeX(rdataptr,
			    bw,
			    m,
			    rinvfltres,
			    transpose_seminaive_naive_table[m]);

	  /* now do the imaginary */	
	  Naive_SynthesizeX(idataptr,
			    bw,
			    m,
			    iminvfltres,
			    transpose_seminaive_naive_table[m]);

	  /* will store normal, then tranpose before doing inverse fft    */
	  memcpy(rfourdata+(m*size), rinvfltres, sizeof(double) * size);
	  memcpy(ifourdata+(m*size), iminvfltres, sizeof(double) * size);
 	
	  /* move to next set of coeffs */
	  rdataptr += bw-m;
	  idataptr += bw-m;
	
	}
    
    } /* closes m loop */

  /*
  fp = fopen("step1-naive.dat","w");
  for(m=0;m<bw*size;m++)
    fprintf(fp,"%.10f\t%.10f\n",rfourdata[m],ifourdata[m]);
  fclose(fp);
  */
  
  
  /* now fill in zero values where m = bw (from problem definition) */
  memset(rfourdata + (bw * size), 0, sizeof(double) * size);
  memset(ifourdata + (bw * size), 0, sizeof(double) * size);
  
  /* now if the data is real, we don't have to compute the
     coefficients whose order is less than 0, i.e. since
     the data is real, we know that
     invf-hat(l,-m) = conjugate(invf-hat(l,m)),
     so use that to get the rest of the real data
     
     isReal =0 -> samples are complex, =1 -> samples real
     
     */

  if(isReal == 0){
    
    /* now do negative m values */
    
    for (m=bw+1; m<size; m++) 
      {

	if ( size-m < cutoff )
	  {
	    /*
	      fprintf(stderr,"m = %d\n",-(size-m));
	    */
	
	    /* do real part first */
	
	    InvSemiNaiveReduced(rdataptr,
				bw,
				size - m,
				rinvfltres,
				transpose_seminaive_naive_table[size - m],
				sin_values,
				scratchpad);
	
	    /* now do imaginary part */
	
	    InvSemiNaiveReduced(idataptr,
				bw,
				size - m,
				iminvfltres,
				transpose_seminaive_naive_table[size - m],
				sin_values,
				scratchpad);
	

	    /* will store normal, then tranpose before doing inverse fft    */
	    if ((m % 2) != 0)
	      for(i=0; i< size; i++){
		rinvfltres[i] = -rinvfltres[i];
		iminvfltres[i] = -iminvfltres[i];
	      }
	
	    memcpy(rfourdata + (m*size), rinvfltres, sizeof(double) * size);
	    memcpy(ifourdata + (m*size), iminvfltres, sizeof(double) * size);
	
	    /* move to next set of coeffs */
	    rdataptr += bw-(size-m);
	    idataptr += bw-(size-m);

	  }
	else
	  {

	  /* first do the real part */
	
	  Naive_SynthesizeX(rdataptr,
			    bw,
			    size - m,
			    rinvfltres,
			    transpose_seminaive_naive_table[size-m]);

	  /* now do the imaginary */	
	  Naive_SynthesizeX(idataptr,
			    bw,
			    size-m,
			    iminvfltres,
			    transpose_seminaive_naive_table[size-m]);

	    /* will store normal, then tranpose before doing inverse fft    */
	    if ((m % 2) != 0)
	      for(i=0; i< size; i++){
		rinvfltres[i] = -rinvfltres[i];
		iminvfltres[i] = -iminvfltres[i];
	      }
	
	    memcpy(rfourdata + (m*size), rinvfltres, sizeof(double) * size);
	    memcpy(ifourdata + (m*size), iminvfltres, sizeof(double) * size);
	
	
	    /* move to next set of coeffs */
	    rdataptr += bw-(size-m);
	    idataptr += bw-(size-m);

	  }

	
      } /* closes m loop */
  }
  else {
    for(m = bw + 1; m < size; m++){
      
      memcpy(rfourdata+(m*size), rfourdata+((size-m)*size),
	     sizeof(double) * size);
      memcpy(ifourdata+(m*size), ifourdata+((size-m)*size),
	     sizeof(double) * size);
      for(i = 0; i < size; i++)
	ifourdata[(m*size)+i] *= -1.0;
    }
    
    
  }
  
  /** now transpose **/
  transpose(rfourdata, size);
  transpose(ifourdata, size);

  /* do some scaling */
  tmpA = 1./sqrt(2.* PI) ;
  for ( i = 0 ; i < size*size ; i ++ )
    {
      rfourdata[ i ] *= tmpA ;
      ifourdata[ i ] *= tmpA ;
    }
  
  /* now do inverse fourier grid computation */
  grid_invfourier(rfourdata, ifourdata, rdata, idata, size, scratchpad);
  
  /* amscray */
  
}
