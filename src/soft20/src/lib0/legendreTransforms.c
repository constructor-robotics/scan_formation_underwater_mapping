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

/* Source code for computing the Legendre transform

   For a description, see the related paper or Sean's thesis.

*/


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>   /** for memcpy **/

#include "s2_primitive.h"
#include "newFCT.h"
#include "cospmls.h"
#include "oddweights.h"
#include "weights.h"


/************************************************************************/
/*
   bw - bandwidth
   m - order
   data - a pointer to double array of size (2*bw) containing a synthesized
          function.
   plmtable - a pointer to a double array of size (2*bw*(bw-m));
	      contains the precomputed plms

   result - a pointer to double array of size (bw-m) and containing the
            computed Legendre coefficients, starting with the Pmm
	    coefficient.

   workspace - array of size 2 * bw;

   A minimal, hacked version of the above routine, except that
   as input ones of the arguments is the precomputed table of
   necessary associated Legendre functions.

*/


void Naive_AnalysisX(double *data,
		     int bw,
		     int m,
		     double *result,
		     double *plmtable,
		     double *workspace)
{
  int i, j;
  const double *weight_vec;
  double result0, result1, result2, result3;
  register double *wdata;

  wdata = workspace;

  /* make sure result is zeroed out */
  for (i=0; i<(bw-m); i++)
    result[i] = 0.0;

  /* apply quadrature weights */
  weight_vec = get_weights(bw);
      
  for(i = 0; i < 2 * bw; i++)
    wdata[i] = data[i] * weight_vec[i];

  for (i = 0; i < bw - m; i++)
	{
	  result0 = 0.0; result1 = 0.0;
	  result2 = 0.0; result3 = 0.0;
	  
	  for(j = 0; j < (2 * bw) % 4; ++j)
	    result0 += wdata[j] * plmtable[j];
	  for( ; j < (2 * bw); j += 4)
	    {
	      result0 += wdata[j] * plmtable[j];
	      result1 += wdata[j + 1] * plmtable[j + 1];
	      result2 += wdata[j + 2] * plmtable[j + 2];
	      result3 += wdata[j + 3] * plmtable[j + 3];
	    }
	  result[i] = result0 + result1 + result2 + result3;

	  plmtable += (2 * bw);
	}

}


/************************************************************************/
/* This is the procedure that synthesizes a function from a list
   of coefficients of a Legendre series.  Function is synthesized
   at the (2*bw) Chebyshev nodes. Associated Legendre functions are
   assumed to be precomputed.
   
   bw - bandwidth
   m - order
   plmtable - precomputed associated Legendre functions
   coeffs - a pointer to double array of size (bw-m).  First coefficient is
            coefficient for Pmm
   result - a pointer to double array of size (2*bw) and containing the
            synthesized function

*/

void Naive_SynthesizeX(double *coeffs,
		       int bw,
		       int m,
		       double *result,
		       double *plmtable)
{
    int i, j;
    double tmpcoef;

    /* make sure result is zeroed out
    for (i=0; i<(2*bw); i++)
      result[i] = 0.0;
      */


    /* add in Pmm contribution */
    /***
    tmpcoef = coeffs[0];
    if (tmpcoef != 0.0)
      {
	if (m == 0)
	  for (j=0; j<(2*bw); j++)
	    result[j] = tmpcoef;
	else
	  for (j=0; j<(2*bw); j++)
	    result[j] = tmpcoef * plmtable[j];
      }
    ***/

    
    tmpcoef = coeffs[0];
    if (tmpcoef != 0.0)
      {
	for (j=0; j<(2*bw); j++)
	  result[j] = tmpcoef * plmtable[j];
      }
    


    plmtable += ( 2 * bw );

    /* now generate remaining pmls while synthesizing function */

    /******************

    if (m == 0)
      {
	for (i=0; i<bw-m-1; i++) {
	  tmpcoef = coeffs[i+1] * 2.0;
	  if (tmpcoef != 0.0)
	    {
	      for (j=0; j<(2*bw); j++)
		result[j] += (tmpcoef * plmtable[j]);
	      plmtable += (2 * bw);
	    }
	}
      }
    else
      {
	for (i=0; i<bw-m-1; i++) {
	  tmpcoef = coeffs[i+1];
	  if (tmpcoef != 0.0)
	    {
	      for (j=0; j<(2*bw); j++)
		result[j] += (tmpcoef * plmtable[j]);
	      plmtable += (2 * bw);
	    }
	}
      }
    *****************/

    for (i=0; i<bw-m-1; i++) {
      tmpcoef = coeffs[i+1];
      if (tmpcoef != 0.0)
	{
	  for (j=0; j<(2*bw); j++)
	    result[j] += (tmpcoef * plmtable[j]);
	  plmtable += (2 * bw);
	}
    }

    

}



/************************************************************************/
/* InvSemiNaiveReduced computes the inverse Legendre transform
   using the transposed seminaive algorithm.  Note that because
   the Legendre transform is orthogonal, the inverse can be
   computed by transposing the matrix formulation of the
   problem.

   The forward transform looks like

   l = PCWf

   where f is the data vector, W is a quadrature matrix,
   C is a cosine transform matrix, P is a matrix
   full of coefficients of the cosine series representation
   of each Pml function P(m,m) P(m,m+1) ... P(m,bw-1),
   and l is the (associated) Legendre series representation
   of f.

   So to do the inverse, you do

   f = trans(C) trans(P) l

   so you need to transpose the matrix P from the forward transform
   and then do a cosine series evaluation.  No quadrature matrix
   is necessary.  If order m is odd, then there is also a sin
   factor that needs to be accounted for.

   Note that this function was written to be part of a full
   spherical harmonic transform, so a lot of precomputation
   has been assumed.

   Input argument description

   assoc_legendre_series - a double pointer to an array of length
                           (bw-m) containing associated
			   Legendre series coefficients.  Assumed
			   that first entry contains the P(m,m)
			   coefficient.

   bw - problem bandwidth, assumed to be a power of two.
   m - order of the associated Legendre functions
   result - a double pointer to an array of (2*bw) samples
            representing the evaluation of the Legendre
	    series at (2*bw) Chebyshev nodes.
   trans_cos_pml_table - double pointer to array representing
                         the linearized form of trans(P) above.
			 See cospmls.{h,c} for a description
			 of the function Transpose_CosPmlTableGen()
			 which generates this array.
   sin_values - when m is odd, need to factor in the sin(x) that
                is factored out of the generation of the values
		in trans(P).
   workspace - a double array of size 5*bw
   

*/

void InvSemiNaiveReduced(double *assoc_legendre_series, 
			 int bw, 
			 int m, 
			 double *result, 
			 double *trans_cos_pml_table, 
			 double *sin_values,
			 double *workspace)
{
  double *fcos; /* buffer for cosine series rep of result */
  double *trans_tableptr;
  double *scratchpad;
  double *assoc_offset;
  int i, j, n, rowsize;

  double *p;
  double fcos0, fcos1, fcos2, fcos3;

  fcos = workspace; /* needs bw */
  scratchpad = fcos + bw; /* needs (4*bw) */

  /* total workspace is 5*bw */

  trans_tableptr = trans_cos_pml_table;
  p = trans_cos_pml_table;

  /* main loop - compute each value of fcos

     Note that all zeroes have been stripped out of the
     trans_cos_pml_table, so indexing is somewhat complicated.
     */

  /***** original version of the loop (easier to see
    how things are being done)

    for (i=0; i<bw; i++)
    {
    if (i == (bw-1))
    {
    if ((m % 2) == 1)
    {
    fcos[bw-1] = 0.0;
    break;
    }
    }

    fcos[i] = 0.0;
    rowsize = Transpose_RowSize(i, m, bw);

    if (i > m)
    assoc_offset = assoc_legendre_series + (i - m) + (m % 2);
    else
    assoc_offset = assoc_legendre_series + (i % 2);

    for (j=0; j<rowsize; j++)
    fcos[i] += assoc_offset[2*j] * trans_tableptr[j];

    trans_tableptr += rowsize;
    }
    
    end of the original version ******/

  /*** this is a more efficient version of the above, original
    loop ***/

  for (i=0; i<=m; i++)
    {
      rowsize = Transpose_RowSize(i, m, bw);

      /* need to point to correct starting value of Legendre series */
      assoc_offset = assoc_legendre_series + (i % 2);

      fcos0 = 0.0 ; fcos1 = 0.0; fcos2 = 0.0; fcos3 = 0.0;

      for (j = 0; j < rowsize % 4; ++j)
	fcos0 += assoc_offset[2*j] * trans_tableptr[j];

      for ( ; j < rowsize; j += 4){
	fcos0 += assoc_offset[2*j] * trans_tableptr[j];
	fcos1 += assoc_offset[2*(j+1)] * trans_tableptr[j+1];
	fcos2 += assoc_offset[2*(j+2)] * trans_tableptr[j+2];
	fcos3 += assoc_offset[2*(j+3)] * trans_tableptr[j+3];
      }

      fcos[i] = fcos0 + fcos1 + fcos2 + fcos3;

      trans_tableptr += rowsize;      
    }
  
  for (i=m+1; i<bw-1; i++)
    {

      rowsize = Transpose_RowSize(i, m, bw);

      /* need to point to correct starting value of Legendre series */
      assoc_offset = assoc_legendre_series + (i - m) + (m % 2);

      fcos0 = 0.0 ; fcos1 = 0.0; fcos2 = 0.0; fcos3 = 0.0;
 
      for (j = 0; j < rowsize % 4; ++j)
	fcos0 += assoc_offset[2*j] * trans_tableptr[j];

      for ( ; j < rowsize; j += 4){
	fcos0 += assoc_offset[2*j] * trans_tableptr[j];
	fcos1 += assoc_offset[2*(j+1)] * trans_tableptr[j+1];
	fcos2 += assoc_offset[2*(j+2)] * trans_tableptr[j+2];
	fcos3 += assoc_offset[2*(j+3)] * trans_tableptr[j+3];
      }
      fcos[i] = fcos0 + fcos1 + fcos2 + fcos3;
      
      trans_tableptr += rowsize;
    }

  if((m % 2) == 1)
    /* if m odd, no need to do last row - all zeroes */
    fcos[bw-1] = 0.0;
  else
    {
      rowsize = Transpose_RowSize(bw-1, m, bw);

      /* need to point to correct starting value of Legendre series */
      assoc_offset = assoc_legendre_series + (bw - 1 - m) + (m % 2);
      
      fcos0 = 0.0; fcos1 = 0.0; fcos2 = 0.0; fcos3 = 0.0;

      for(j = 0; j < rowsize % 4; ++j)
	fcos0 += assoc_offset[2*j] * trans_tableptr[j];

      for ( ; j < rowsize; j += 4){
	fcos0 += assoc_offset[2*j] * trans_tableptr[j];
	fcos1 += assoc_offset[2*(j+1)] * trans_tableptr[j+1];
	fcos2 += assoc_offset[2*(j+2)] * trans_tableptr[j+2];
	fcos3 += assoc_offset[2*(j+3)] * trans_tableptr[j+3];
      }

      fcos[bw - 1] = fcos0 + fcos1 + fcos2 + fcos3;

      trans_tableptr += rowsize;
    }


  /* now we have the cosine series for the result,
     so now evaluate the cosine series at 2*bw Chebyshev nodes 
     */

  ExpIFCT(fcos, result, scratchpad, 2*bw, bw, 1);

  /* if m is odd, then need to multiply by sin(x) at Chebyshev nodes */

  if ((m % 2) == 1)
    {
      n = 2*bw;

      for (j=0; j<n; j++)
	result[j] *= sin_values[j];
    }

  trans_tableptr = p;

  /* amscray */

}


/************************************************************************/

/* SemiNaiveReduced computes the Legendre transform of data.
   This function has been designed to be a component in
   a full spherical harmonic transform.  

   data - pointer to double array of size (2*bw) containing
          function to be transformed.  Assumes sampling at Chebyshev nodes
   bw   - bandwidth of the problem - power of 2 expected
   m   - order of the problem.  0 <= m < bw
   result - pointer to double array of length bw for returning computed
            Legendre coefficients.  Contains 
            bw-m coeffs, with the <f,P(m,m)> coefficient
            located in result[0]
   cos_pml_table - a pointer to an array containing the cosine
                   series coefficients of the Pmls (or Gmls)
		   for this problem.  This table can be computed
		   using the CosPmlTableGen() function, and
		   the offset for a particular Pml can be had
		   by calling the function TableOffset().
		   The size of the table is computed using
		   the TableSize() function.  Note that
		   since the cosine series are always zero-striped,
		   the zeroes have been removed.

   workspace - a pointer to a double array of size (8 * bw)

   cos_even - ptr to double array of length bw, used as "dummy"
              workspace

   CoswSave - ptr to double array holding the precomputed data
           the fftpack's dct needs, for a dct of length 2*bw

*/

void SemiNaiveReduced(double *data, 
		      int bw, 
		      int m, 
		      double *result, 
		      double *cos_pml_table, 
		      double *workspace,
		      double *cos_even)
{
  int i, j, n;
  const double *weights;
  double *weighted_data;
  double *cos_data;
  double *scratchpad;

  double *cos_odd;

  double eresult0, eresult1, eresult2, eresult3;
  double oresult0, oresult1;  
  double *pml_ptr_even, *pml_ptr_odd;

  int ctr;
  double d_bw;

  n = 2*bw;
  d_bw = (double) bw;

  cos_odd = cos_even + (bw/2);

  /* assign workspace */
  weighted_data = workspace;
  cos_data = workspace + (2*bw);
  scratchpad = cos_data + (2*bw); /* scratchpad needs (4*bw) */
  /* total workspace = (8 * bw) */
 
  /*
    need to apply quadrature weights to the data and compute
    the cosine transform: if the order is even, get the regular
    weights; if the order is odd, get the oddweights (those
    where I've already multiplied the sin factor in
    */

  if( (m % 2) == 0)
    weights = get_weights(bw);
  else
    weights = get_oddweights(bw);

 
  /*
    smooth the weighted signal
    */
  for ( i = 0; i < n    ; ++i )
    weighted_data[i] = data[ i ] * weights[ i ];
  
  kFCT(weighted_data, cos_data, scratchpad, n, bw, 1);
  
  /* normalize the data for this problem */
  for (i=0; i<bw; i++)
    cos_data[i] *= d_bw ;
  cos_data[0] *= 2.0;


  /*** separate cos_data into even and odd indexed arrays ***/
  ctr = 0;
  for(i = 0; i < bw; i += 2)
    {
      cos_even[ctr] = cos_data[i];
      cos_odd[ctr]  = cos_data[i+1];
      ctr++;
    }
  
  /*
    do the projections; Note that the cos_pml_table has
    had all the zeroes stripped out so the indexing is
    complicated somewhat
    */
  

  /******** this is the original routine 
    for (i=m; i<bw; i++)
    {
    pml_ptr = cos_pml_table + NewTableOffset(m,i);

    if ((m % 2) == 0)
    {
    for (j=0; j<(i/2)+1; j++)
    result[i-m] += cos_data[(2*j)+toggle] * pml_ptr[j];
    }
    else
    {
    if (((i-m) % 2) == 0)
    {
    for (j=0; j<(i/2)+1; j++)
    result[i-m] += cos_data[(2*j)+toggle] * pml_ptr[j];
    }
    else
    {
    for (j=0; j<(i/2); j++)
    result[i-m] += cos_data[(2*j)+toggle] * pml_ptr[j];
    }
    } 
      
    toggle = (toggle+1) % 2;
    }

    end of original routine ***/

  /*** new version of the above for-loop; I'll
    try to remove some conditionals and move others;
    this may make things run faster ... I hope ! ***/

  /*
    this loop is unrolled to compute two coefficients
    at a time (for efficiency)
    */

  for(i = 0; i < (bw - m) - ( (bw - m) % 2 ); i += 2)
    {
      /* get ptrs to precomputed data */
      pml_ptr_even = cos_pml_table + NewTableOffset(m, m + i);
      pml_ptr_odd  = cos_pml_table + NewTableOffset(m, m + i + 1);

      eresult0 = 0.0; eresult1 = 0.0;
      oresult0 = 0.0; oresult1 = 0.0;

      for(j = 0; j < (((m + i)/2) + 1) % 2; ++j)
	{
	  eresult0 += cos_even[j] * pml_ptr_even[j];
	  oresult0 += cos_odd[j] * pml_ptr_odd[j];
	}
      for( ;  j < ((m + i)/2) + 1; j += 2)
	{
	  eresult0 += cos_even[j] * pml_ptr_even[j];
	  eresult1 += cos_even[j + 1] * pml_ptr_even[j + 1];
	  oresult0 += cos_odd[j] * pml_ptr_odd[j];
	  oresult1 += cos_odd[j + 1] * pml_ptr_odd[j + 1];
	}

      /* save the result */
      result[i] = eresult0 + eresult1;
      result[i + 1] = oresult0 + oresult1;
    }

  /* if there are an odd number of coefficients to compute
     at this order, get that last coefficient! */
  if( ( (bw - m) % 2 ) == 1)
    {
      pml_ptr_even = cos_pml_table + NewTableOffset(m, bw - 1);

      eresult0 = 0.0; eresult1 = 0.0;
      eresult2 = 0.0; eresult3 = 0.0;

      for(j = 0; j < (((bw - 1)/2) + 1) % 4; ++j)
	{
	  eresult0 += cos_even[j] * pml_ptr_even[j];
	}

      for( ;  j < ((bw - 1)/2) + 1; j += 4)
	{
	  eresult0 += cos_even[j] * pml_ptr_even[j];
	  eresult1 += cos_even[j + 1] * pml_ptr_even[j + 1];
	  eresult2 += cos_even[j + 2] * pml_ptr_even[j + 2];
	  eresult3 += cos_even[j + 3] * pml_ptr_even[j + 3];
	}
      result[bw - m - 1] = eresult0 + eresult1 + eresult2 + eresult3;
    }
}

/************************************************************************/
/************************************************************************/

