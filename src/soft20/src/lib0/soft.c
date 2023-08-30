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

  functions that do the FORWARD and INVERSE transforms on the
  full group SO(3)

  sample size = (2*bw)^3
  coefficient size = on the order of bw^3 (a little more, actually)

  functions in here:

  Forward_SO3_naive();
  Inverse_SO3_naive();

*/

#include <string.h>
#include <math.h>
#include "utils_so3.h"
#include "fft_grids_so3.h"
#include "makeWigner.h"
#include "makeweights.h"
#include "wignerTransforms.h"

/*
  ok, the forward transform

  Function arguments are as follows:

  bw = bandwidth of transform
  rdata, idata: real and imaginary parts of the input signal,
                EACH array of size (2*bw)^3
  rcoeffs, icoeffs: real and imaginary parts of the coefficients,
                    EACH array of size bw*(4*bw^2-1)/3
  workspace1: array for tmp storage, of size (gulp) 4 * (2*bw)^3
  workspace2: another array for tmp storage, of size 26*bw + 2*bw^2 ;

*/

void Forward_SO3_Naive( int bw,
			double *rdata, double *idata,
			double *rcoeffs, double *icoeffs,
			double *workspace1, double *workspace2 )
{
  int j, n ;
  int m1, m2 ;
  double *t1r, *t1i, *t2r, *t2i ;
  double *sinPts, *cosPts, *sinPts2, *cosPts2 ;
  double *wigners, *scratch ;
  double *weights ;
  double *rcoeffsPtr, *icoeffsPtr ;
  //  double sqrtn ;
  double scalar ;
  int tmpInt ;

  n = 2 * bw ;
  t1r = workspace1 ;
  t1i = workspace1 + (n * n * n) ;
  t2r = t1i + (n * n * n) ;
  t2i = t2r + (n * n * n) ;

  /* I'll need these for later */
  rcoeffsPtr = rcoeffs ;
  icoeffsPtr = icoeffs ;

  weights = workspace2 ;
  sinPts = weights + n ;
  cosPts = sinPts + n ;
  sinPts2 = cosPts + n ;
  cosPts2 = sinPts2 + n ;
  wigners = cosPts2 + n ;
  scratch = wigners + ( bw * n ) ; /* wigners need at most bw*n space AT
				      ANY given orders m1, m2 */

  /*
    before going further, let's precompute all the sines
    and cosines I'll need. No matter what order transform
    I'm doing, these'll stay the same.
  */
  SinEvalPts( n, sinPts );
  CosEvalPts( n, cosPts );
  SinEvalPts2( n, sinPts2 );
  CosEvalPts2( n, cosPts2 );

  /* compute quadrature weights */
  makeweights2( bw, weights );

    
  /*
    Stage 1: FFT the "rows". Instead of treating the signal as
    3-D object, I can also think of it as an array of size
    (n^2) x n. This means all I'm doing in the first stage
    is taking n^2-many FFTs, each of length n.

    NOTE: Since I'm reusing the FFT code from SpharmonicKit,
          even though I'm doing the FORWARD SO(3) transform
	  here, I need to call grid_invfourier_so3  -> the signs
	  on the complex exponentials are switched (detailed
	  explanation to be put here eventually, but trust
	  me)
  */
  
  grid_invfourier_so3( rdata, idata,
		       t1r, t1i,
		       n*n, n,
		       scratch ) ;

  /* normalize the Fourier coefficients (sorry, have to do it) */
  /*
    But wait! Instead of normalizing here, and right after the second
    fft, I'm going to wait till the end, and combine the 3 normalizations
    I have to do. Cut down on the for-loops ...
  */
  /*
    sqrtn = 1./sqrt((double) n);
    for ( j = 0 ; j < n*n*n; j++ )
    {
    t1r[ j ] *= sqrtn ;
    t1i[ j ] *= sqrtn ;
    }
  */

  /*
    Stage 2: transpose!
  */
  
  transpose_so3( t1r, t2r, n*n, n ) ;
  transpose_so3( t1i, t2i, n*n, n ) ;

  /*
    Stage 3: FFT again. Note that I'm using the tmp space
             of t1r, t1i again
  */

  grid_invfourier_so3( t2r, t2i,
		       t1r, t1i,
		       n*n, n,
		       scratch ) ;

  /* normalize the Fourier coefficients (sorry, have to do it) */
  /*
    for ( j = 0 ; j < n*n*n; j++ )
    {
    t1r[ j ] *= sqrtn ;
    t1i[ j ] *= sqrtn ;
    }
  */

  /*
    Stage 4: transpose again! And note I'm using the tmp space
             of t2r, t2i again.
  */
  
  transpose_so3( t1r, t2r, n*n, n ) ;
  transpose_so3( t1i, t2i, n*n, n ) ;
 
  /*
    Stage 5: Do the Wigner transforms. This is the tricky bit.

    Since I'm working with two order indeces, m1 and m2, the
    for-loops will be more intricate than in the case of the
    "ordinary" spherical transform on S^2.

    If assumptions are made regarding the original input signal,
    e.g. it's strictly real, then one may take advantage of
    symmetries of the big D wigners (i.e. function of all 3
    parameters alpha, beta, gamma) and so simplify the for-loops
    some and hence increase the speed of the program. However,
    the for-loops to follow will make no such assumptions.
    Whether the signal is real or complex, these for-loops will
    handle it.

    The for-loops will be "designed" as follows. They will be
    divided into cases according to the orders:

    1) 0 <= m1 <= bw-1

        1a) 0 <= m2 <= bw-1
        1b) -(bw-1) <= m2 <= -1


    2) -(bw-1) <= m1 <= -1

        2a) 0 <= m2 <= bw-1
	2b) -(bw-1) <= m2 <= -1


    Fasten your seatbelt, folks. It's going to be a bumpy ride.

  */


  /*** 0 <= m1 <= bw-1 ***/
  for ( m1 = 0 ; m1 < bw ; m1 ++ )
    {

      /*** 0 <= m2 <= bw-1 ***/
      for ( m2 = 0 ; m2 < bw ; m2 ++ )
	{
	  /* compute the wigners I'll need at these orders */
	  genWig_L2( m1, m2, bw,
		     sinPts, cosPts,
		     sinPts2, cosPts2,
		     wigners, scratch ) ;

	  /* now apply the wigner transform to the
	     real and imaginary parts */

	  wigNaiveAnalysis( m1, m2, bw, t2r,
			    wigners, weights,
			    rcoeffs,
			    scratch ) ;

	  wigNaiveAnalysis( m1, m2, bw, t2i,
			    wigners, weights,
			    icoeffs,
			    scratch ) ;

	  /* advance the pointers by the appropriate
	     amount at these orders m1, m2 */
	  t2r += n ;
	  t2i += n ;
	  rcoeffs += howMany_so3( m1, m2, bw ) ;
	  icoeffs += howMany_so3( m1, m2, bw ) ;
	}

      /* need to advance data pointers to skip the "zeros" */
      t2r += n ;
      t2i += n ;

      /*** -(bw-1) <= m2 <= -1 ***/
      for ( m2 = -(bw-1) ; m2 < 0 ; m2 ++ )
	{
	  /* compute the wigners I'll need at these orders */
	  genWig_L2( m1, m2, bw,
		     sinPts, cosPts,
		     sinPts2, cosPts2,
		     wigners, scratch ) ;

	  /* now apply the wigner transform to the
	     real and imaginary parts */

	  wigNaiveAnalysis( m1, m2, bw, t2r,
			    wigners, weights,
			    rcoeffs,
			    scratch ) ;

	  wigNaiveAnalysis( m1, m2, bw, t2i,
			    wigners, weights,
			    icoeffs,
			    scratch ) ;

	  /* advance the pointers by the appropriate
	     amount at these orders m1, m2 */
	  t2r += n ;
	  t2i += n ;
	  rcoeffs += howMany_so3( m1, m2, bw ) ;
	  icoeffs += howMany_so3( m1, m2, bw ) ;
	}
    }

  /* need to advance data pointers to skip the "zeros" */
  t2r += n*n ;
  t2i += n*n ;



  /*** -(bw-1) <= m1 <= -1 ***/
  for ( m1 = -(bw-1) ; m1 < 0 ; m1 ++ )
    {

      /*** 0 <= m2 <= bw-1 ***/
      for ( m2 = 0 ; m2 < bw ; m2 ++ )
	{
	  /* compute the wigners I'll need at these orders */
	  genWig_L2( m1, m2, bw,
		     sinPts, cosPts,
		     sinPts2, cosPts2,
		     wigners, scratch ) ;

	  /* now apply the wigner transform to the
	     real and imaginary parts */

	  wigNaiveAnalysis( m1, m2, bw, t2r,
			    wigners, weights,
			    rcoeffs,
			    scratch ) ;

	  wigNaiveAnalysis( m1, m2, bw, t2i,
			    wigners, weights,
			    icoeffs,
			    scratch ) ;

	  /* advance the pointers by the appropriate
	     amount at these orders m1, m2 */
	  t2r += n ;
	  t2i += n ;
	  rcoeffs += howMany_so3( m1, m2, bw ) ;
	  icoeffs += howMany_so3( m1, m2, bw ) ;
	}

      /* need to advance data pointers to skip the "zeros" */
      t2r += n ;
      t2i += n ;

      /*** -(bw-1) <= m2 <= -1 ***/
      for ( m2 = -(bw-1) ; m2 < 0 ; m2 ++ )
	{
	  /* compute the wigners I'll need at these orders */
	  genWig_L2( m1, m2, bw,
		     sinPts, cosPts,
		     sinPts2, cosPts2,
		     wigners, scratch ) ;

	  /* now apply the wigner transform to the
	     real and imaginary parts */

	  wigNaiveAnalysis( m1, m2, bw, t2r,
			    wigners, weights,
			    rcoeffs,
			    scratch ) ;

	  wigNaiveAnalysis( m1, m2, bw, t2i,
			    wigners, weights,
			    icoeffs,
			    scratch ) ;

	  /* advance the pointers by the appropriate
	     amount at these orders m1, m2 */
	  t2r += n ;
	  t2i += n ;
	  rcoeffs += howMany_so3( m1, m2, bw ) ;
	  icoeffs += howMany_so3( m1, m2, bw ) ;
	}
    }
  
  /* need to normalize, one last time */
  tmpInt = totalCoeffs_so3( bw ) ;
  scalar = M_PI/((double) (bw*n));
  for ( j = 0 ; j < tmpInt ; j ++ )
    {
      rcoeffsPtr[ j ] *= scalar ;
      icoeffsPtr[ j ] *= scalar ;
    }

  /*** and we're done ! ***/
}

/*****************************************************************/
/*****************************************************************/
/*****************************************************************/

/*
  ok, the inverse transform

  Function arguments are as follows:

  bw = bandwidth of transform
  rcoeffs, icoeffs: real and imaginary parts of the coefficients,
                    EACH array of size (1/3 * bw * ( 4 bw^2 - 1 ))
  rdata, idata: real and imaginary parts of the synthesized (output)
                signal, EACH array of size (2*bw)^3
  workspace1: array for tmp storage, of size (gulp) 2 * (2*bw)^3

       (I don't need as much space as Forward_SO3_Naive because I'll
        be able to use the output rdata, idata arrays as tmp storage
	within this function. I couldn't use the rcoeffs, icoeffs arrays
	in Forward_SO3_Naive() because they weren't large enough ... I
	needed (2 bw)^3 space and they're only (1/3 * bw * ( 4 bw^2 - 1 ))
  workspace2: another array for tmp storage, of size 24*bw + 2*bw^2 ;

*/

void Inverse_SO3_Naive( int bw,
			double *rcoeffs, double *icoeffs,
			double *rdata, double *idata,
			double *workspace1, double *workspace2 )
{
  int j, n ;
  int m1, m2 ;
  double *t1r, *t1i, *t2r, *t2i ;
  double *sinPts, *cosPts, *sinPts2, *cosPts2 ;
  double *wignersTrans, *scratch ;
  double *rdataPtr, *idataPtr ;
  double *rcoeffsPtr, *icoeffsPtr ;
  double dn ;
  int tmpInt ;

  n = 2 * bw ;
  t1r = workspace1 ;
  t1i = workspace1 + (n * n * n) ;
  t2r = t1i + (n * n * n) ;
  t2i = t2r + (n * n * n) ;


  /* I'll need these for later */
  rdataPtr = rdata ;
  idataPtr = idata ;
  rcoeffsPtr = rcoeffs ;
  icoeffsPtr = icoeffs ;


  sinPts = workspace2 ;
  cosPts = sinPts + n ;
  sinPts2 = cosPts + n ;
  cosPts2 = sinPts2 + n ;
  wignersTrans = cosPts2 + n ;
  scratch = wignersTrans + ( bw * n ) ; /* wignersTrans need at most bw*n
					   space AT ANY given orders m1, m2 */

  /*
    before going further, let's precompute all the sines
    and cosines I'll need. No matter what order transform
    I'm doing, these'll stay the same.
  */
  SinEvalPts( n, sinPts );
  CosEvalPts( n, cosPts );
  SinEvalPts2( n, sinPts2 );
  CosEvalPts2( n, cosPts2 );


  /* Stage 0.5: Need to normalize the numbers before
     doing the IDWT
 */
  /* Nope! I'll do at the end of the function call,
     when I have to normalize for the two ffts anyway.
     This should cut down on the for-loops. */
  /*
  dn = ((double)bw)/M_PI;
  dn = 1.0 ;
  for ( j = 0 ; j < totalCoeffs_so3( bw ) ; j++ )
    {
      t1r[ j ] = rcoeffs[ j ] * dn ;
      t1i[ j ] = icoeffs[ j ] * dn ;
    }
  */
  tmpInt = totalCoeffs_so3( bw ) ;
  memcpy( t1r, rcoeffs, sizeof(double)*tmpInt );
  memcpy( t1i, icoeffs, sizeof(double)*tmpInt );

  /*
    Stage 1: Do the Inverse Wigner transform. The rcoeffs, icoeffs
    arrays are assumed to be in the same "arrangement" as that produced
    by Forward_SO3_Naive().

    Since I'm working with two order indeces, m1 and m2, the
    for-loops will be more intricate than in the case of the
    "ordinary" spherical transform on S^2.

    If assumptions are made regarding the original input signal,
    e.g. it's strictly real, then one may take advantage of
    symmetries of the big D wigners (i.e. function of all 3
    parameters alpha, beta, gamma) and so simplify the for-loops
    some and hence increase the speed of the program. However,
    the for-loops to follow will make no such assumptions.
    Whether the signal is real or complex, these for-loops will
    handle it.

    The for-loops will be "designed" as follows. They will be
    divided into cases according to the orders:

    1) 0 <= m1 <= bw-1

    1a) 0 <= m2 <= bw-1
    1b) -(bw-1) <= m2 <= -1


    2) -(bw-1) <= m1 <= -1

    2a) 0 <= m2 <= bw-1
    2b) -(bw-1) <= m2 <= -1


    Fasten your seatbelt, folks. It's going to be a bumpy ride.

  */


  /* NOTE that I'm using the rdata, idata arrays as tmp space
     in the early going of the function */


  /*** 0 <= m1 <= bw-1 ***/
  for ( m1 = 0 ; m1 < bw ; m1 ++ )
    {

      /*** 0 <= m2 <= bw-1 ***/
      for ( m2 = 0 ; m2 < bw ; m2 ++ )
	{
	  /* compute the wigners I'll need at these orders ...
	     TRANSPOSE version of genWig_L2() */
	  genWigTrans_L2( m1, m2, bw,
			  sinPts, cosPts,
			  sinPts2, cosPts2,
			  wignersTrans, scratch ) ;
	  
	  /* now apply the inverse wigner transform to the
	     real and imaginary parts */
	  
	  wigNaiveSynthesis( m1, m2, bw, t1r,
			     wignersTrans, t2r,
			     scratch ) ;

	  wigNaiveSynthesis( m1, m2, bw, t1i,
			     wignersTrans, t2i,
			     scratch ) ;

	  /* advance the pointers by the appropriate
	     amount at these orders m1, m2 */
	  t1r += howMany_so3( m1, m2, bw ) ;
	  t1i += howMany_so3( m1, m2, bw ) ;
	  t2r += n ;
	  t2i += n ;
	}

      /* need to set n-many zeros in the output arrays,
	 and advance things along in the array */
      for ( j = 0 ; j < n ; j ++ )
	{
	  *t2r++ = 0.0 ;
	  *t2i++ = 0.0 ;
	}


      /*** -(bw-1) <= m2 <= -1 ***/
      for ( m2 = -(bw-1) ; m2 < 0 ; m2 ++ )
	{
	  /* compute the wigners I'll need at these orders */
	  genWigTrans_L2( m1, m2, bw,
			  sinPts, cosPts,
			  sinPts2, cosPts2,
			  wignersTrans, scratch ) ;
	  
	  /* now apply the wigner transform to the
	     real and imaginary parts */

	  wigNaiveSynthesis( m1, m2, bw, t1r,
			     wignersTrans, t2r,
			     scratch ) ;

	  wigNaiveSynthesis( m1, m2, bw, t1i,
			     wignersTrans, t2i,
			     scratch ) ;

	  /* advance the pointers by the appropriate
	     amount at these orders m1, m2 */
	  t1r += howMany_so3( m1, m2, bw ) ;
	  t1i += howMany_so3( m1, m2, bw ) ;
	  t2r += n ;
	  t2i += n ;
	}
    }


  /* need to set n-many zeros in the output arrays,
     and advance things along in the array */
  for ( j = 0 ; j < n*n ; j ++ )
    {
      *t2r++ = 0.0 ;
      *t2i++ = 0.0 ;
    }

  /*** -(bw-1) <= m1 <= -1 ***/
  for ( m1 = -(bw-1) ; m1 < 0 ; m1 ++ )
    {

      /*** 0 <= m2 <= bw-1 ***/
      for ( m2 = 0 ; m2 < bw ; m2 ++ )
	{
	  /* compute the wigners I'll need at these orders */
	  genWigTrans_L2( m1, m2, bw,
			  sinPts, cosPts,
			  sinPts2, cosPts2,
			  wignersTrans, scratch ) ;
	  
	  /* now apply the wigner transform to the
	     real and imaginary parts */
	  
	  wigNaiveSynthesis( m1, m2, bw, t1r,
			     wignersTrans, t2r,
			     scratch ) ;
	  
	  wigNaiveSynthesis( m1, m2, bw, t1i,
			     wignersTrans, t2i,
			     scratch ) ;

	  /* advance the pointers by the appropriate
	     amount at these orders m1, m2 */
	  t1r += howMany_so3( m1, m2, bw ) ;
	  t1i += howMany_so3( m1, m2, bw ) ;
	  t2r += n ;
	  t2i += n ;
	}

      /* need to set n-many zeros in the output arrays,
	 and advance things along in the array */
      for ( j = 0 ; j < n ; j ++ )
	{
	  *t2r++ = 0.0 ;
	  *t2i++ = 0.0 ;
	}

      /*** -(bw-1) <= m2 <= -1 ***/
      for ( m2 = -(bw-1) ; m2 < 0 ; m2 ++ )
	{
	  /* compute the wigners I'll need at these orders */
	  genWigTrans_L2( m1, m2, bw,
			  sinPts, cosPts,
			  sinPts2, cosPts2,
			  wignersTrans, scratch ) ;

	  /* now apply the wigner transform to the
	     real and imaginary parts */

	  wigNaiveSynthesis( m1, m2, bw, t1r,
			     wignersTrans, t2r,
			     scratch ) ;

	  wigNaiveSynthesis( m1, m2, bw, t1i,
			     wignersTrans, t2i,
			     scratch ) ;
	  
	  /* advance the pointers by the appropriate
	     amount at these orders m1, m2 */
	  t1r += howMany_so3( m1, m2, bw ) ;
	  t1i += howMany_so3( m1, m2, bw ) ;
	  t2r += n ;
	  t2i += n ;
	}
    }

  /*
    Stage 1.5: reset pointers
  */
  t1r -= totalCoeffs_so3( bw ) ;
  t1i -= totalCoeffs_so3( bw ) ;
  t2r -= (n * n * n);
  t2i -= (n * n * n);

  /*
    Stage 2: transpose! Note I'm using the rdata, idata arrays
    as tmp space
  */
  
  transpose_so3( t2r, t1r, n, n*n ) ;
  transpose_so3( t2i, t1i, n, n*n ) ;


  /*
    Stage 3: FFT the "rows". Instead of treating the signal as
    3-D object, I can also think of it as an array of size
    (n^2) x n. This means all I'm doing in the first stage
    is taking n^2-many FFTs, each of length n.

    NOTE: Since I'm reusing the FFT code from SpharmonicKit,
    even though I'm doing the INVERSE SO(3) transform
    here, I need to call grid_fourier_so3  -> the signs
    on the complex exponentials are switched (detailed
    explanation to be put here eventually, but trust
    me)
  */
  
  grid_fourier_so3( t1r, t1i,
		    t2r, t2i,
		    n*n, n,
		    scratch ) ;

  /* normalize the Fourier coefficients (sorry, have to do it) */
  /*
    dn = sqrt( (double) n );
    dn = 1.0 ;
    for ( j = 0 ; j < n*n*n; j++ )
    {
    t2r[ j ] *= dn ;
    t2i[ j ] *= dn ;
    }
  */

  /*
    Stage 4: transpose! Note I'm using the rdata, idata arrays
    as tmp space
  */
  
  transpose_so3( t2r, t1r, n, n*n ) ;
  transpose_so3( t2i, t1i, n, n*n ) ;


  /*
    Stage 5: FFT again. Note that THIS TIME, the rdata, idata
    arrays will hold the final answers I want
  */

  grid_fourier_so3( t1r, t1i,
		    rdata, idata,
		    n*n, n,
		    scratch ) ;
  
  /* normalize the Fourier coefficients (sorry, have to do it) */
  // dn = (double) n ;
  // dn *= ( ((double) bw) / M_PI ) ;
  // dn *= ((double) bw);
  // dn = sqrt((double)n);
  dn = (double) n;
  dn *= ((double)bw)/M_PI;
  for ( j = 0 ; j < n*n*n; j++ )
    {
      rdata[ j ] *= dn ;
      idata[ j ] *= dn ;
    }

  /* and that's all, folks */

}
