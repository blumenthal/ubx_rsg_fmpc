/*
 * fdosv.hpp
 *
 *  Created on: Jun 19, 2014
 *      Author: zhanglin
 */

#ifndef FDOSV_HPP_
#define FDOSV_HPP_

namespace MOL{
/*
Purpose
=======
FPOSV computes the solution to a real system of linear equations
   A * X = B,
where A is an N-by-N symmetric positive definite matrix and X and B
are N-by-NRHS matrices.

The Cholesky decomposition is used to factor A as
   A = U**T* U,  if UPLO = 'U', or
   A = L * L**T,  if UPLO = 'L',
where U is an upper triangular matrix and L is a lower triangular
matrix.  The factored form of A is then used to solve the system of
equations A * X = B.
*/

template<typename opT, int opAN, int opBROW, int opBCOL>
void fposv_ (const char uplo, Matrix<opT, opAN, opAN>& A ,
                        Matrix<opT, opBROW, opBCOL>& B){
    fpotf2_(uplo,A);
    //FIXME: This is a work around due to HLS compiler bug, originally it should be dpotrf.
    fpotrs_(uplo,A,B);
}
}//namespace MOL


#endif /* FDOSV_HPP_ */
