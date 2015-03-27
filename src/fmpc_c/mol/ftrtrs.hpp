/*
 * ftrtrs.hpp
 *
 *  Created on: Jun 19, 2014
 *      Author: zhanglin
 */

#ifndef FTRTRS_HPP_
#define FTRTRS_HPP_


namespace MOL{
/*
Purpose
=======

DTRTRS solves a triangular system of the form

   A * X = B  or  A**T * X = B,

where A is a triangular matrix of order N, and B is an N-by-NRHS
matrix.
*/
template<typename opT, int opBROW, int opBCOL, int opAN>
void ftrtrs_ (const char uplo, const char At, const Matrix<opT, opAN, opAN>& A, Matrix<opT, opBROW, opBCOL>& B){
    ftrsm_(MOL_MAT_LEFT, uplo, At, (opT)1, A, B);        //TODO, FIXME: check "Left"=0?
}
}//namespace MOL


#endif /* FTRTRS_HPP_ */
