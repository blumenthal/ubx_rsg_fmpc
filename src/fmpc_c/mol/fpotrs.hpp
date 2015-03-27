/*
 * fpotrs.hpp
 *
 *  Created on: Jun 19, 2014
 *      Author: zhanglin
 */

#ifndef FPOTRS_HPP_
#define FPOTRS_HPP_


namespace MOL{
/*
 *    Solve a system of linear equations A*X = B with a
      symmetric positive definite matrix A using the Cholesky
      factorization A = U**T*U or A = L*L**T.
 */

template<typename opT, int opBROW, int opBCOL, int opAN>
void fpotrs_ (const char UPLO, const Matrix<opT, opAN, opAN>& A, Matrix<opT, opBROW, opBCOL>& B){
    switch (UPLO){
        case 0:
            ftrsm_(MOL_MAT_LEFT, MOL_UP_TRI, MOL_TRANS, (type_f)1.0, A, B);
            ftrsm_(MOL_MAT_LEFT, MOL_UP_TRI, MOL_NO_TRANS, (type_f)1.0, A, B);
            break;
        case 1:
            ftrsm_(MOL_MAT_LEFT, MOL_LO_TRI, MOL_NO_TRANS, (type_f)1.0, A, B);
            ftrsm_(MOL_MAT_LEFT, MOL_LO_TRI, MOL_TRANS, (type_f)1.0, A, B);
            break;
    }
}
}//namespace MOL



#endif /* FPOTRS_HPP_ */
