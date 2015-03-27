/*
 * fpotf2.hpp
 *
 *  Created on: Jun 19, 2014
 *      Author: zhanglin
 */

#ifndef FPOTF2_HPP_
#define FPOTF2_HPP_


namespace MOL{

template <typename opT, int opAN>
void fpotf2_ (const char UPLO, Matrix<opT, opAN, opAN>& A){
#define A(I,J) A.elem[(I)-1][(J)-1]
        opT ajj=0.0;
        opT temp;
        switch (UPLO){
            case 0:
    /*        Compute the Cholesky factorization A = U'*U. */
            for (int j = 1; j <= opAN; ++j) {
    /*           Compute U(J,J) and test for non-positive-definiteness. */
                temp=fdot_(A,0,j-1,A,0,j-1,j-1,1);
                ajj=A(j,j)-temp;
                ajj=sqrtf(ajj);
                A(j,j)=ajj;
    /*           Compute elements J+1:N of row J. */
                if (j < opAN) {
                      fgemv_(
                    (TransT)1,(type_f)-1,  A,0,j,j-1,opAN-j,
                            A,0,j-1,j-1,1,   (type_f)1,    A,j-1,j,1,j-1);
                    fscal_(1/ajj,A,j-1,j,1,opAN-j);
                }
            }
                break;
            case 1:
    /*        Compute the Cholesky factorization A = U'*U. */
            for (int j = 1; j <= opAN; ++j) {
    /*           Compute U(J,J) and test for non-positive-definiteness. */
                temp=fdot_(A,j-1,0,A,j-1,0,1,j-1);
                ajj=A(j,j)-temp;
                ajj=sqrt(ajj);
                A(j,j)=ajj;
    /*           Compute elements J+1:N of row J. */
                if (j < opAN) {

                      fgemv_(   (TransT)0,(type_f)-1,
                                A,j,0,opAN-j,j-1,
                                A,j-1,0,1,j-1,
                                (type_f)1,
                                A,j,j-1,opAN-j,1);

                    fscal_(1/ajj,A,j,j-1,opAN-j,1);
                }
            }
                break;
    }

}

}//namespace TMatrix



#endif /* FPOTF2_HPP_ */
