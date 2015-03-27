/*
 * ftrsm.hpp
 *
 *  Created on: Jun 19, 2014
 *      Author: zhanglin
 */

#ifndef FTRSM_HPP_
#define FTRSM_HPP_

#undef A
namespace MOL{

/*  Purpose
    =======
    DTRSM  solves one of the matrix equations
       op( A )*X = alpha*B,   or   X*op( A ) = alpha*B,
    where alpha is a scalar, X and B are m by n matrices, A is a unit, or
    non-unit,  upper or lower triangular matrix  and  op( A )  is one  of
       op( A ) = A   or   op( A ) = A'.
    The matrix X is overwritten on B.
*/

template<typename opT, int opBROW, int opBCOL, int opAN>
void ftrsm_ (const char SIDE, const char UPLO, const char At, const opT alpha,
		const Matrix<opT, opAN, opAN>& A, Matrix<opT, opBROW, opBCOL>& B){
    char selector=0;
    selector = SIDE * 4 + At * 2 + UPLO;
    int i,j,k;
    /* SIDE:    0 = Left side   -> A*x
     *          1 = Right side  -> x*A
     * At:      0, A is not Not transposed
     *          1, A is transposed
     * UPLO:    0, upper
     *          1, lower
     */
    opT temp=0.0;
#define A(I,J) A.elem[(I)-1][(J)-1]
#define B(I,J) B.elem[(I)-1][(J)-1]

    switch (selector){
        //inv(a)*B
        case 0: // inv(a)*B, upper

        for (j = 1; j <=opBCOL; ++j) {
                for (i = 1; i <= opAN; ++i) {
                    B(i,j) = alpha * B(i,j);
                }
            for (k = opAN; k >= 1; --k) {
                if (B(k,j) != (opT)0) {
                    B(k,j) /= A(k,k);
                    for (i = 1; i <= k-1; ++i) {
                        B(i,j) -= B(k,j) * A(i,k);
                    }
                }
            }
        }
        break;
        case 1: // inv(a)*B, lower
        for (j = 1; j <= opBCOL; ++j) {
            for (i = 1; i <= opAN; ++i) {
                B(i,j) = alpha * B(i,j);
            }
            for (k = 1; k <= opAN; ++k) {
                B(k,j) /= A(k,k);
                for (i = k + 1; i <= opAN; ++i) {
                    B(i,j) -= B(k,j) * A(i,k);
                }
            }
        }
        break;

        //inv(a')*B
        case 2: // inv(a')*B, upper
        for (j = 1; j <= opBCOL; ++j) {
            for (i = 1; i <= opAN; ++i) {
                temp = alpha * B(i,j);
                for (k = 1; k <= i-1; ++k) {
                    temp -= A(k,i) * B(k,j);
                }
                temp /= A(i,i);
                B(i,j) = temp;
            }
	}
        break;
        case 3:// inv(a')*B, lower

        for (j = 1; j <= opBCOL; ++j) {
            for (i = opAN; i >= 1; --i) {
                temp = alpha * B(i,j);
                for (k = i + 1; k <= opAN; ++k) {
                    temp -= A(k,i) * B(k,j);
                }
                temp /= A(i,i);
                B(i,j) = temp;
            }
        }
        break;
        //B*inv(a)
        case 4:// B*inv(a), upper
        for (j = 1; j <= opBCOL; ++j) {
        		for (i = 1; i <= opBROW; ++i) {
        			B(i,j) = alpha * B(i,j);
        		}
            for (k = 1; k <= j-1; ++k) {
                if (A(k,j) != 0.) {
                    for (i = 1; i <= opBROW; ++i) {
                        B(i,j) -= A(k,j) * B(i,k);
                    }
                }
            }
            temp = (opT)1. / A(j,j);
            for (i = 1; i <= opBROW; ++i) {
                B(i,j) = temp * B(i,j);
            }
        }
        break;

        case 5:// B*inv(a), lower
        for (j = opBCOL; j >= 1; --j) {
                for (i = 1; i <= opBROW; ++i) {
                    B(i,j) = alpha * B(i,j);
                }
            for (k = j + 1; k <= opBCOL; ++k) {
                for (i = 1; i <= opBROW; ++i) {
                    B(i,j) -= A(k,j) * B(i,k);
                }
            }
            temp = (opT)1. / A(j,j);
            for (i = 1; i <= opBROW; ++i) {
                B(i,j) = temp * B(i,j);
            }
        }
        break;

        case 6:// B*inv(a'), upper
        for (k = opBCOL; k >= 1; --k) {
                temp = (opT)1. / A(k,k);
                for (i = 1; i <= opBROW; ++i) {
                    B(i,k) = temp * B(i,k);
                }
            for (j = 1; j <= k-1; ++j) {
                    for (i = 1; i <= opBROW; ++i) {
                        B(i,j) -= A(j,k) * B(i,k);
                    }
            }
                for (i = 1; i <= opBROW; ++i) {
                    B(i,k) = alpha * B(i,k);
                }
        }
        break;

        case 7:// B*inv(a'), lower
        for (k = 1; k <= opBCOL; ++k) {
            temp = (opT)1. / A(k,k);
            for (i = 1; i <= opBROW; ++i) {
                B(i,k) = temp * B(i,k);
            }
            for (j = k + 1; j <= opBCOL; ++j) {
                if (A(j,k) != 0.) {
                    for (i = 1; i <= opBROW; ++i) {
                        B(i,j) -= A(j,k) * B(i,k);

                    }
                }
            }
            for (i = 1; i <= opBROW; ++i) {
                B(i,k) = alpha * B(i,k);
            }
        }
        break;
    }

}
}//namespace MOL



#endif /* FTRSM_HPP_ */
