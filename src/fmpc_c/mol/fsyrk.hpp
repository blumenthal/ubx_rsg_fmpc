/*
 * fsyrk.hpp
 *
 *  Created on: Jun 19, 2014
 *      Author: zhanglin
 */

#ifndef FSYRK_HPP_
#define FSYRK_HPP_

#undef A
namespace MOL{
/*  Purpose
    =======
    DSYRK  performs one of the symmetric rank k operations
       C := alpha*A*A' + beta*C,
    or

       C := alpha*A'*A + beta*C,
    where  alpha and beta  are scalars, C is an  n by n  symmetric matrix
    and  A  is an  n by k  matrix in the first case and a  k by n  matrix
    in the second case.
*/

template<typename opT, int opAROW, int opACOL, int opCN>
void fsyrk_ (const char UPLO, const char At, const opT alpha,
        const Matrix<opT, opAROW, opACOL>& A, const opT beta, Matrix<opT, opCN, opCN>& C){
        char selector=0;
        selector = At * 2 + UPLO;
        printf("case %d, At=%d, UPLO=%d .. ", selector, At, UPLO);
        int i,j,l;
        opT temp;
#define A(I,J) A.elem[(I)-1][(J)-1]
#define C(I,J) C.elem[(I)-1][(J)-1]

        switch(selector){
            case 0:
 	    for (j = 1; j <= opCN; ++j) {
                for (i = 1; i <= j; ++i) {
                    C(i,j) = beta * C(i,j);
                }
		for (l = 1; l <= opACOL; ++l) {
			temp = alpha * A(j,l);
			for (i = 1; i <= j; ++i) {
			    C(i,j) += temp * A(i,l);
			}
                }
            }
            break;
            case 1:
	    for (j = 1; j <= opCN; ++j) {
                for (i = j; i <= opCN; ++i) {
                    C(i,j) = beta * C(i,j);
                }
		for (l = 1; l <= opACOL; ++l) {
		    if (A(j,l) != (opT)0) {
			temp = alpha * A(j,l);
			for (i = j; i <= opCN; ++i) {
			    C(i,j) += temp * A(i,l);
			}
		    }
		}
	    }
            break;
            case 2:
            for (j = 1; j <= opCN; ++j) {
		for (i = 1; i <= j; ++i) {
		    temp = (opT)0.;
		    for (l = 1; l <= opAROW; ++l) {
			temp += A(l,i) * A(l,j);
		    }
		    if (beta == (opT)0) {
			C(i,j) = alpha * temp;
		    } else {
			C(i,j) = alpha * temp + beta * C(i,j);
		    }
		}
	    }
            break;
            case 3:
            for (j = 1; j <= opCN; ++j) {
		for (i = j; i <= opCN; ++i) {
		    temp = (opT)0.;
		    for (l = 1; l <= opAROW; ++l) {
			temp += A(l,i) * A(l,j);
		    }
		    if (beta == (opT)0) {
			C(i,j) = alpha * temp;
		    } else {
			C(i,j) = alpha * temp + beta * C(i,j);
		    }
		}
	    }
            break;
        }
}
}//namespace MOL


#endif /* FSYRK_HPP_ */
