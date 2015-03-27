/*
 * fgemv.hpp
 *
 *  Created on: Jun 19, 2014
 *      Author: zhanglin
 */

#ifndef FGEMV_HPP_
#define FGEMV_HPP_


namespace MOL{
#undef ASub
#undef BSub
#undef CSub

/*  Purpose
    =======
    FGEMV  performs one of the matrix-vector operations
       y := alpha*A*x + beta*y,   or   y := alpha*A'*x + beta*y,
    where alpha and beta are scalars, x and y are vectors and A is an
    m by n matrix.
*/
template<typename opT, int opAROW, int opACOL, int opBROW, int opCROW>
void fgemv_ (const TransT At, const opT alpha,
		const Matrix<opT, opAROW, opACOL>& A,
		const Matrix<opT, opBROW, 1>& B, const opT beta,
        Matrix<opT, opCROW, 1>& C){

#define ASub(I,J) A.elem[(I)][(J)]
#define BSub(I) B.elem[(I)][0]
#define CSub(I) C.elem[(I)][0]

	int i,k;
	opT temp;
        int M,K;
        opT mm,nn;
        for (int i=0;i<opCROW;i++)
            CSub(i)*=beta;

        switch (At){
            case (TransT)0:
                M=opAROW;
                K=opACOL;
                break;
            case (TransT)1:
                M=opACOL;
                K=opAROW;
                break;
            default:
                M=opAROW;
                K=opACOL;
                break;
        }
	for(i = 0; i < M; i++){
                temp = 0;
                for(k = 0; k < K; k++){
                    switch (At){
                        case 0:
                                mm=ASub(i,k);
                            break;
                        case 1:
                                mm=ASub(k,i);
                            break;
                        default:
                                mm=ASub(i,k);
                            break;
                    }
                        nn=BSub(k);
                    temp += mm*nn;
                }
                        CSub(i)=alpha*temp+CSub(i);
	}
}


/*  Purpose
    =======
    DGEMV  performs one of the matrix-vector operations
       y := alpha*A*x + beta*y,   or   y := alpha*A'*x + beta*y,
    where alpha and beta are scalars, x and y are vectors and A is an
    m by n matrix.

	This template allows operations on sub matrices.

*/
#undef ASub
#undef BSub
#undef CSub

template<typename opT, int opAROW, int opACOL, int opBROW, int opBCOL, int opCROW, int opCCOL>
void fgemv_ (const TransT At, const opT alpha,
		const Matrix<opT, opAROW, opACOL>& A, const int Ai, const int Aj, const int ASubRow, const int ASubCol,
        Matrix<opT, opBROW, opBCOL>& B, const int Bi, const int Bj, const int BSubRow, const int BSubCol,
        const opT beta,
        Matrix<opT, opCROW, opCCOL>& C, const int Ci, const int Cj, const int CSubRow, const int CSubCol){

#define ASub(I,J) A.elem[(I) + Ai][(J) + Aj]
#define BSub(I,J) B.elem[(I) + Bi][(J) + Bj]
#define CSub(I,J) C.elem[(I) + Ci][(J) + Cj]


        for (int i=0;i<CSubRow;i++)
        for (int j=0;j<CSubCol;j++){
            CSub(i,j)*=beta;
        }

	int i,j,k;
	opT temp;
        int M,N,K;
        opT mm,nn;
        switch (At){
            case 0:
                M=ASubRow;
                N=1;
                K=ASubCol;
                break;
            case 1:
                M=ASubCol;
                N=1;
                K=ASubRow;
                break;
            default:
                M=ASubRow;
                N=1;
                K=ASubCol;
                break;
        }

	for(i = 0; i < M; i++){
            for(j = 0; j < N; j++){
                temp = 0;
                for(k = 0; k < K; k++){
                    switch (At){
                        case 0:
                                mm=ASub(i,k);
                            break;
                        case 1:
                                mm=ASub(k,i);
                            break;
                        default:
                                mm=ASub(i,k);
                            break;
                    }

                    if(BSubRow>BSubCol)
                        nn=BSub(k,j);
                    else
                        nn=BSub(j,k);


                    temp += mm*nn;
#ifdef DEBUG_FGEMV
                    printf("i=%d,j=%d,k=%d,\t mm=%10.3f,nn=%10.3f\n",i,j,k,mm,nn);
#endif
                }

                if(CSubRow==1)
                        CSub(j,i)=alpha*temp+CSub(j,i);
                else
                        CSub(i,j)=alpha*temp+CSub(i,j);
            }
	}
}
}//namespace MOL


#endif /* FGEMV_HPP_ */
