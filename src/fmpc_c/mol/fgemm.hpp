/*
 * fgemm.hpp
 *
 *  Created on: Jun 19, 2014
 *      Author: zhanglin
 */

#ifndef FGEMM_HPP_
#define FGEMM_HPP_

namespace MOL{
/*  Purpose
    =======
    fgemm  performs one of the matrix-matrix operations
       C := alpha*op( A )*op( B ) + beta*C,
    where  op( X ) is one of
       op( X ) = X   or   op( X ) = X',
    alpha and beta are scalars, and A, B and C are matrices, with op( A )
    an m by k matrix,  op( B )  a  k by n matrix and  C an m by n matrix.
*/
template<typename T, int opMROW, int opMCOL,
        int opNROW, int opNCOL, int opRROW, int opRCOL>
void fgemm_ (const TransT TransAB, const T alpha,
        const Matrix<T, opMROW, opMCOL>& A,
        const Matrix<T, opNROW, opNCOL>& B, const T beta,
        Matrix<T, opRROW, opRCOL>& C){

#ifndef __SYNTHESIS__
	if(TransAB>3||TransAB<0) printf("[Failed] Transpose must be [0, 1, 2 or 3]\n");
	//TODO: assert?
#endif

		C*=beta;
		T temp;
        int MM=0,NN=0,KK=0;
        switch (TransAB){
            case (TransT)0:
                MM=opMROW;
                NN=opNCOL;
                KK=opMCOL;
                break;
            case (TransT)1:
                MM=opMROW;
                NN=opNROW;
                KK=opNCOL;
                break;
            case (TransT)2:
                MM=opMCOL;
                NN=opNCOL;
                KK=opMROW;
                break;
            case (TransT)3:
                MM=opMCOL;
                NN=opNROW;
                KK=opMROW;
                break;
            default:
                MM=opMROW;
                NN=opNCOL;
                KK=opMCOL;
                break;
        }
        for(int i = 0; i < MM; i++){
                   for(int j = 0; j < NN; j++){
                       temp = 0;
                       for(int k = 0; k < KK; k++){
                    	   temp+=elemTranspose(A,i,k,TransAB>>1&1)*elemTranspose(B,k,j,TransAB&1);
                       }
                       C.elem[i][j]+=alpha*temp;
                   }
       	}
}


/*  Purpose
    =======
    fgemm  performs one of the matrix-matrix operations
       C := alpha*op( A )*op( B ) + beta*C,
    where  op( X ) is one of
       op( X ) = X   or   op( X ) = X',
    alpha and beta are scalars, and A, B and C are matrices, with op( A )
    an m by k matrix,  op( B )  a  k by n matrix and  C an m by n matrix.

	This template allows operations on sub matrices.
*/
template<typename T, int opAROW, int opACOL, int opBROW, int opBCOL, int opCROW, int opCCOL>
void fgemm_ (const TransT TransAB, T alpha, const
        Matrix<T, opAROW, opACOL>& A, int Ai, int Aj, int ASubnRow, int ASubnCol,
        Matrix<T, opBROW, opBCOL>& B, int Bi, int Bj, int BSubnRow, int BSubnCol,
        T beta,
        Matrix<T, opCROW, opCCOL>& C, int Ci, int Cj, int CSubnRow, int CSubnCol){

#define ASub(I,J) A.elem[(I)+Ai][(J)+Aj]
#define BSub(I,J) B.elem[(I)+Bi][(J)+Bj]
#define CSub(I,J) C.elem[(I)+Ci][(J)+Cj]

        for (int i=0;i<CSubnRow;i++)
        for (int j=0;j<CSubnCol;j++){
            CSub(i,j)*=beta;
        }
	int i,j,k;
	T temp;
        int MM,NN,KK;
        T mm,nn;
        switch (TransAB){
            case (TransT)0:
                MM=ASubnRow;
                NN=BSubnCol;
                KK=ASubnCol;
                break;
            case (TransT)1:
                MM=ASubnRow;
                NN=BSubnRow;
                KK=BSubnCol;
                break;
            case (TransT)2:
                MM=ASubnCol;
                NN=BSubnCol;
                KK=ASubnRow;
                break;
            case (TransT)3:
                MM=ASubnCol;
                NN=BSubnRow;
                KK=ASubnRow;
                break;
            default:
                MM=ASubnRow;
                NN=BSubnCol;
                KK=ASubnCol;
                break;
        }
	for(i = 0; i < MM; i++){
            for(j = 0; j < NN; j++){
                temp = 0;
                for(k = 0; k < KK; k++){
                    switch (TransAB){
                        case (TransT)0:
                                mm=ASub(i,k); nn=BSub(k,j);
                            break;
                        case (TransT)1:
                                mm=ASub(i,k); nn=BSub(j,k);
                            break;
                        case (TransT)2:
                                mm=ASub(k,i); nn=BSub(k,j);
                            break;
                        case (TransT)3:
                                mm=ASub(k,i); nn=BSub(j,k);
                            break;
                        default:
                                mm=ASub(i,k); nn=BSub(k,j);
                            break;
                    }
                    temp += mm*nn;
                }
               CSub(i,j)+=alpha*temp;
            }
	}
}

}//namespace MOL

#endif /* FGEMM_HPP_ */
