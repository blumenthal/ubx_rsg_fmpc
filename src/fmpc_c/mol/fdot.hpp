/*
 * fdot.hpp
 *
 *  Created on: Jun 19, 2014
 *      Author: zhanglin
 */

#ifndef FDOT_HPP_
#define FDOT_HPP_

namespace MOL{
/*     forms the dot product of two vectors.
       uses unrolled loops for increments equal to one.
*/

template<typename T, int NROW, int NCOL>
T fdot_ (const Matrix<T,NROW,NCOL>& DX, const Matrix<T,NROW,NCOL>& DY){
    T res=(T)0.0;
    for(int i=0;i<NROW;i++)
        for(int j=0;j<NCOL;j++)
        	res+=DX.elem[i][j]*DY.elem[i][j];
    return res;
}

/*     forms the dot product of two vectors.
       uses unrolled loops for increments equal to one.

	   This template performs _sub vector_ dot product.
*/

template<typename T, int NROW, int NCOL>
T fdot_ (
		const Matrix<T,NROW,NCOL>& DX,const int DXi,const int DXj,
		const Matrix<T,NROW,NCOL>& DY,const int DYi,const int DYj,const int Dm,const int Dn){
#define DXsub(I,J) DX.elem[(I) + DXi][(J) + DXj] //already tested.
#define DYsub(I,J) DY.elem[(I) + DYi][(J) + DYj]

	T res=(T)0.0;
    for(int i=0;i<Dm;i++){
        for(int j=0;j<Dn;j++){
        	res+=DXsub(i,j)*DYsub(i,j);
#ifdef DEBUG_FDOT
        printf("DXsub(%d,%d)=%8.3f\t*\tDYsub(%d,%d)=%8.3f\t=%8.3f\n",i,j,DYsub(i,j),i,j,DXsub(i,j),DXsub(i,j)*DYsub(i,j));
#endif
        }
    }
    return res;
}
}//namespace MOL

#endif /* FDOT_HPP_ */

