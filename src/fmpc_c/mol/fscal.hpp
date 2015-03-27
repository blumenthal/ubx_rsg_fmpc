/*
 * fscal.hpp
 *
 *  Created on: Jun 19, 2014
 *      Author: zhanglin
 */

#ifndef FSCAL_HPP_
#define FSCAL_HPP_

namespace MOL{
template<typename opT, int opMROW, int opMCOL>
void fscal_ (const opT DA, Matrix<opT, opMROW, opMCOL>& DX){
     DX*=DA;
}

template<typename opT, int opXROW, int opXCOL>
void fscal_ (const type_f DA, Matrix<opT,opXROW,opXCOL>& DX,
		const int DXi, const int DXj, const int DXm, const int DXn){
#define DXsub(I,J) DX.elem[(I) + DXi][(J) + DXj]
    for(int i=0;i<DXm;i++)
        for(int j=0;j<DXn;j++){
                DXsub(i,j)=DA*DXsub(i,j);
        }
}
}//namespace MOL


#endif /* FSCAL_HPP_ */
