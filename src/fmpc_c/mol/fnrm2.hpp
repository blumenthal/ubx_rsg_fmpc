/*
 * fnrm2.hpp
 *
 *  Created on: Jun 19, 2014
 *      Author: zhanglin
 */

#ifndef FNRM2_HPP_
#define FNRM2_HPP_


namespace MOL{

/*  FNRM2 returns the Euclidean norm of a vector via the function
    name, so that

       FNRM2 := sqrt( x'*x )
*/
template<typename opT, int opMROW, int opMCOL>
void fnrm2_ (const Matrix<opT, opMROW, opMCOL>& A, opT *res){
    for(int i=0;i<opMROW;i++)
        for(int j=0;j<opMCOL;j++)
        	*res+=A.elem[i][j]*A.elem[i][j];
    *res=sqrtf(*res);
    //TODO: Check efficiency in HLS
}

}//namespace MOL


#endif /* FNRM2_HPP_ */
