/*
 * faxpy.hpp
 *
 *  Created on: Jun 19, 2014
 *      Author: zhanglin
 */

#ifndef FAXPY_HPP_
#define FAXPY_HPP_

namespace MOL{
/*     constant times a vector plus a vector.
       uses unrolled loops for increments equal to one.
*/
template<typename T, int NROW, int NCOL>
void faxpy_ (const T DA, const Matrix<T, NROW, NCOL>& DX, Matrix<T, NROW, NCOL>& DY){
    DY+=DX*DA;
}
}//namespace MOL

#endif /* FAXPY_HPP_ */
