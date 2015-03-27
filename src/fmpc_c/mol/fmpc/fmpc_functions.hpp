/*
 * fmpc_functions.hpp
 *
 *  Created on: Jun 24, 2014
 *      Author: zhanglin
 */

#ifndef FMPC_FUNCTIONS_HPP_
#define FMPC_FUNCTIONS_HPP_
#include "../mol.hpp"
#include "../mol_blas.hpp"
#include "fmpc_solver_top.hpp"

using namespace MOL;
/*
 * Overall optimization variable: 
 * z=(u(t),x(t+1), ... ,u(t+T-1), x(t+T)) in R, dim [T(m+n)]
 * 
 * minimize: z_transpose * H * z + g_transpose * z
 * subject to: Pz<=h, Cz=b
 */

/* Prepare terms for residual calculation.
 * Pz<=h is approximated by log barrier, kappa * phi(z)
 * 	    lT+kappa
 * phi(z) = SUM OF(Sigma symbol)  (- log (h_i-p_i_transpose * z))		
 *	    i=1
 */
void gfgphp(Matrix<type_f,FMPC_GN,FMPC_GN>& Q, Matrix<type_f,FMPC_GM,FMPC_GM>& R, Matrix<type_f,FMPC_GN,FMPC_GN>& Qf,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& zmax, Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& zmin,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& z,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& gf,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& gp,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& hp);
/* Compute primal and dual residual, rp and rd
 * rp=Cz-b
 * rd=2Hz+g+kappa*P_transpose*d+C_transpose*nu	
 */
void rdrp(	Matrix<type_f,FMPC_GN,FMPC_GN>& A, Matrix<type_f,FMPC_GN,FMPC_GM>& B,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& z, Matrix<type_f,FMPC_GT,FMPC_GN>& nu,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& gf, Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& gp,
			Matrix<type_f,FMPC_GT,FMPC_GN>& b,
			type_f kappa, Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& rd, Matrix<type_f,FMPC_GT,FMPC_GN>& rp, Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& Ctnu);

void resdresp(Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& rd, Matrix<type_f,FMPC_GT,FMPC_GN>& rp, type_f *resd, type_f *resp, type_f *res);


/* Solving linear equations

	| 2H+kappa*P_transpose*diag(d^2)*P	C_transpose	| |dz |     |rd|
	|							| |   |  = -|  |
	|		C			     0		| |dnu|     |rp|

*/
void dnudz(	Matrix<type_f,FMPC_GN,FMPC_GN>& A, 		Matrix<type_f,FMPC_GN,FMPC_GM>& B,
			Matrix<type_f,FMPC_GN,FMPC_GN>& Q, 		Matrix<type_f,FMPC_GM,FMPC_GM>& R, 		Matrix<type_f,FMPC_GN,FMPC_GN>& Qf,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& hp,	Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& rd, 	Matrix<type_f,FMPC_GT,FMPC_GN>& rp,
			type_f	kappa,				Matrix<type_f,FMPC_GT,FMPC_GN>& dnu,	Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& dz);
#endif /* FMPC_FUNCTIONS_HPP_ */
