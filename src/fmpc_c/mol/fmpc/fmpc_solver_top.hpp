/*
 * fmpc_solver_top.hpp
 *
 *  Created on: Jun 20, 2014
 *      Author: zhanglin
 */

#ifndef FMPC_SOLVER_TOP_HPP_
#define FMPC_SOLVER_TOP_HPP_

#include "../mol.hpp"
#include "../mol_blas.hpp"
#include "fmpc_functions.hpp"
using namespace MOL;

void fmpc_solver_top(
#ifdef ARR_1D
		char fs,
		type_f A_a[FMPC_GN*FMPC_GN], type_f B_a[FMPC_GN*FMPC_GM],
		type_f xmax_a[FMPC_GT*FMPC_GN], type_f xmin_a[FMPC_GT*FMPC_GN],
		type_f umax_a[FMPC_GT*FMPC_GM], type_f umin_a[FMPC_GT*FMPC_GM],
		type_f Q_a[FMPC_GN*FMPC_GN], type_f R_a[FMPC_GM*FMPC_GM], type_f Qf_a[FMPC_GN*FMPC_GN],
		type_f kappa,
		int niter,
		type_f X_a[FMPC_GT*FMPC_GN],type_f U_a[FMPC_GT*FMPC_GM], type_f x_a[FMPC_GN*1],
		type_f X_ao[FMPC_GT*FMPC_GN],type_f U_ao[FMPC_GT*FMPC_GM]
#else
		char fs,
		type_f A_a[FMPC_GN][FMPC_GN], type_f B_a[FMPC_GN][FMPC_GM],
		type_f xmax_a[FMPC_GT][FMPC_GN], type_f xmin_a[FMPC_GT][FMPC_GN],
		type_f umax_a[FMPC_GT][FMPC_GM], type_f umin_a[FMPC_GT][FMPC_GM],
		type_f Q_a[FMPC_GN][FMPC_GN], type_f R_a[FMPC_GM][FMPC_GM], type_f Qf_a[FMPC_GN][FMPC_GN],
		type_f kappa,
		int niter,
		type_f X_a[FMPC_GT][FMPC_GN],type_f U_a[FMPC_GT][FMPC_GM], type_f x_a[FMPC_GN][1],
		type_f X_ao[FMPC_GT][FMPC_GN],type_f U_ao[FMPC_GT][FMPC_GM]
#endif
		);

void fmpc_solver(	Matrix<type_f,FMPC_GN,FMPC_GN>& A, 		Matrix<type_f,FMPC_GN,FMPC_GM>& B,
					Matrix<type_f,FMPC_GN,FMPC_GN>& Q, 		Matrix<type_f,FMPC_GM,FMPC_GM>& R, Matrix<type_f,FMPC_GN,FMPC_GN>& Qf,
					Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& zmax, Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& zmin,
					Matrix<type_f,FMPC_GN,1>& x, 		Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& z0,
					int niters, type_f kappa);
void fmpc_solver_top_fpga(
#ifdef ARR_1D
		XFmpc_solver_top *XFmpc,
		char fs,
		type_f A_a[FMPC_GN*FMPC_GN], type_f B_a[FMPC_GN*FMPC_GM],
		type_f xmax_a[FMPC_GT*FMPC_GN], type_f xmin_a[FMPC_GT*FMPC_GN],
		type_f umax_a[FMPC_GT*FMPC_GM], type_f umin_a[FMPC_GT*FMPC_GM],
		type_f Q_a[FMPC_GN*FMPC_GN], type_f R_a[FMPC_GM*FMPC_GM], type_f Qf_a[FMPC_GN*FMPC_GN],
		type_f kappa,
		int niter,
		type_f X_a[FMPC_GT*FMPC_GN],type_f U_a[FMPC_GT*FMPC_GM], type_f x_a[FMPC_GN*1],
		type_f X_ao[FMPC_GT*FMPC_GN],type_f U_ao[FMPC_GT*FMPC_GM]
#else
		char fs,
		type_f A_a[FMPC_GN][FMPC_GN], type_f B_a[FMPC_GN][FMPC_GM],
		type_f xmax_a[FMPC_GT][FMPC_GN], type_f xmin_a[FMPC_GT][FMPC_GN],
		type_f umax_a[FMPC_GT][FMPC_GM], type_f umin_a[FMPC_GT][FMPC_GM],
		type_f Q_a[FMPC_GN][FMPC_GN], type_f R_a[FMPC_GM][FMPC_GM], type_f Qf_a[FMPC_GN][FMPC_GN],
		type_f kappa,
		int niter,
		type_f X_a[FMPC_GT][FMPC_GN],type_f U_a[FMPC_GT][FMPC_GM], type_f x_a[FMPC_GN][1],
		type_f X_ao[FMPC_GT][FMPC_GN],type_f U_ao[FMPC_GT][FMPC_GM]
#endif
		);

void fmpc_solver_top_assign_variables(
                XFmpc_solver_top *XFmpc,
                char fs,
                type_f A_a[FMPC_GN*FMPC_GN], type_f B_a[FMPC_GN*FMPC_GM],
                type_f xmax_a[FMPC_GT*FMPC_GN], type_f xmin_a[FMPC_GT*FMPC_GN],
                type_f umax_a[FMPC_GT*FMPC_GM], type_f umin_a[FMPC_GT*FMPC_GM],
                type_f Q_a[FMPC_GN*FMPC_GN], type_f R_a[FMPC_GM*FMPC_GM], type_f Qf_a[FMPC_GN*FMPC_GN],
                type_f kappa,
                int niter,
                type_f X_a[FMPC_GT*FMPC_GN],type_f U_a[FMPC_GT*FMPC_GM], type_f x_a[FMPC_GN*1]
);

void fmpc_solver_top_acquire_result(
        XFmpc_solver_top *XFmpc,
        type_f X_ao[FMPC_GT*FMPC_GN],type_f U_ao[FMPC_GT*FMPC_GM]
);

#endif /* FMPC_SOLVER_TOP_HPP_ */
