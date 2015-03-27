/*
 * fmpc_solver_top.cpp
 *
 *  Created on: Jun 20, 2014
 *      Author: zhanglin
 */
#define DEBUG
#include "fmpc_solver_top.hpp"
#include <time.h>
using namespace MOL;
/*
 *
 * This function is the top to be convereted to IP core
 * in Vivado HLS
 *
 * */

#define INIT_PHASE 0
Matrix<type_f,FMPC_GN,FMPC_GN> A;
Matrix<type_f,FMPC_GN,FMPC_GM> B;

Matrix<type_f,FMPC_GN,FMPC_GN> Q;
Matrix<type_f,FMPC_GM,FMPC_GM> R;
Matrix<type_f,FMPC_GN,FMPC_GN> Qf;

Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> zmax;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> zmin;
Matrix<type_f,FMPC_GN,1> x;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> z;
Matrix<type_f,1,FMPC_GM> umax;
Matrix<type_f,1,FMPC_GM> umin;
Matrix<type_f,1,FMPC_GN> xmax;
Matrix<type_f,1,FMPC_GN> xmin;
Matrix<type_f,1,FMPC_GM> umaxp;
Matrix<type_f,1,FMPC_GM> uminp;
Matrix<type_f,1,FMPC_GN> xmaxp;
Matrix<type_f,1,FMPC_GN> xminp;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> zmaxp;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> zminp;
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
		){
#ifdef HLS_SYNTHESIS
#pragma HLS RESOURCE variable=fs core=AXI4LiteS
#pragma HLS RESOURCE variable=A_a core=AXI4LiteS
#pragma HLS RESOURCE variable=B_a core=AXI4LiteS
#pragma HLS RESOURCE variable=xmax_a core=AXI4LiteS
#pragma HLS RESOURCE variable=xmin_a core=AXI4LiteS
#pragma HLS RESOURCE variable=umax_a core=AXI4LiteS
#pragma HLS RESOURCE variable=umin_a core=AXI4LiteS
#pragma HLS RESOURCE variable=Q_a core=AXI4LiteS
#pragma HLS RESOURCE variable=R_a core=AXI4LiteS
#pragma HLS RESOURCE variable=Qf_a core=AXI4LiteS
#pragma HLS RESOURCE variable=kappa core=AXI4LiteS
#pragma HLS RESOURCE variable=niter core=AXI4LiteS
#pragma HLS RESOURCE variable=X_a core=AXI4LiteS
#pragma HLS RESOURCE variable=U_a core=AXI4LiteS
#pragma HLS RESOURCE variable=x_a core=AXI4LiteS
#pragma HLS RESOURCE variable=X_ao core=AXI4LiteS
#pragma HLS RESOURCE variable=U_ao core=AXI4LiteS
#pragma HLS INTERFACE ap_none port=X_ao
#pragma HLS INTERFACE ap_none port=U_ao
#pragma HLS ARRAY_PARTITION variable=A_a complete dim=1
#pragma HLS ARRAY_PARTITION variable=B_a complete dim=1
#pragma HLS ARRAY_PARTITION variable=xmax_a complete dim=1
#pragma HLS ARRAY_PARTITION variable=xmin_a complete dim=1
#pragma HLS ARRAY_PARTITION variable=umax_a complete dim=1
#pragma HLS ARRAY_PARTITION variable=umin_a complete dim=1
#pragma HLS ARRAY_PARTITION variable=Q_a complete dim=1
#pragma HLS ARRAY_PARTITION variable=R_a complete dim=1
#pragma HLS ARRAY_PARTITION variable=Qf_a complete dim=1
#pragma HLS ARRAY_PARTITION variable=X_a complete dim=1
#pragma HLS ARRAY_PARTITION variable=U_a complete dim=1
#pragma HLS ARRAY_PARTITION variable=x_a complete dim=1
#pragma HLS ARRAY_PARTITION variable=X_ao complete dim=1
#pragma HLS ARRAY_PARTITION variable=U_ao complete dim=1
#endif
		if(fs==INIT_PHASE){
			A<<A_a;
			B<<B_a;
			Q<<Q_a;
			R<<R_a;
			Qf<<Qf_a;
		}

//		ArrayConn2Matrix<type_f, FMPC_GT, FMPC_GM, FMPC_GN>(z,U_a,X_a);

		for(int i=0;i<FMPC_GT;i++){
			for(int j=0;j<FMPC_GM;j++){
#ifdef ARR_1D
				z.elem[i][j]=U_a[i*FMPC_GM+j];
#else
				z.elem[i][j]=U_a[i][j];
#endif
			}
			for(int j=0;j<FMPC_GN;j++){
#ifdef ARR_1D
				z.elem[i][j+FMPC_GM]=X_a[i*FMPC_GN+j];
#else
				z.elem[i][j+FMPC_GM]=X_a[i][j];
#endif
			}
		}

		for(int i=0;i<FMPC_GT;i++){
			for(int j=0;j<FMPC_GM;j++){
#ifdef ARR_1D
				zmax.elem[i][j]=umax_a[i*FMPC_GM+j];
#else
				zmax.elem[i][j]=umax_a[i][j];
#endif
			}
			for(int j=0;j<FMPC_GN;j++){
#ifdef ARR_1D
				zmax.elem[i][j+FMPC_GM]=xmax_a[i*FMPC_GN+j];
#else
				zmax.elem[i][j+FMPC_GM]=xmax_a[i][j];
#endif
			}
		}

		for(int i=0;i<FMPC_GT;i++){
			for(int j=0;j<FMPC_GM;j++){
#ifdef ARR_1D
				zmin.elem[i][j]=umin_a[i*FMPC_GM+j];
#else
				zmin.elem[i][j]=umin_a[i][j];
#endif
			}
			for(int j=0;j<FMPC_GN;j++){
#ifdef ARR_1D
				zmin.elem[i][j+FMPC_GM]=xmin_a[i*FMPC_GN+j];
#else
				zmin.elem[i][j+FMPC_GM]=xmin_a[i][j];
#endif
			}
		}

//		ArrayConn2Matrix<type_f, FMPC_GT, FMPC_GM, FMPC_GN>(zmax,umax_a,xmax_a);
//		ArrayConn2Matrix<type_f, FMPC_GT, FMPC_GM, FMPC_GN>(zmin,umin_a,xmin_a);
		x<<x_a;

		for(int i=0;i<FMPC_GT;i++){
			for(int j=0;j<FMPC_GM;j++){
#ifdef ARR_1D
				umax.elem[i][j]=umax_a[i*FMPC_GM+j];
				umin.elem[i][j]=umin_a[i*FMPC_GM+j];
#else
				umax.elem[i][j]=umax_a[i][j];
				umin.elem[i][j]=umin_a[i][j];
#endif
			}
		}

//		GetSubMatrix_A2M<type_f, FMPC_GT, FMPC_GM, 1, FMPC_GM>(umax_a,umax,0,0,0);
//		GetSubMatrix_A2M<type_f, FMPC_GT, FMPC_GM, 1, FMPC_GM>(umin_a,umin,0,0,0);

		zminp=zmin+(zmax-zmin)*(type_f)0.01;
		zmaxp=zmax-(zmax-zmin)*(type_f)0.01;

		checkBoundary(z, zminp, zmaxp);
	    fmpc_solver(A, B, Q, R, Qf, zmax, zmin, x, z, niter, kappa);


		for(int i=0;i<FMPC_GT;i++){
			for(int j=0;j<FMPC_GM;j++){
#ifdef ARR_1D
				U_ao[i*FMPC_GM+j]=z.elem[i][j];
#else
				U_ao[i][j]=z.elem[i][j];
#endif
			}
			for(int j=0;j<FMPC_GN;j++){
#ifdef ARR_1D
				X_ao[i*FMPC_GN+j]=z.elem[i][j+FMPC_GM];
#else
				X_ao[i][j]=z.elem[i][j+FMPC_GM];
#endif
			}
		}

//	    GetSubMatrix_M2A<type_f, FMPC_GT, FMPC_GM+FMPC_GN, FMPC_GT, FMPC_GM>(z, U_ao, 0, 0, 0);
//	    GetSubMatrix_M2A<type_f, FMPC_GT, FMPC_GM+FMPC_GN, FMPC_GT, FMPC_GN>(z, X_ao, 0, FMPC_GM, 0);


}
Matrix<type_f,FMPC_GT,FMPC_GN> 	bb;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> 	dz;

Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> 	Ctnu;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> 	newCtnu;
//Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> 	z;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> 	newz;

Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> 	gp;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> 	hp;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> 	gf;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> 	newgp;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> 	newhp;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> 	newgf;

Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> 	rd;
Matrix<type_f,FMPC_GT,FMPC_GN> 	rp;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> 	newrd;
Matrix<type_f,FMPC_GT,FMPC_GN> 	newrp;
void fmpc_solver(	Matrix<type_f,FMPC_GN,FMPC_GN>& A, 		Matrix<type_f,FMPC_GN,FMPC_GM>& B,
					Matrix<type_f,FMPC_GN,FMPC_GN>& Q, 		Matrix<type_f,FMPC_GM,FMPC_GM>& R, Matrix<type_f,FMPC_GN,FMPC_GN>& Qf,
					Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& zmax, Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& zmin,
					Matrix<type_f,FMPC_GN,1>& x, 		Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& z0,
					int niters, type_f kappa){
	int maxiter = niters;
	int iiter, count;
	type_f alpha = 0.01;
	type_f beta = 0.9;
	type_f tol=0.1;
	type_f s=0.0;
	type_f res=0;
	type_f resd=0, resp=0, newresd=0, newresp=0, newres=0;
	type_f temp0=0.0;

//	Matrix<type_f,FMPC_GN,FMPC_GN> 	eyen;
//	Matrix<type_f,FMPC_GM,FMPC_GM> 	eyem;
//	eyen.SetEye();
//	eyem.SetEye();
	Matrix<type_f,FMPC_GT,FMPC_GN> 	dnu;



	Matrix<type_f,FMPC_GT,FMPC_GN> 	nu;
	Matrix<type_f,FMPC_GT,FMPC_GN> 	newnu;

	//bb=A*x;
    fgemv_(	0,(type_f)1.0,
    		A,0,0,FMPC_GN,FMPC_GN,
    		x,0,0,FMPC_GN,1,
    		(type_f)0.0,
    		bb,0,0,1,FMPC_GN);
    z=z0;
    //printf("\n iteration \t step \t\t rd \t\t\t rp\n");

	for(iiter=0; iiter<maxiter; iiter++){
		gfgphp(Q,R,Qf,zmax,zmin,z,gf,gp,hp);
		rdrp(A,B,z,nu,gf,gp,bb,kappa,rd,rp,Ctnu);

		resdresp(rd,rp,&resd,&resp,&res);

		if(res < tol) break;
		dnudz(A,B,Q,R,Qf,hp,rd,rp,kappa,dnu,dz);
		s=1;
		while(1){
			count=0;
			//checking convergence, if still out of range, continue
			for(int j=0;j<FMPC_GT;j++){
				for(int i=0;i<(FMPC_GM+FMPC_GN);i++){
					temp0=(z.elem[j][i]+s*dz.elem[j][i]);
					if(temp0 >= zmax.elem[j][i] || temp0 <= zmin.elem[j][i]){
						count=1;
					}
				}
			}
			if (count==1){
				s*=beta;
				continue;
			}
			else
				break;
		}
//		/* elementary matrix multiplication: new(nu) = (nu) + s*(d(nu))*/
			newnu=nu+dnu*s;

//		/* elementary matrix multiplication: new(z) = (z) + s*(d(z))*/
			newz=z+dz*s;
		/* Insert backtracking line search*/
		while(1){
			gfgphp(Q,R,Qf,zmax,zmin,newz,newgf,newgp,newhp);
			rdrp(A,B,newz,newnu,newgf,newgp,bb,kappa,newrd,newrp,newCtnu);
			resdresp(newrd,newrp,&newresd,&newresp,&newres);
			if(newres <= (1-alpha*s)*res) break;
			s*=beta;
			/* elementary matrix multiplication: new(nu) = (nu) + s*(d(nu))*/
			newnu=nu+dnu*s;
			/* elementary matrix multiplication: new(z) = (z) + s*(d(z))*/
			newz=z+dz*s;
		}
			nu=newnu;
			z=newz;
        //printf("    %d \t\t %5.4f \t %0.5e \t\t %0.5e\n",iiter,(float)s,(float)newresd,(float)newresp);
	}

    z0=z;

	return;
}

union data_bridge{
	int i;
	type_f f;
};


template <int NROW, int NCOL>
void setData(XFmpc_solver_top *XFmpc, int offset, type_f data[NROW*NCOL]){
	data_bridge bData;
	for(int i=0;i<NROW;i++){
		for(int j=0;j<NCOL;j++){
			bData.f=data[j+i*NCOL];
			XFmpc_solver_top_WriteReg(XFmpc->Axi4lites_BaseAddress,offset+(j+i*NCOL)*0x08,bData.i);
		}
	}
}
template <int NROW, int NCOL>
void getData(XFmpc_solver_top *XFmpc, int offset, type_f data[NROW*NCOL]){
	data_bridge bData;
	for(int i=0;i<NROW;i++){
		for(int j=0;j<NCOL;j++){
			bData.i=XFmpc_solver_top_ReadReg(XFmpc->Axi4lites_BaseAddress,offset+(j+i*NCOL)*0x08);
			data[j+i*NCOL]=bData.f;
		}
	}
}
void setData(XFmpc_solver_top *XFmpc, int offset, type_f data){
	XFmpc_solver_top_WriteReg(XFmpc->Axi4lites_BaseAddress,offset,data);
}

void set_fs(XFmpc_solver_top *XFmpc, int offset, char data){
//	setData(XFmpc,offset,(int)data);
	XFmpc_solver_top_WriteReg(XFmpc->Axi4lites_BaseAddress,offset,(int)data);

}
void set_niter(XFmpc_solver_top *XFmpc, int offset, int data){
//	setData(XFmpc,offset,data);
	XFmpc_solver_top_WriteReg(XFmpc->Axi4lites_BaseAddress,offset,data);
}
void set_kappa(XFmpc_solver_top *XFmpc, int offset, type_f data){
	data_bridge bData;
	bData.f=data;
//	setData(XFmpc,offset,bData.i);
	XFmpc_solver_top_WriteReg(XFmpc->Axi4lites_BaseAddress,offset,bData.i);

}


void set_A_a(XFmpc_solver_top *XFmpc, int offset, type_f data[FMPC_GN*FMPC_GN]){
	setData<FMPC_GN,FMPC_GN>(XFmpc,offset,data);
}
void set_B_a(XFmpc_solver_top *XFmpc, int offset, type_f data[FMPC_GN*FMPC_GM]){
	setData<FMPC_GN,FMPC_GM>(XFmpc,offset,data);
}
void set_Q_a(XFmpc_solver_top *XFmpc, int offset, type_f data[FMPC_GN*FMPC_GN]){
	setData<FMPC_GN,FMPC_GN>(XFmpc,offset,data);
}
void set_R_a(XFmpc_solver_top *XFmpc, int offset, type_f data[FMPC_GM*FMPC_GM]){
	setData<FMPC_GM,FMPC_GM>(XFmpc,offset,data);
}
void set_Qf_a(XFmpc_solver_top *XFmpc, int offset, type_f data[FMPC_GN*FMPC_GN]){
	setData<FMPC_GN,FMPC_GN>(XFmpc,offset,data);
}
void set_Xmax_a(XFmpc_solver_top *XFmpc, int offset, type_f data[FMPC_GT*FMPC_GN]){
	setData<FMPC_GT,FMPC_GN>(XFmpc,offset,data);
}
void set_Xmin_a(XFmpc_solver_top *XFmpc, int offset, type_f data[FMPC_GT*FMPC_GN]){
	setData<FMPC_GT,FMPC_GN>(XFmpc,offset,data);
}
void set_Umax_a(XFmpc_solver_top *XFmpc, int offset, type_f data[FMPC_GT*FMPC_GM]){
	setData<FMPC_GT,FMPC_GM>(XFmpc,offset,data);
}
void set_Umin_a(XFmpc_solver_top *XFmpc, int offset, type_f data[FMPC_GT*FMPC_GM]){
	setData<FMPC_GT,FMPC_GM>(XFmpc,offset,data);
}
void set_X_a(XFmpc_solver_top *XFmpc, int offset, type_f data[FMPC_GT*FMPC_GN]){
	setData<FMPC_GT,FMPC_GN>(XFmpc,offset,data);
}
void set_U_a(XFmpc_solver_top *XFmpc, int offset, type_f data[FMPC_GT*FMPC_GM]){
	setData<FMPC_GT,FMPC_GM>(XFmpc,offset,data);
}
void set_x_a(XFmpc_solver_top *XFmpc, int offset, type_f data[1*FMPC_GN]){
	setData<1,FMPC_GN>(XFmpc,offset,data);
}

void get_X_ao(XFmpc_solver_top *XFmpc, int offset, type_f data[FMPC_GT*FMPC_GN]){
	getData<FMPC_GT,FMPC_GN>(XFmpc,offset,data);
}
void get_U_ao(XFmpc_solver_top *XFmpc, int offset, type_f data[FMPC_GT*FMPC_GM]){
	getData<FMPC_GT,FMPC_GM>(XFmpc,offset,data);
}

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
		){
	set_fs(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_FS_DATA,fs);
	set_niter(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_NITER_DATA,niter);
	set_kappa(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_KAPPA_DATA,kappa);

	set_A_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_A_A_0_DATA,A_a);
	set_B_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_B_A_0_DATA,B_a);
	set_Q_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_Q_A_0_DATA,Q_a);
	set_R_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_R_A_0_DATA,R_a);
	set_Qf_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_QF_A_0_DATA,Qf_a);
	set_Xmax_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_XMAX_A_0_DATA,xmax_a);
	set_Xmin_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_XMIN_A_0_DATA,xmin_a);
	set_Umax_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_UMAX_A_0_DATA,umax_a);
	set_Umax_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_UMIN_A_0_DATA,umin_a);

	set_X_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_X_A_0_DATA,X_a);
	set_U_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_U_A_0_DATA,U_a);
	set_x_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_X_A_0_R_DATA,x_a);

	usleep(5400);

	get_X_ao(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_X_AO_0_DATA,X_ao);
	get_U_ao(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_U_AO_0_DATA,U_ao);

}

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
){
        set_fs(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_FS_DATA,fs);
        set_niter(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_NITER_DATA,niter);
        set_kappa(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_KAPPA_DATA,kappa);

        set_A_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_A_A_0_DATA,A_a);
        set_B_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_B_A_0_DATA,B_a);
        set_Q_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_Q_A_0_DATA,Q_a);
        set_R_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_R_A_0_DATA,R_a);
        set_Qf_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_QF_A_0_DATA,Qf_a);
        set_Xmax_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_XMAX_A_0_DATA,xmax_a);
        set_Xmin_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_XMIN_A_0_DATA,xmin_a);
        set_Umax_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_UMAX_A_0_DATA,umax_a);
        set_Umax_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_UMIN_A_0_DATA,umin_a);

        set_X_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_X_A_0_DATA,X_a);
        set_U_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_U_A_0_DATA,U_a);
        set_x_a(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_X_A_0_R_DATA,x_a);
}

void fmpc_solver_top_acquire_result(
	XFmpc_solver_top *XFmpc,
        type_f X_ao[FMPC_GT*FMPC_GN],type_f U_ao[FMPC_GT*FMPC_GM]
){
        get_X_ao(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_X_AO_0_DATA,X_ao);
        get_U_ao(XFmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_U_AO_0_DATA,U_ao);
}
