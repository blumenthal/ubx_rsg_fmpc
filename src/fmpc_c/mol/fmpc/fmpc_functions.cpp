/*
 * fmpc_functions.cpp
 *
 *  Created on: Jun 24, 2014
 *      Author: zhanglin
 */

#include "fmpc_functions.hpp"

using namespace MOL;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> gp1;
Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN> gp2;
Matrix<type_f,FMPC_GT,FMPC_GN> Cz;
Matrix<type_f,FMPC_GN,FMPC_GN> PhiQ[FMPC_GT];
Matrix<type_f,FMPC_GM,FMPC_GM> PhiR[FMPC_GT];
Matrix<type_f,FMPC_GN,FMPC_GN> PhiinvQAt[FMPC_GT];
Matrix<type_f,FMPC_GM,FMPC_GN> PhiinvRBt[FMPC_GT];
Matrix<type_f,FMPC_GN,FMPC_GN> PhiinvQeye[FMPC_GT];
Matrix<type_f,FMPC_GM,FMPC_GM> PhiinvReye[FMPC_GT];
Matrix<type_f,FMPC_GN,1> CPhiinvrd[FMPC_GT];
Matrix<type_f,FMPC_GN,FMPC_GN> Yd[FMPC_GT];
Matrix<type_f,FMPC_GN,FMPC_GN> Yud[FMPC_GT-1];
Matrix<type_f,FMPC_GN,FMPC_GN> Ld[FMPC_GT];
Matrix<type_f,FMPC_GN,FMPC_GN> Lld[FMPC_GT-1];
Matrix<type_f,FMPC_GN,1> v[FMPC_GT];
Matrix<type_f,FMPC_GN,1> be[FMPC_GT];

Matrix<type_f,FMPC_GN,1> dnu[FMPC_GT];
Matrix<type_f,FMPC_GN,1> tempT;
Matrix<type_f,1,FMPC_GM> temp2;
Matrix<type_f,FMPC_GM,1> temp2T;
Matrix<type_f,1, FMPC_GM+FMPC_GN> temp4;
Matrix<type_f,FMPC_GN,FMPC_GN> tempmatn;
Matrix<type_f,1,FMPC_GM+FMPC_GN> Ctdnu[FMPC_GT];
Matrix<type_f,1,FMPC_GM+FMPC_GN> rdmCtdnu[FMPC_GT];

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
void gfgphp(Matrix<type_f,FMPC_GN,FMPC_GN>& Q,
			Matrix<type_f,FMPC_GM,FMPC_GM>& R,
			Matrix<type_f,FMPC_GN,FMPC_GN>& Qf,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& zmax,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& zmin,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& z,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& gf,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& gp,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& hp){


	int i,j;
	for(i=0;i<FMPC_GT;i++){
		for(j=0;j<(FMPC_GM+FMPC_GN);j++){
			gp1.elem[i][j]=type_f(1.)/(zmax.elem[i][j]-z.elem[i][j]);
		}
	}


	for(i=0;i<FMPC_GT;i++){
		for(j=0;j<(FMPC_GM+FMPC_GN);j++){
			gp2.elem[i][j]=type_f(1.)/(z.elem[i][j]-zmin.elem[i][j]);
		}
	}

	for(i=0;i<FMPC_GT;i++){
		for(j=0;j<(FMPC_GM+FMPC_GN);j++){
			hp.elem[i][j]=gp1.elem[i][j]*gp1.elem[i][j]+gp2.elem[i][j]*gp2.elem[i][j];
		}
	}

		gp=gp1-gp2;

	for(i=0;i<FMPC_GT-1;i++){
		fgemv_(0,(type_f)2.0,R,0,0,FMPC_GM,FMPC_GM,z,i,0,1,FMPC_GM,(type_f)0.0,gf,i,0,1,FMPC_GM);
		fgemv_(0,(type_f)2.0,Q,0,0,FMPC_GN,FMPC_GN,z,i,FMPC_GM,1,FMPC_GN,(type_f)0.0,gf,i,FMPC_GM,1,FMPC_GN);
	}

	fgemv_(0,(type_f)2.0,R,0,0,FMPC_GM,FMPC_GM,z,FMPC_GT-1,0,1,FMPC_GM,(type_f)0.0,gf,FMPC_GT-1,0,1,FMPC_GM);
	fgemv_(0,(type_f)2.0,Qf,0,0,FMPC_GN,FMPC_GN,z,FMPC_GT-1,FMPC_GM,1,FMPC_GN,(type_f)0.0,gf,FMPC_GT-1,FMPC_GM,1,FMPC_GN);

	/*TODO: z should be transposed and separated for N and M elements! */

	return;
}// end of gfgphp

/* Compute primal and dual residual, rp and rd
 * rp=Cz-b
 * rd=2Hz+g+kappa*P_transpose*d+C_transpose*nu	
 */
void rdrp(	Matrix<type_f,FMPC_GN,FMPC_GN>& A,
			Matrix<type_f,FMPC_GN,FMPC_GM>& B,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& z,
			Matrix<type_f,FMPC_GT,FMPC_GN>& nu,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& gf,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& gp,
			Matrix<type_f,FMPC_GT,FMPC_GN>& b,
			type_f kappa,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& rd,
			Matrix<type_f,FMPC_GT,FMPC_GN>& rp,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& Ctnu){
	int i,j;
	/*Compute Cz*/
	i=0;
		for(j=0;j<FMPC_GN;j++){
			Cz.elem[i][j]=z.elem[i][j+FMPC_GM];
		}
	fgemv_(0,(type_f)-1.0,B,0,0,FMPC_GN,FMPC_GM,z,0,0,1,FMPC_GM,(type_f)1.0,Cz,0,0,1,FMPC_GN);



	for(i=2;i<=FMPC_GT;i++){
		for(j=0;j<FMPC_GN;j++){
			Cz.elem[i-1][j]=z.elem[i-1][j+FMPC_GM];
		}
		fgemv_(0,(type_f)-1.0,A,0,0,FMPC_GN,FMPC_GN,z,i-2,FMPC_GM,1,FMPC_GN,(type_f)1.0,Cz,i-1,0,1,FMPC_GN);
		fgemv_(0,(type_f)-1.0,B,0,0,FMPC_GN,FMPC_GM,z,i-1,0,1,FMPC_GM,(type_f)1.0,Cz,i-1,0,1,FMPC_GN);
	}
	/*Compute Ctnu*/
	for(i=1;i<=FMPC_GT-1;i++){
		fgemv_(1,(type_f)-1.0,B,0,0,FMPC_GN,FMPC_GM,nu,i-1,0,1,FMPC_GN,(type_f)0.0,Ctnu,i-1,0,1,FMPC_GM);
		for(j=0;j<FMPC_GN;j++){
			Ctnu.elem[i-1][FMPC_GM+j]=nu.elem[i-1][j];
		}
		fgemv_(1,(type_f)-1.0,A,0,0,FMPC_GN,FMPC_GN,nu,i,0,1,FMPC_GN,(type_f)1.0,Ctnu,i-1,FMPC_GM,1,FMPC_GN);
	}
	fgemv_(1,(type_f)-1.0,B,0,0,FMPC_GN,FMPC_GM,nu,FMPC_GT-1,0,1,FMPC_GN,(type_f)0.0,Ctnu,FMPC_GT-1,0,1,FMPC_GM);


	for(i=0;i<FMPC_GN;i++){
		Ctnu.elem[FMPC_GT-1][FMPC_GM+i]=nu.elem[FMPC_GT-1][i]; //FMPC_GN element copy: Ctnu last FMPC_GN elements= nu[FMPC_GT-1] FMPC_GN elements
	}

	rp=Cz-b;
	rd=Ctnu+gf;

	faxpy_(kappa,gp,rd);

	return;
}

void resdresp(Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& rd, Matrix<type_f,FMPC_GT,FMPC_GN>& rp, type_f *resd, type_f *resp, type_f *res){
	int i,j;
	type_f resp_temp=0.0;
	type_f resd_temp=0.0;

	for(i=0;i<FMPC_GT;i++){
		for(j=0;j<FMPC_GN;j++)
			resp_temp+=rp.elem[i][j]*rp.elem[i][j];
		for(j=0;j<(FMPC_GM+FMPC_GN);j++)
			resd_temp+=rd.elem[i][j]*rd.elem[i][j];
	}

	*res =sqrt(resp_temp+resd_temp);
	*resp=sqrt(resp_temp); //could be simplified by a walkaround of sqrt for FPGA
	*resd=sqrt(resd_temp);
	return;
}

/* Solving linear equations

	| 2H+kappa*P_transpose*diag(d^2)*P	C_transpose	| |dz |     |rd|
	|							| |   |  = -|  |
	|		C			     0		| |dnu|     |rp|

*/
void dnudz(	Matrix<type_f,FMPC_GN,FMPC_GN>& A,
			Matrix<type_f,FMPC_GN,FMPC_GM>& B,
			Matrix<type_f,FMPC_GN,FMPC_GN>& Q,
			Matrix<type_f,FMPC_GM,FMPC_GM>& R,
			Matrix<type_f,FMPC_GN,FMPC_GN>& Qf,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& hp,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& rd,
			Matrix<type_f,FMPC_GT,FMPC_GN>& rp,
			type_f	kappa,
			Matrix<type_f,FMPC_GT,FMPC_GN>& dnu0,
			Matrix<type_f,FMPC_GT,FMPC_GM+FMPC_GN>& dz){
	int i,j;

	for(i=0;i<FMPC_GT;i++){
		for(j=0;j<FMPC_GN;j++){
			dnu[i].elem[j][0]=dnu0.elem[i][j];
		}
	}


	/*Form PhiQ and PhiR*/
for(i=0;i<FMPC_GT-1;i++){
		PhiQ[i]=Q*(type_f)2.0;
		for(j=0;j<FMPC_GN;j++)
		PhiQ[i].elem[j][j]+=kappa*hp.elem[i][FMPC_GM+j];
		PhiR[i]=R*(type_f)2.0;
		for(j=0;j<FMPC_GM;j++)
		PhiR[i].elem[j][j]+=kappa*hp.elem[i][j];
	}

	PhiR[FMPC_GT-1]=R*(type_f)2.0;;
	for(j=0;j<FMPC_GM;j++)
	PhiR[FMPC_GT-1].elem[j][j]+=kappa*hp.elem[FMPC_GT-1][j];

	PhiQ[FMPC_GT-1]=Qf*(type_f)2.0;
	for(j=0;j<FMPC_GN;j++)
	PhiQ[FMPC_GT-1].elem[j][j]+=kappa*hp.elem[FMPC_GT-1][FMPC_GM+j];

	/* compute PhiinvQAt, PhiinvRBt, PhiinvQeye, PhiinvReye */

   for(i=0;i<FMPC_GT;i++){
	   PhiinvQAt[i].Transpose(A);
       fpotf2_(0,PhiQ[i]);
       ftrsm_(0, 0, 1, (type_f)1.0, PhiQ[i], PhiinvQAt[i]);
       ftrsm_(0, 0, 0, (type_f)1.0, PhiQ[i], PhiinvQAt[i]);
       PhiinvQeye[i].SetEye();
       ftrsm_(0, 0, 0, (type_f)1.0, PhiQ[i], PhiinvQeye[i]);
       ftrsm_(0, 0, 1, (type_f)1.0, PhiQ[i], PhiinvQeye[i]);
   }

   for(i=0;i<FMPC_GT;i++){
	   PhiinvRBt[i].Transpose(B);
       fpotf2_(0,PhiR[i]);
       ftrsm_(0, 0, 1, (type_f)1.0, PhiR[i], PhiinvRBt[i]);
       ftrsm_(0, 0, 0, (type_f)1.0, PhiR[i], PhiinvRBt[i]);

       PhiinvReye[i].SetEye();
       ftrsm_(0, 0, 0, (type_f)1.0, PhiR[i], PhiinvReye[i]);
       ftrsm_(0, 0, 1, (type_f)1.0, PhiR[i], PhiinvReye[i]);
   }
   /* form Yd and Yud */

	for(int i=0;i<FMPC_GT-1;i++){
		Yud[i]=PhiinvQAt[i]*(type_f)-1.0;
	}
   Yd[0]=PhiinvQeye[0]+B*PhiinvRBt[0];

   for (i = 1; i < FMPC_GT; i++)
   {
	   Yd[i]=PhiinvQeye[i]+B*PhiinvRBt[i]+A*PhiinvQAt[i-1];
   }//end form Yd and Yud

   /* compute Lii */
   Ld[0]=Yd[0];
   fposv_(1,Ld[0],tempT);
   for (i = 1; i < FMPC_GT; i++)
   {
	   Lld[i-1]=Yud[i-1];
	   ftrtrs_(1,0,Ld[i-1],Lld[i-1]); //lower == 1
	   Ld[i]=Yd[i];
	   fgemm_(2,(type_f)-1.0,Lld[i-1],Lld[i-1],(type_f)1.0,Ld[i]);
	   fposv_(1,Ld[i],tempT);//lower == 1
   }//end compute Lii


   /* compute CPhiinvrd */
   for (i = 0; i < FMPC_GN; i++)
   {
	   CPhiinvrd[0].elem[i][0]=rd.elem[0][FMPC_GM+i];
   }

   ftrsm_(0,1,0,(type_f)1.0,PhiQ[0],CPhiinvrd[0]);
   ftrsm_(0,1,1,(type_f)1.0,PhiQ[0],CPhiinvrd[0]);

	GetSubMatrix(rd,temp2T,0,0,1);
	ftrsm_(0,1,0,(type_f)1.0,PhiR[0],temp2T);
	ftrsm_(0,1,1,(type_f)1.0,PhiR[0],temp2T);
	fgemv_(0,(type_f)-1.0,B,0,0,FMPC_GN,FMPC_GM,temp2T,0,0,FMPC_GM,1,(type_f)1.0,CPhiinvrd[0],0,0,FMPC_GN,1);
for (i = 1; i < FMPC_GT; i++)
   {
       for (j = 0; j < FMPC_GN; j++)
       {
    	   CPhiinvrd[i].elem[j][0]=rd.elem[i][FMPC_GM+j];
       }
       ftrsm_(0,1,0,(type_f)1.0,PhiQ[i],CPhiinvrd[i]);
       ftrsm_(0,1,1,(type_f)1.0,PhiQ[i],CPhiinvrd[i]);

       for (j = 0; j < FMPC_GM; j++)
       {
    	   temp2T.elem[j][0]=rd.elem[i][j];
       }
       GetSubMatrix(rd,temp2T,0,i,1);
       ftrsm_(0,1,0,(type_f)1.0,PhiR[i],temp2T);
       ftrsm_(0,1,1,(type_f)1.0,PhiR[i],temp2T);
       fgemv_(0,(type_f)-1.0,B,0,0,FMPC_GN,FMPC_GM,temp2T,0,0,FMPC_GM,1,(type_f)1.0,CPhiinvrd[i],0,0,FMPC_GN,1);
       for (j = 0; j < FMPC_GN; j++)
       {
    	   tempT.elem[j][0]=rd.elem[i-1][FMPC_GM+j];
       }
       GetSubMatrix(rd,tempT,FMPC_GM,i-1,1);

       ftrsm_(0,1,0,(type_f)1.0,PhiQ[i-1],tempT);
       ftrsm_(0,1,1,(type_f)1.0,PhiQ[i-1],tempT);
       fgemv_(0,(type_f)-1.0,A,0,0,FMPC_GN,FMPC_GN,tempT,0,0,FMPC_GN,1,(type_f)1.0,CPhiinvrd[i],0,0,FMPC_GN,1);

   }//end    /* compute CPhiinvrd */

       /* form be */
   	   for(j=0;j<FMPC_GT;j++)
   		   for(i=0;i<FMPC_GN;i++)
   			   be[j].elem[i][0]=CPhiinvrd[j].elem[i][0]-rp.elem[j][i];


       /* solve for dnu */
       v[0]=be[0]*(type_f)-1.0;
       ftrsm_(0,1,0,(type_f)1.0,Ld[0],v[0]);

       for (i = 1; i < FMPC_GT; i++)
       {
    	   	  v[i]=be[i];

    	   	  fgemv_(1,(type_f)-1.0,Lld[i-1],0,0,FMPC_GN,FMPC_GN,v[i-1],0,0,1,FMPC_GN,(type_f)-1.0,v[i],0,0,1,FMPC_GN);
    	   	  ftrsm_(0,1,0,(type_f)1.0,Ld[i],v[i]);
       }

       for(j=0;j<FMPC_GN;j++)
    	   dnu[FMPC_GT-1].elem[j][0]=v[FMPC_GT-1].elem[j][0];
       ftrsm_(0,1,1,(type_f)1.0,Ld[FMPC_GT-1],dnu[FMPC_GT-1]);

       for (i = FMPC_GT-1; i > 0; i--)
       {
           for(j=0;j<FMPC_GN;j++)
        	   dnu[i-1].elem[j][0]=v[i-1].elem[j][0];
    	   fgemv_(0,(type_f)-1.0,Lld[i-1],0,0,FMPC_GN,FMPC_GN,dnu[i],0,0,FMPC_GN,1,(type_f)1.0,dnu[i-1],0,0,FMPC_GN,1);
    	   ftrsm_(0,1,1,(type_f)1.0,Ld[i-1],dnu[i-1]);
       }
       /* form Ctdnu */
       for (i = 0; i < FMPC_GT-1; i++)
       {
           fgemv_(1,(type_f)-1.0,B,0,0,FMPC_GN,FMPC_GM,dnu[i],0,0,FMPC_GN,1,(type_f)0.0,Ctdnu[i],0,0,1,FMPC_GM);
           for (j = 0; j < FMPC_GN; j++)
           {
        	   Ctdnu[i].elem[0][FMPC_GM+j]=dnu[i].elem[j][0];
           }
           fgemv_(1,(type_f)-1.0,A,0,0,FMPC_GN,FMPC_GN,dnu[i+1],0,0,FMPC_GN,1,(type_f)1.0,Ctdnu[i],0,FMPC_GM,1,FMPC_GN);
       }

       fgemv_(1,(type_f)-1.0,B,0,0,FMPC_GN,FMPC_GM,dnu[FMPC_GT-1],0,0,1,FMPC_GN,(type_f)0.0,Ctdnu[FMPC_GT-1],0,0,1,FMPC_GM);

       /*update Ctdnu*/
       for (i = 0; i < FMPC_GN; i++)
       {
    	   Ctdnu[FMPC_GT-1].elem[0][FMPC_GM+i]=dnu[FMPC_GT-1].elem[i][0];
       }
       for(j=0;j<FMPC_GT;j++){
    	   GetSubMatrix(rd, temp4, j, 0, 0);
    	   rdmCtdnu[j] = Ctdnu[j]*(type_f)-1.0 - temp4;	//suspicous?
       }


       /* solve for dz */
       for (i = 0; i < FMPC_GT; i++)
       {
    	   fgemv_(0,(type_f)1.0,PhiinvQeye[i],0,0,FMPC_GN,FMPC_GN,rdmCtdnu[i],0,FMPC_GM,1,FMPC_GN,(type_f)0.0,dz,i,FMPC_GM,1,FMPC_GN);
       }
       for (i = 0; i < FMPC_GT; i++)
       {
    	   fgemv_(0,(type_f)1.0,PhiinvReye[i],0,0,FMPC_GM,FMPC_GM,rdmCtdnu[i],0,0,1,FMPC_GM,(type_f)0.0,dz,i,0,1,FMPC_GM);
       }

		for(i=0;i<FMPC_GT;i++){
			for(j=0;j<FMPC_GN;j++){
				dnu0.elem[i][j]=dnu[i].elem[j][0];
			}
		}
		return;
}
