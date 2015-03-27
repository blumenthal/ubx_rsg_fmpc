/*
 * mol.h
 *
 *  Created on: Apr 5, 2013
 *      Author: zhanglin
 *      The matrix size should be decided when the matrix object is created.
 *      This class is designed for hardware matrix computation. A vector can
 *      be defined by setting the number of rows or columns to 1.
 */

#ifndef MOL_H_
#define MOL_H_
#include "mol_config.h"
namespace MOL {
	template <typename T, int NROW, int NCOL>
	class Matrix {
	public:
		T elem[NROW][NCOL];
		//constructors
		Matrix<T, NROW, NCOL>();
		Matrix<T, NROW, NCOL>(const Matrix<T, NROW, NCOL>& arg);
		Matrix<T, NROW, NCOL>(const T elements[NROW][NCOL]);
		Matrix<T, NROW, NCOL>(const T init_val);
		//destructor
		~Matrix<T, NROW, NCOL>();

		//basic operations
		Matrix<T, NROW, NCOL>& SetOnes(void);
		Matrix<T, NROW, NCOL>& SetZeros(void);
		Matrix<T, NROW, NCOL>& SetEye(void);
		Matrix<T, NROW, NCOL>& Test(TransT Trans);


		Matrix<T, NROW, NCOL>& Transpose(const Matrix<T, NCOL, NROW>& arg);
		T Transpose(const Matrix<T, NROW, NCOL>& arg, int i, int j);

		//operator overload
		Matrix<T, NROW, NCOL>& operator= (const Matrix<T, NROW, NCOL>& arg);
		Matrix<T, NROW, NCOL>& operator<< (const T elements[NROW][NCOL]);
		Matrix<T, NROW, NCOL>& operator<< (const T elements[NROW*NCOL]);

		Matrix<T, NROW, NCOL>& operator+= (const Matrix<T, NROW, NCOL>& arg);
		Matrix<T, NROW, NCOL>& operator-= (const Matrix<T, NROW, NCOL>& arg);

		Matrix<T, NROW, NCOL>& operator+= (const T scalar);
		Matrix<T, NROW, NCOL>& operator-= (const T scalar);
		Matrix<T, NROW, NCOL>& operator*= (const T scalar);
		Matrix<T, NROW, NCOL>& operator/= (const T scalar);

		Matrix<T, NROW, NCOL>& GetArray (const T elements[NROW][NCOL]);

	};

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>::Matrix() {
		for(int i=0;i<NROW;i++)
			for(int j=0;j<NCOL;j++)
				this->elem[i][j]=0;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>::Matrix(const T elements[NROW][NCOL]) {
		for(int i=0;i<NROW;i++)
			for(int j=0;j<NCOL;j++)
				this->elem[i][j]=elements[i][j];
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>::Matrix(const Matrix<T, NROW, NCOL>& arg) {
		for(int i=0;i<NROW;i++)
			for(int j=0;j<NCOL;j++)
				this->elem[i][j]=arg.elem[i][j];
	}


	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>::Matrix(const T init_val){
		for(int i=0;i<NROW;i++)
			for(int j=0;j<NCOL;j++)
				this->elem[i][j]=init_val;
	};

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::Transpose(const Matrix<T, NCOL, NROW>& arg) {
#ifdef __SYNTHESIS__
label_transpose:;
#endif
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				this->elem[i][j]=arg.elem[j][i];
			}
		}
		return *this;
	}

	template <typename T, int NROW, int NCOL>
	T elemTranspose(const Matrix<T, NROW, NCOL>& arg, const int i,const int j,TransT trans){
		if(trans==(TransT)0)
			return arg.elem[i][j];
		else
			return arg.elem[j][i];
	}


	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>::~Matrix() {
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::SetEye(void){
#ifdef __SYNTHESIS__
label_seteye:;
#endif
#ifndef __SYNTHESIS__
		if(NROW!=NCOL) printf("[WARNING] Matrix is not square!\n");
#endif
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				if(i==j)
					this->elem[i][j] = (type_f)1.0;
				else
					this->elem[i][j] = (type_f)0.0;
			}
		}
		return *this;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::Test(TransT Trans){
		if(Trans==0){
			for(int i=0;i<NROW;i++){
				for(int j=0;j<NCOL;j++){
						this->elem[i][j] = i*NCOL+j;
				}
			}
		}
		else{
			for(int i=0;i<NCOL;i++){
				for(int j=0;j<NROW;j++){
						this->elem[j][i] = i*NROW+j;
				}
			}
		}
		return *this;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::SetZeros(void){
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
					this->elem[i][j] = 0.0;
			}
		}
		return *this;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::SetOnes(void){
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
					this->elem[i][j] = 1.0;
			}
		}
		return *this;
	}

	template <typename T, int NROW, int NCOL>
	void MatPrint(Matrix<T,NROW,NCOL> Mat, const char* matName=""){
#ifndef __SYNTHESIS__
#ifdef DEBUG
		printf("%d-by-%d matrix %s:\n", NROW, NCOL, matName);
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				if(Mat.elem[i][j]==0)
				printf("%8.0f\t",(float)Mat.elem[i][j]);
				else
				printf("%8.5f\t",(float)Mat.elem[i][j]);
			}
			printf("\n");
		}
		printf("\n");
#endif
#endif
	};

	template <typename T, int NROW, int NCOL>
	void ArrPrint(T array[NROW][NCOL], const char* matName=""){
#ifndef __SYNTHESIS__
#ifdef DEBUG
		printf("%d-by-%d Array %s:\n", NROW, NCOL, matName);
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				if(array[i][j]==0)
					printf("%8.0f\t",(float)array[i][j]);
				else
					printf("%8.5f\t",(float)array[i][j]);
			}
			printf("\n");
		}
		printf("\n");
#endif
#endif
	};

	template <typename T, int NROW, int NCOL>
	void ArrPrint(T array[NROW*NCOL], const char* matName=""){
#ifndef __SYNTHESIS__
#ifdef DEBUG
		printf("%d-by-%d Array %s:\n", NROW, NCOL, matName);
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				if(array[i*NCOL+j]==0)
					printf("%8.0f\t",array[i*NCOL+j]);
				else
					printf("%8.5f\t",array[i*NCOL+j]);
			}
			printf("\n");
		}
		printf("\n");
#endif
#endif
	};

	template <typename T, int SIZE>
	Matrix<T, SIZE, 1>& GetDiag(const Matrix<T, SIZE, SIZE>& arg){
		Matrix<T, SIZE, 1> diag;
		for(int i=0;i<SIZE;i++){
			diag[i][0]=arg.elem[i][i];
		}
	}

	template <typename T, int SIZE>
	void SetDiag(const Matrix<T, SIZE, SIZE>& arg, Matrix<T, SIZE, 1> diag){
		for(int i=0;i<SIZE;i++){
			arg.elem[i][i]=diag.elem[i][0];
		}
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NCOL, NROW>& GetTranspose(const Matrix<T, NROW, NCOL>& arg){
		Matrix<T, NCOL, NROW> res;
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				res->elem[j][i]=arg.elem[i][j];
			}
		}
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::operator=(const Matrix<T, NROW, NCOL>& arg)
	{
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				this->elem[i][j]=arg.elem[i][j];
			}
		}
		return *this;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::operator<<(const T elements[NROW][NCOL])
	{
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				this->elem[i][j]=elements[i][j];
			}
		}
		return *this;
	}
	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::operator<<(const T elements[NROW*NCOL])
	{
#ifdef __SYNTHESIS__
label_operator_assign:;
#endif

		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				this->elem[i][j]=elements[i*NCOL+j];
			}
		}
		return *this;
	}
	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL> operator+(const Matrix<T, NROW, NCOL>& lhs, T rhs){
		Matrix<T, NROW, NCOL> res;
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
			res.elem[i][j]=lhs.elem[i][j]+rhs;
			}
		}
		return res;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL> operator+( T lhs, const Matrix<T, NROW, NCOL>& rhs){
		Matrix<T, NROW, NCOL> res;
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
			res.elem[i][j]=lhs+rhs.elem[i][j];
			}
		}
		return res;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL> operator+(const Matrix<T, NROW, NCOL>& lhs, const Matrix<T, NROW, NCOL>& rhs){
		Matrix<T, NROW, NCOL> res;
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				res.elem[i][j]=lhs.elem[i][j]+rhs.elem[i][j];
			}
		}
		return res;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL> operator-(const Matrix<T, NROW, NCOL>& lhs, const Matrix<T, NROW, NCOL>& rhs){
		Matrix<T, NROW, NCOL> res;
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				res.elem[i][j]=lhs.elem[i][j]-rhs.elem[i][j];
			}
		}
		return res;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL> operator-(const Matrix<T, NROW, NCOL>& lhs, const T rhs){
		Matrix<T, NROW, NCOL> res;
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
			res.elem[i][j]=lhs.elem[i][j]-rhs;
			}
		}
		return res;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL> operator*(const Matrix<T, NROW, NCOL>& lhs, const T rhs){
		Matrix<T, NROW, NCOL> res;
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
			res.elem[i][j]=lhs.elem[i][j]*rhs;
			}
		}
		return res;
	}

	template<typename opT, int opMROW, int opMCOL, int opNROW, int opNCOL>
	Matrix<opT, opMROW, opNCOL> operator*(const Matrix<opT, opMROW, opMCOL>& lhs ,const Matrix<opT, opNROW, opNCOL>& rhs){

	        Matrix<opT, opMROW, opNCOL> res;
	        int i,j,k;
	        type_f temp;

	        for(i = 0; i < opMROW; i++){
	                for(j = 0; j < opNCOL; j++){
	                    temp = 0;
	                    for(k = 0; k < opMCOL; k++){
	                        temp += lhs.elem[i][k] * rhs.elem[k][j];
	                    }
	                    res.elem[i][j]=temp;
	                }
	            }
	        return res;
	}


	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL> operator/(const Matrix<T, NROW, NCOL>& lhs, const T rhs){
		Matrix<T, NROW, NCOL> res;
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
			res.elem[i][j]=lhs.elem[i][j]/rhs;
			}
		}
		return res;
	}

	template <typename T, int NROW, int NCOL>
	int MatDiff (const Matrix<T, NROW, NCOL>& lhs, const Matrix<T, NROW, NCOL>& rhs){
		int ret=0;
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				ret+=(int)(lhs.elem[i][j]!=rhs.elem[i][j]);
			}
		}
		return ret;
	}



	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::operator+=(const Matrix<T, NROW, NCOL>& arg)
	{
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				this->elem[i][j]+=arg.elem[i][j];
			}
		}
		return *this;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::operator-=(const Matrix<T, NROW, NCOL>& arg)
	{
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				this->elem[i][j]-=arg.elem[i][j];
			}
		}
		return *this;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::operator+=(const T scalar)
	{
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				this->elem[i][j]+=scalar;
			}
		}
		return *this;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::operator-=(const T scalar)
	{
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				this->elem[i][j]-=scalar;
			}
		}
		return *this;
	}
	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::operator*=(const T scalar)
	{
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				this->elem[i][j]*=scalar;
			}
		}
		return *this;
	}
	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::operator/=(const T scalar)
	{
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				this->elem[i][j]/=scalar;
			}
		}
		return *this;
	}

	template <typename T, int NROW, int NCOL>
	Matrix<T, NROW, NCOL>& Matrix<T, NROW, NCOL>::GetArray(const T elements[NROW][NCOL])
	{
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				this->elem[i][j]=elements[i][j];
			}
		}
		return *this;
	}

	template <typename T, int NROW, int NCOL, int AROW, int ACOL>
	void GetRowFromArray(Matrix<T, NROW, NCOL>& res, int row_res, const T elements[AROW][ACOL], int row)
	{
#ifndef __SYNTHESIS__
		if(ACOL<NCOL){
			printf("[WARNING]Array columns must be more than Matrix!\n");
			return;
		}
#endif
		for(int i=0;i<AROW;i++){
			for(int j=0;j<ACOL;j++){
				res.elem[row_res+i][j]=elements[row+i][j];
			}
		}
	}


	template<typename T, int NROW, int NCOL>
	void Matrix2array(Matrix<T, NROW, NCOL>& Mat, T array[NROW][NCOL]){
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				array[i][j]=Mat.elem[i][j];
			}
		}
	}
	template<typename T, int NROW, int NCOL>
	void Matrix2array(Matrix<T, NROW, NCOL>& Mat, T array[NROW*NCOL]){
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				array[i*NCOL+j]=Mat.elem[i][j];
			}
		}
	}
	template<typename T, int NROW, int NCOL>
	void Array2matrix(T array[NROW][NCOL], Matrix<T, NROW, NCOL>& Mat){
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				Mat.elem[i][j]=array[i][j];
			}
		}
	}
	template<typename T, int NROW, int NCOL>
	void Array2matrix(T array[NROW*NCOL], Matrix<T, NROW, NCOL>& Mat){
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				Mat.elem[i][j]=array[i*NCOL+j];
			}
		}
	}

	template<typename T, int NROW, int NCOL, int SNROW, int SNCOL>
	void GetSubMatrix(const Matrix<T, NROW, NCOL>& Mat, Matrix<T, SNROW, SNCOL>& SubMat, unsigned int X, unsigned int Y, TransT trans){
#ifndef __SYNTHESIS__
		if(trans==0){
			if(SNROW+X>NROW||SNCOL+Y>NCOL){
				printf("[WARNING]Submatrix is out of original matrix boundary.\n");
				return;
			}
		}
		else{
			if(SNROW+X>NCOL||SNCOL+Y>NROW){
				printf("[WARNING]Submatrix is out of original matrix boundary.\n");
				return;
			}
		}
#endif
		for(int i=0;i<SNROW;i++){
			for(int j=0;j<SNCOL;j++){
				if(trans==0)
				SubMat.elem[i][j]=Mat.elem[i+X][j+Y];
				else
				SubMat.elem[i][j]=Mat.elem[j+Y][i+X];

			}
		}
	}

	template<typename T, int NROW, int NCOL, int SNROW, int SNCOL>
	void GetSubMatrix(const Matrix<T, NROW, NCOL>& Mat, Matrix<T, SNROW, SNCOL>& SubMat,
						unsigned int X, unsigned int Y,
						unsigned int nXRows, unsigned int nYCols, TransT trans){
#ifndef __SYNTHESIS__
		if(trans==0){
			if(SNROW+X>NROW||SNCOL+Y>NCOL||nXRows>SNROW||nYCols>SNCOL){
				printf("[WARNING]Submatrix is out of original matrix boundary.\n");
				return;
			}
		}
		else{
			if(SNROW+X>NCOL||SNCOL+Y>NROW||nYCols>SNROW||nXRows>SNCOL){
				printf("[WARNING]Submatrix is out of original matrix boundary.\n");
				return;
			}
		}
#endif
		for(int i=0;i<SNROW;i++){
			for(int j=0;j<SNCOL;j++){
				if(trans==0)
				SubMat.elem[i][j]=Mat.elem[i+X][j+Y];
				else
				SubMat.elem[i][j]=Mat.elem[j+Y][i+X];

			}
		}
	}

	template<typename T, int NROW, int NCOL, int SNROW, int SNCOL>
	void GetSubMatrix_M2A(const Matrix<T, NROW, NCOL>& Mat, T SubMat[SNROW][SNCOL], unsigned int X, unsigned int Y, TransT trans){
#ifndef __SYNTHESIS__
		if(SNROW+X>NROW||SNCOL+Y>NCOL){
			printf("[WARNING]Submatrix is out of original matrix boundary.\n");
			return;
		}
#endif
		for(int i=0;i<SNROW;i++){
			for(int j=0;j<SNCOL;j++){
				if(trans==0)
				SubMat[i][j]=Mat.elem[i+X][j+Y];
				else
				SubMat[i][j]=Mat.elem[j+Y][i+X];
			}
		}
	}

	template<typename T, int NROW, int NCOL, int SNROW, int SNCOL>
	void GetSubMatrix_A2M(const T Mat[NROW][NCOL], Matrix<T, SNROW, SNCOL>& SubMat, unsigned int X, unsigned int Y, TransT trans){
#ifndef __SYNTHESIS__
		if(SNROW+X>NROW||SNCOL+Y>NCOL){
			printf("[WARNING]Submatrix is out of original matrix boundary. NROW:%d, NCOL:%d, SNROW:%d, SNCOL%d\n",NROW, NCOL, SNROW, SNCOL);
			return;
		}
#endif
		for(int i=0;i<SNROW;i++){
			for(int j=0;j<SNCOL;j++){
				if(trans==0)
				SubMat.elem[i][j]=Mat[i+X][j+Y];
				else
				SubMat.elem[i][j]=Mat[j+Y][i+X];

			}
		}
	}

	template<typename T, int NROW, int NCOL>
	void LoadMatrixFromFile(Matrix<T, NROW, NCOL>& Mat, const char* fn){
        printf("[INFO] Loading Matrix, size %d-by-%d, from file: %s\t\n", NROW, NCOL, fn);
        FILE *fp= fopen(fn, "r");
        float num;
        int i=0, j=0, counter=0;
        while(!feof(fp))
        {
			fscanf(fp, "%f", &num);
			Mat.elem[i][j]=num;
			counter++;
			if(j<NCOL-1)
				j++;
			else{
				j=0;
				if(i<NROW-1) i++; else {i=0;break;};
			}
        }
		printf("[INFO] %d elements were read for a matrix of <%d, %d>\n",counter, NROW, NCOL);
		if(counter!=NROW*NCOL)
			printf("[WARNING] Mismatch on Matrix size and the data file!\n\n");

   fclose(fp);
	}

	template<typename T, int NROW, int NCOL>
	void LoadArrayFromFile(T array[NROW][NCOL], const char* fn){
	        printf("[INFO] Loading Array, size %d-by-%d, from file: %s\t\n", NROW, NCOL, fn);
	        FILE *fp= fopen(fn, "r");
	        float num;
	        int i=0, j=0, counter=0;
	        while(!feof(fp))
	        {
				fscanf(fp, "%f", &num);
				array[i][j]=num;
				counter++;
				if(j<NCOL-1)
					j++;
				else{
					j=0;
					if(i<NROW-1) i++; else {i=0;break;};
				}
	        }
			printf("[INFO] %d elements were read for an array of [%d][%d]\n",counter, NROW, NCOL);
			if(counter!=NROW*NCOL)
				printf("[WARNING] Mismatch on array size and the data file!\n\n");

	   fclose(fp);
	}

	template <typename T, int NROW, int NCOL, int MCOL>
	void MatrixConn(Matrix<T, NROW, NCOL+MCOL>& res, const Matrix<T, NROW, NCOL>& lhs, const Matrix<T, NROW, MCOL>& rhs){
		for(int i=0;i<NROW;i++){
			for(int j=0;j<MCOL;j++){
				res.elem[i][j]=lhs.elem[i][j];
			}
			for(int j=0;j<NCOL;j++){
				res.elem[i][j+MCOL]=rhs.elem[i][j];
			}
		}
	}

	template <typename T, int NROW, int NCOL, int MCOL>
	void ArrayConn(T res[NROW][NCOL+MCOL], const T lhs[NROW][NCOL], const T rhs[NROW][MCOL]){
		for(int i=0;i<NROW;i++){
			for(int j=0;j<MCOL;j++){
				res[i][j]=lhs[i][j];
			}
			for(int j=0;j<NCOL;j++){
				res[i][j+MCOL]=rhs[i][j];
			}
		}
	}

	template <typename T, int NROW, int MCOL, int NCOL>
	void ArrayConn2Matrix(Matrix<T, NROW, MCOL+NCOL>& res, const T lhs[NROW][MCOL], const T rhs[NROW][NCOL]){
		for(int i=0;i<NROW;i++){
			for(int j=0;j<MCOL;j++){
				res.elem[i][j]=lhs[i][j];
			}
			for(int j=0;j<NCOL;j++){
				res.elem[i][j+MCOL]=rhs[i][j];
			}
		}
	}

	template <typename T, int NROW, int NCOL>
	void checkBoundary(Matrix<T, NROW, NCOL>& res, const Matrix<T, NROW, NCOL>& lowerLimit, const Matrix<T, NROW, NCOL>& upperLimit){
		for(int i=0;i<NROW;i++){
			for(int j=0;j<NCOL;j++){
				res.elem[i][j]=res.elem[i][j]>upperLimit.elem[i][j]?upperLimit.elem[i][j]:res.elem[i][j];
				res.elem[i][j]=res.elem[i][j]<lowerLimit.elem[i][j]?lowerLimit.elem[i][j]:res.elem[i][j];
			}
		}
	}

	template <typename T, int NROW, int NCOL, int SNROW, int SNCOL>
	void ReturnSubMatrix(const Matrix<T, NROW, NCOL>& Mat, Matrix<T, SNROW, SNCOL>& SubMat, unsigned int X, unsigned int Y,
						unsigned int nXRows, unsigned int nYCols, TransT trans){
		if(trans==0){
			if(SNROW+X>NROW||SNCOL+Y>NCOL||nXRows>SNROW||nYCols>SNCOL){
				printf("[WARNING]Submatrix is out of original matrix boundary.\n");
				return;
			}
		}
		else{
			if(SNROW+X>NCOL||SNCOL+Y>NROW||nYCols>SNROW||nXRows>SNCOL){
				printf("[WARNING]Submatrix is out of original matrix boundary.\n");
				return;
			}
		}
		for(int i=0;i<SNROW;i++){
			for(int j=0;j<SNCOL;j++){
				if(trans==0)
					SubMat.elem[i][j]=Mat.elem[i+X][j+Y];
				else
					SubMat.elem[i][j]=Mat.elem[j+Y][i+X];

			}
		}
	}
} /* namespace MOL */


#endif /* MOL_H_ */
