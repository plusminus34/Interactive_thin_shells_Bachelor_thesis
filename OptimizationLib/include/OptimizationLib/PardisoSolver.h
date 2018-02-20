//
//  PardisoSolver.h
//
//  Created by Olga Diamanti on 07/01/15.
//  Copyright (c) 2015 Olga Diamanti. All rights reserved.
//

#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Sparse>

//extract II,JJ,SS (row,column and value vectors) from sparse matrix, Eigen version 
//Olga Diamanti's method for PARDISO
void extract_ij_from_matrix(const Eigen::SparseMatrix<double> &A,
	Eigen::VectorXi &II,
	Eigen::VectorXi &JJ,
	Eigen::VectorXd &SS);

//extract II,JJ,SS (row,column and value vectors) from sparse matrix, std::vector version
void extract_ij_from_matrix(const Eigen::SparseMatrix<double> &A,
	std::vector<int> &II,
	std::vector<int> &JJ,
	std::vector<double> &SS);

 extern "C" {
 /* PARDISO prototype. */
 void pardisoinit (void   *, int    *,   int *, int *, double *, int *);
 void pardiso     (void   *, int    *,   int *, int *,    int *, int *,
                   double *, int    *,    int *, int *,   int *, int *,
                   int *, double *, double *, int *, double *);
 void pardiso_chkmatrix  (int *, int *, double *, int *, int *, int *);
 void pardiso_chkvec     (int *, int *, double *, int *);
 void pardiso_printstats (int *, int *, double *, int *, int *, int *,
                          double *, int *);
 }

template <typename vectorTypeI, typename vectorTypeS>
 class PardisoSolver
 {
 public:
   
   PardisoSolver() ;
   ~PardisoSolver();
   
   void set_type(int _mtype, bool is_upper_half = false);
   
   void init();

   void set_pattern(const vectorTypeI &II,
                    const vectorTypeI &JJ,
                    const vectorTypeS &SS);
   void analyze_pattern();
   
   bool factorize();
   
   void solve(Eigen::VectorXd &rhs,
              Eigen::VectorXd &result);

   void update_a(const vectorTypeS &SS);

 protected:
   //vector that indicates which of the elements II,JJ input will be
   //kept and read into the matrix (for symmetric matrices, only those
   //elements II[i],JJ[i] for which II[i]<<JJ[i] will be kept)
   std::vector<int> lower_triangular_ind;

   Eigen::VectorXi ia, ja;
   std::vector<Eigen::VectorXi> iis;
   Eigen::VectorXd a;
   int numRows;

   //pardiso stuff
   /*
    1: real and structurally symmetric, supernode pivoting
    2: real and symmetric positive definite
    -2: real and symmetric indefinite, diagonal or Bunch-Kaufman pivoting
    11: real and nonsymmetric, complete supernode pivoting
    */
   int mtype;       /* Matrix Type */

   // Remember if matrix is symmetric or not, to
   // decide whether to eliminate the non-upper-
   // diagonal entries from the input II,JJ,SS
   bool is_symmetric;
   bool is_upper_half;
   
   int nrhs = 1;     /* Number of right hand sides. */
   /* Internal solver memory pointer pt, */
   /* 32-bit: int pt[64]; 64-bit: long int pt[64] */
   /* or void *pt[64] should be OK on both architectures */
   void *pt[64];
   /* Pardiso control parameters. */
   int iparm[64];
   double   dparm[64];
   int maxfct, mnum, phase, error, msglvl, solver =0;
   /* Number of processors. */
   int      num_procs;
   /* Auxiliary variables. */
   char    *var;
   int i, k;
   double ddum;          /* Double dummy */
   int idum;         /* Integer dummy. */

};

 template <typename DerivedX, typename DerivedIX>
 inline void sortrows(
	 const Eigen::DenseBase<DerivedX>& X,
	 const bool ascending,
	 Eigen::PlainObjectBase<DerivedX>& Y,
	 Eigen::PlainObjectBase<DerivedIX>& IX)
 {
	 // This is already 2x faster than matlab's builtin `sortrows`. I have tried
	 // implementing a "multiple-pass" sort on each column, but see no performance
	 // improvement.
	 using namespace std;
	 using namespace Eigen;
	 // Resize output
	 const size_t num_rows = X.rows();
	 const size_t num_cols = X.cols();
	 Y.resize(num_rows, num_cols);
	 IX.resize(num_rows, 1);
	 for (int i = 0; i < num_rows; i++)
	 {
		 IX(i) = i;
	 }
	 if (ascending) {
		 auto index_less_than = [&X, num_cols](size_t i, size_t j) {
			 for (size_t c = 0; c < num_cols; c++) {
				 if (X.coeff(i, c) < X.coeff(j, c)) return true;
				 else if (X.coeff(j, c) < X.coeff(i, c)) return false;
			 }
			 return false;
		 };
		 std::sort(
			 IX.data(),
			 IX.data() + IX.size(),
			 index_less_than
		 );
	 }
	 else {
		 auto index_greater_than = [&X, num_cols](size_t i, size_t j) {
			 for (size_t c = 0; c < num_cols; c++) {
				 if (X.coeff(i, c) > X.coeff(j, c)) return true;
				 else if (X.coeff(j, c) > X.coeff(i, c)) return false;
			 }
			 return false;
		 };
		 std::sort(
			 IX.data(),
			 IX.data() + IX.size(),
			 index_greater_than
		 );
	 }
	 for (size_t j = 0; j < num_cols; j++) {
		 for (int i = 0; i < num_rows; i++)
		 {
			 Y(i, j) = X(IX(i), j);
		 }
	 }
 }

 template <typename DerivedA, typename DerivedIA, typename DerivedIC>
 inline void unique_rows(
	 const Eigen::PlainObjectBase<DerivedA>& A,
	 Eigen::PlainObjectBase<DerivedA>& C,
	 Eigen::PlainObjectBase<DerivedIA>& IA,
	 Eigen::PlainObjectBase<DerivedIC>& IC)
 {
	 using namespace std;
	 using namespace Eigen;
	 VectorXi IM;
	 DerivedA sortA;
	 sortrows(A, true, sortA, IM);


	 const int num_rows = sortA.rows();
	 const int num_cols = sortA.cols();
	 vector<int> vIA(num_rows);
	 for (int i = 0; i < num_rows; i++)
	 {
		 vIA[i] = i;
	 }

	 auto index_equal = [&sortA, &num_cols](const size_t i, const size_t j) {
		 for (size_t c = 0; c < num_cols; c++) {
			 if (sortA.coeff(i, c) != sortA.coeff(j, c))
				 return false;
		 }
		 return true;
	 };
	 vIA.erase(
		 std::unique(
			 vIA.begin(),
			 vIA.end(),
			 index_equal
		 ), vIA.end());

	 IC.resize(A.rows(), 1);
	 {
		 int j = 0;
		 for (int i = 0; i < num_rows; i++)
		 {
			 if (sortA.row(vIA[j]) != sortA.row(i))
			 {
				 j++;
			 }
			 IC(IM(i, 0), 0) = j;
		 }
	 }
	 const int unique_rows = vIA.size();
	 C.resize(unique_rows, A.cols());
	 IA.resize(unique_rows, 1);
	 // Reindex IA according to IM
	 for (int i = 0; i < unique_rows; i++)
	 {
		 IA(i, 0) = IM(vIA[i], 0);
		 C.row(i) = A.row(IA(i, 0));
	 }
 }
