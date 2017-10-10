#pragma once

#include<Eigen/Core>
#include<Eigen/SparseCore>

  /*!
   * Finds x that minimizes f = (Ax-b)' S (Ax-b) + x' W x, such that Cx = c and d <= Dx <= f
   * @param [in] A a matrix (mxn)
   * @param [in] S a diagonal weighting matrix (mxm)
   * @param [in] b a vector (mx1)
   * @param [in] W a diagonal weighting matrix (nxn)
   * @param [in] C a (possibly null) matrix (m_cxn)
   * @param [in] c a vector (m_cx1)
   * @param [in] D a (possibly null) matrix (m_dxn)
   * @param [in] d a vector (m_dx1)
   * @param [in] f a vector (m_dx1)
   * @param [out] x a vector (nx1)
   * @return true if successful
   */
  bool leastSquaresSolve(Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S, Eigen::VectorXd& b,
                    Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    Eigen::SparseMatrix<double, Eigen::RowMajor>& C, Eigen::VectorXd& c,
                    Eigen::SparseMatrix<double, Eigen::RowMajor>& D,
                    Eigen::VectorXd& d, Eigen::VectorXd& f,
                    Eigen::VectorXd& x);

  /*!
   * Finds x that minimizes f = (Ax-b)' S (Ax-b) + x' W x, such that d <= Dx <= f
   * @param [in] A a matrix (mxn)
   * @param [in] S a diagonal weighting matrix (mxm)
   * @param [in] b a vector (mx1)
   * @param [in] W a diagonal weighting matrix (nxn)
   * @param [in] D a (possibly null) matrix (m_dxn)
   * @param [in] d a vector (m_dx1)
   * @param [in] f a vector (m_dx1)
   * @param [out] x a vector (nx1)
   * @return true if successful
   */
  bool leastSquaresSolve(Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S, Eigen::VectorXd& b,
                    Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    Eigen::SparseMatrix<double, Eigen::RowMajor>& D,
                    Eigen::VectorXd& d, Eigen::VectorXd& f,
                    Eigen::VectorXd& x);

  /*!
  * Finds x that minimizes f = (Ax-b)' S (Ax-b) + x' W x
  * @param [in] A a matrix (mxn)
  * @param [in] S a diagonal weighting matrix (mxm)
  * @param [in] b a vector (mx1)
  * @param [in] W a diagonal weighting matrix (nxn)
  * @param [out] x a vector (nx1)
  * @return true if successful
  */
  bool leastSquaresSolve(Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
	  Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S, Eigen::VectorXd& b,
	  Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
	  Eigen::VectorXd& x);
