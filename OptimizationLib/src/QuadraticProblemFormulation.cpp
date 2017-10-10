#include <OptimizationLib/QuadraticProblemFormulation.hpp>
#include <OptimizationLib/ooqpei_assert_macros.hpp>
#include <stdexcept>
#include <OptimizationLib/OoqpEigenInterface.hpp>

using namespace Eigen;
using namespace std;

bool leastSquaresSolve(
    Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S, Eigen::VectorXd& b,
    Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
    Eigen::SparseMatrix<double, Eigen::RowMajor>& C, Eigen::VectorXd& c,
    Eigen::SparseMatrix<double, Eigen::RowMajor>& D, Eigen::VectorXd& d,
    Eigen::VectorXd& f, Eigen::VectorXd& x)
{
  // f = (Ax-b)' S (Ax-b) + x' W x = x'A'SAx - 2x'A'Sb + b'Sb + x'Wx.
  // This means minimizing f is equivalent to minimizing: 1/2 x'Qx + c'x,
  // where Q = A'SA + W and c = -A'Sb.

  int m = A.rows();
  int n = A.cols();
  x.setZero(n);
  OOQPEI_ASSERT_EQ(range_error, b.size(), m, "Vector b has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, S.rows(), m, "Matrix S has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, W.rows(), n, "Matrix W has wrong size.");

  Eigen::SparseMatrix<double, Eigen::RowMajor> Q_temp;
  Q_temp = A.transpose() * S * A;
  Q_temp += W;

  VectorXd c_temp = -A.transpose() * (S * b);

//  return ooqpei::OoqpEigenInterface::solve(Q_temp, c_temp, C, c, D, d, f, x);

  return false;
}

bool leastSquaresSolve(
    Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S, Eigen::VectorXd& b,
    Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
    Eigen::SparseMatrix<double, Eigen::RowMajor>& D, Eigen::VectorXd& d,
    Eigen::VectorXd& f, Eigen::VectorXd& x)
{
  Eigen::SparseMatrix<double, Eigen::RowMajor> C;
  Eigen::VectorXd c;
  return leastSquaresSolve(A, S, b, W, C, c, D, d, f, x);
}


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
	Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W, Eigen::VectorXd& x)
{
	//TODO: test this. Could also just directly use a linear solver from eigen
	Eigen::SparseMatrix<double, Eigen::RowMajor> C, D;
	Eigen::VectorXd c, d, f;
	return leastSquaresSolve(A, S, b, W, C, c, D, d, f, x);
}
