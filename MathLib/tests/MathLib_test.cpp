#include <gtest/gtest.h>
#include <iostream>

//#include <MathLib/AutoDiff.h>
#include <Eigen/Eigen>
#include <unsupported/Eigen/AutoDiff>

// Using our own AutoDiff:
//TEST(AutoDiffTest, ScalarTest) {

//	typedef AutoDiffT<double, double> ScalarDiff;

//	// f(x) = x^a
//	// f'(x) = a*x^(a-1)
//	ScalarDiff x(2.0, 1.0);
//	ScalarDiff a = 3.0;
//	ScalarDiff f = pow(x, a);
//	EXPECT_EQ(f.value(), std::pow(x.value(), a.value()));
//	EXPECT_EQ(f.deriv(), a.value()*std::pow(x.value(), a.value()-1));
//}

//TEST(AutoDiffTest, GradientTest) {

//	typedef AutoDiffT<double, double> ADScalar;
//	typedef Eigen::Matrix<ADScalar, 2, 1> ADVector;
////	typedef Eigen::Matrix<ADScalar, Eigen::Dynamic, 1>  ADVector;
////	typedef AutoDiffT<double, double> ScalarDiffScalar;

////	int n = 2;
////	ADVector x(n);
////	x(0) = 1;
////	x(1) = 2;
////	ADVector y(n);
////	y(0) = 1;
////	y(1) = 2;

//	ADVector x;
//	x(0) = 2;
//	x(0).deriv() = 1.0;
//	x(1) = 3;
//	std::cout << "x =  " << x(0) << std::endl;
//	std::cout << "x =  " << x(1) << std::endl;

//	ADScalar f;
//	f = x(0)*x(1);


//	std::cout << "f =  " << f << std::endl;
////	std::cout << "f' = " << f.deriv() << std::endl;
//}

// adapted from Eigen AutoDiff example
TEST(AutoDiffTest, EigenAutoDiffScalarTest) {
	typedef Eigen::AutoDiffScalar<Eigen::VectorXd> AScalar;
	// AScalar stores a scalar and a derivative vector.

	// Instantiate an AutoDiffScalar variable with a normal Scalar
	double a = 3;
	double b = 4;
	AScalar Aa(a);

	// Get the value from the Instance
//	std::cout << "value: " << Aa.value() << std::endl;

	// The derivative vector
	Aa.derivatives();	// gives you a reference of
				// the contained derivative vector

	// Resize the derivative vector
	Aa.derivatives().resize(2);
	/**
	 * Important note:
	 * All ActiveScalars which are used in one computation must have
	 * either a common derivative vector length or a zero-length
	 * derivative vector.
	 */

	// Set the initial derivative vector
	Aa.derivatives() = Eigen::VectorXd::Unit(2,0);
//	std::cout << "As Derivative vector : " << As.derivatives().transpose() << std::endl;

	// Instantiate another AScalar
	AScalar Ab(b);
	Ab.derivatives() = Eigen::VectorXd::Unit(2,1);
//	std::cout << "Ab Derivative vector : " << Ab.derivatives().transpose() << std::endl;

	// Do the most simple calculation
	AScalar Ac = Aa * Ab; // c = a*b

	EXPECT_EQ(Ac.value(), a*b);
	EXPECT_EQ(Ac.derivatives()(0), b); // d(a*b)/da = b
	EXPECT_EQ(Ac.derivatives()(1), a); // d(a*b)/db = a

//	std::cout << "Result/Ac.value()" << Ac.value() << std::endl;
//	std::cout << "Gradient: " << Ac.derivatives().transpose() << std::endl;
}

template<class T>
T compute_f(const T &x) {
	return x * x.dot(x);
}

Eigen::MatrixXd jacobianOf_f(const Eigen::VectorXd &x) {
	return Eigen::MatrixXd::Identity(x.rows(), x.rows()) * x.dot(x)
			+ 2*x*x.transpose();
}

Eigen::MatrixXd jacobianOf_f_FD(const Eigen::VectorXd &x, double dx) {
	const int n = x.rows();
	Eigen::MatrixXd dfdx(n, n);
	for (int i = 0; i < n; ++i) {
		Eigen::VectorXd dxv = Eigen::VectorXd::Zero(n);
		dxv(i) = dx;
		Eigen::VectorXd xp = x+dxv;
		Eigen::VectorXd xm = x-dxv;
		Eigen::VectorXd fp = compute_f(xp);
		Eigen::VectorXd fm = compute_f(xm);
		dfdx.col(i) = (fp-fm) / (2*dx);
	}
	return dfdx;
}

template<class T>
inline void checkIsNear(const T &a, const T &b, double tol) {
	EXPECT_EQ(a.cols(), b.cols());
	EXPECT_EQ(a.rows(), b.rows());

	for (int i = 0; i < a.rows(); ++i) {
		for (int j = 0; j < b.cols(); ++j) {
			EXPECT_NEAR(a(i,j), b(i,j), tol);
		}
	}
}

TEST(AutoDiffTest, EigenAutoDiffVectorTest) {
	typedef Eigen::AutoDiffScalar<Eigen::VectorXd> AScalar;
	typedef Eigen::Matrix<AScalar, Eigen::Dynamic, 1> AVector2;

	Eigen::VectorXd x(2);
	x << 3, 4;

	const int n = x.rows();

	AVector2 xAD = x;
	for (int i = 0; i < n; ++i) {
		xAD(i).derivatives() = Eigen::VectorXd::Unit(n,i);
	}

	AVector2 f = compute_f(xAD);

	Eigen::MatrixXd dfdx(n, n);
	for (int i = 0; i < n; ++i) {
		dfdx.col(i) = f(i).derivatives();
	}

	EXPECT_EQ(f, compute_f(x));
	EXPECT_EQ(dfdx, jacobianOf_f(x));
	checkIsNear(dfdx, jacobianOf_f_FD(x, 1e-4), 1e-4);
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
