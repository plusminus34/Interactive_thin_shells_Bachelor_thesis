#include <OptimizationLib/SoftUnilateralConstraint.h>
#include <OptimizationLib/ObjectiveFunctionAD.h>

#include <gtest/gtest.h>
#include <iostream>

void test_constraint_with_FD(SoftUnilateralConstraint &c, const double x0, const double dx, double &resGrad, double &resHess){
	// test gradient
	double grad = c.computeDerivative(x0);
	double gradFD = (c.computeValue(x0+dx) - c.computeValue(x0-dx)) / (2*dx);
	resGrad = std::abs(grad - gradFD);

	// test hessian
	double hess = c.computeSecondDerivative(x0);
	double hessFD = (c.computeDerivative(x0+dx) - c.computeDerivative(x0-dx)) / (2*dx);
	resHess = std::abs(hess - hessFD);
}

TEST(OptimizationLibTest, SoftUnilateralConstraint_value) {
	double limit = 1.0;
	double stiffness = 10.0;
	double epsilon = 0.1;
	SoftUnilateralConstraint c(limit, stiffness, epsilon);

	EXPECT_EQ(c.computeValue(limit+epsilon), 0);
	EXPECT_NE(c.computeValue(limit-epsilon), 0);

	EXPECT_EQ(c.computeDerivative(limit+epsilon), 0);

	{
		double resGrad, resHess;
		double x0 = limit;
		test_constraint_with_FD(c, x0, 1e-8, resGrad, resHess);
		EXPECT_NEAR(resGrad, 0, 1e-5);
		EXPECT_NEAR(resHess, 0, 1e-5);
	}
	{
		double resGrad, resHess;
		double x0 = limit-epsilon;
		test_constraint_with_FD(c, x0, 1e-8, resGrad, resHess);
		EXPECT_NEAR(resGrad, 0, 1e-5);
		EXPECT_NEAR(resHess, 0, 1e-5);
	}
}

class ObjADTest : public ObjectiveFunctionAD
{
public:
	ObjADTest() {}

private:
	template<class T>
	T computeEnergy(const DOFColl<T> &dofColl) {
		const T& x = dofColl.scalars.at("x");
		return x*x;
	}

	virtual double computeEnergy(const DOFColl<double> &dofColl){
		return computeEnergy<double>(dofColl);
	}
	virtual ScalarDiff computeEnergy(const DOFColl<ScalarDiff> &dofColl){
		return computeEnergy<ScalarDiff>(dofColl);
	}
	virtual ScalarDiffDiff computeEnergy(const DOFColl<ScalarDiffDiff> &dofColl){
		return computeEnergy<ScalarDiffDiff>(dofColl);
	}

	template<class T>
	DOFColl<T> collectDOFs(const dVector &p){
		DOFColl<T> dofColl;
		dofColl.addScalar(p, 0, "x");
		return dofColl;
	}

	virtual DOFColl<double> collectDOFs(const dVector &p){
		return collectDOFs<double>(p);
	}
	virtual DOFColl<ScalarDiff> collectDOFsD(const dVector &p){
		return collectDOFs<ScalarDiff>(p);
	}
	virtual DOFColl<ScalarDiffDiff> collectDOFsDD(const dVector &p){
		return collectDOFs<ScalarDiffDiff>(p);
	}
};

TEST(OptimizationLibTest, ObjectiveFunctionAD) {

	dVector p(1);
	p[0] = 3;

	ObjADTest obj;

	double e = obj.computeValue(p);
	std::cout << "e = " << e << std::endl;

	dVector grad(1);
	obj.addGradientTo(grad, p);
	std::cout << "grad = " << grad << std::endl;

	SparseMatrix hessian(1,1);
	std::vector<MTriplet> hessianEntries;
	obj.addHessianEntriesTo(hessianEntries, p);
	hessian.setFromTriplets(hessianEntries.begin(), hessianEntries.end());
	std::cout << "hess = " << hessian << std::endl;
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
