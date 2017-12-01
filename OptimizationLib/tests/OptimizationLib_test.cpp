#include <OptimizationLib/SoftUnilateralConstraint.h>
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

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
