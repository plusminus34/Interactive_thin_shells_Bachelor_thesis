#pragma once

#include <math.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>
#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/FunctionConstraints.h>

/*!
	class used to model unilateral constraints of the type x > l using a C2 penalty energy f(x).
		- l is the lower limit that x needs to be greater than
		- epsilon is the value away from the limit (how much larger should x be compared to l) after which f(x) = 0
		- stiffness controls the rate at which f(x) increases if x < l
*/
class SoftUnilateralConstraint {
private:
	double a1, b1, c1, a2, b2, c2, d2, epsilon;
	double limit = 0;
public:

	SoftUnilateralConstraint(double l, double stiffness, double epsilon);

	virtual ~SoftUnilateralConstraint();

	void setLimit(double l);
	void setEpsilon(double eps);

	//comptue f(x)
	double computeValue(double x);

	//compute df/dx
	double computeDerivative(double x);

	//compute ddf/dxdx
	double computeSecondDerivative(double x);
};

class SoftUnilateralUpperConstraint {
private:
	double a1, b1, c1, a2, b2, c2, d2, epsilon;
	double limit = 0;
public:
	SoftUnilateralUpperConstraint(double l, double stiffness, double epsilon);

	virtual ~SoftUnilateralUpperConstraint();

	void setLimit(double l);
	void setEpsilon(double eps);

	//comptue f(x)
	double computeValue(double x);

	//compute df/dx
	double computeDerivative(double x);

	//compute ddf/dxdx
	double computeSecondDerivative(double x);
};

class SoftSymmetricBarrierConstraint {

public:

	SoftSymmetricBarrierConstraint(double limit, double stiffness);

	virtual ~SoftSymmetricBarrierConstraint();

	//comptue f(x)
	double computeValue(double x);

	//compute df/dx
	double computeDerivative(double x);

	//compute ddf/dxdx
	double computeSecondDerivative(double x);
	double stiffness = 1;
	double limit = 1;
};

/*!
	class used to model unilateral constraints of the type x > 0 using a C2 penalty energy f(x). f(x) goes to infinity as x goes to 0, and to 0 as x goes to epsilon.
*/
class SmoothBarrierConstraint {
private:
	double epsilon = 1;
public:
	SmoothBarrierConstraint(double epsilon);

	virtual ~SmoothBarrierConstraint();

	//comptue f(x)
	double computeValue(double x);

	//compute df/dx
	double computeDerivative(double x);

	//compute ddf/dxdx
	double computeSecondDerivative(double x);
};
