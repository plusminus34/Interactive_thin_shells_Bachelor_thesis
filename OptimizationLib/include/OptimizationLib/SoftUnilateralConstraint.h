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

template<class T>
class SoftUnilateralUpperConstraintT {
private:
	T a1, b1, c1, a2, b2, c2, d2, epsilon;
	T limit = 0;
public:
	SoftUnilateralUpperConstraintT(T l, T stiffness, T epsilon){
		this->limit = l;
		this->epsilon = epsilon;
		a1 = stiffness;

		b1 = (T)0.5*a1*epsilon;
		c1 = (T)1/(T)6 * a1*epsilon*epsilon;
		a2 = (T)1/((T)2*epsilon)*a1;
		b2 = a1;
		c2 = (T)0.5*a1*epsilon;
		d2 = (T)1/(T)6*a1*epsilon*epsilon;
	}

	virtual ~SoftUnilateralUpperConstraintT()
	{

	}

	void setLimit(T l)
	{
		limit = l;
	}

	void setEpsilon(T eps)
	{
		epsilon = eps;
	}

	//comptue f(x)
	T computeValue(T x)
	{
		x = x - limit;
		if (x > 0)
			return (T)0.5 * a1 * x * x + b1 * x + c1;
		if (x > -epsilon)
			return (T)1.0 / (T)3 * a2 * x * x * x + (T)0.5 * b2 * x * x + c2 * x + d2;
		return 0;

	}
};



class SoftLowerBarrierConstraint {

public:

	SoftLowerBarrierConstraint::SoftLowerBarrierConstraint(double limit);

	virtual ~SoftLowerBarrierConstraint();

	//comptue f(x)
	double computeValue(double x);

	//compute df/dx
	double computeDerivative(double x);

	//compute ddf/dxdx
	double computeSecondDerivative(double x);
	double limit = 1;
};

class SoftSymmetricBarrierConstraint {

public:

	SoftSymmetricBarrierConstraint::SoftSymmetricBarrierConstraint(double limit);

	virtual ~SoftSymmetricBarrierConstraint();

	//comptue f(x)
	double computeValue(double x);

	//compute df/dx
	double computeDerivative(double x);

	//compute ddf/dxdx
	double computeSecondDerivative(double x);
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
