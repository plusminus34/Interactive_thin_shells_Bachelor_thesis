#pragma once

#include "ScalarFunction.h"
#include <PlushHelpers/helpers_star.h>

/*
	if x < 0             : E = 0
	if     0 < x < e     : E = some interpolating cubic
	if             e < x : E ~ c*x*x + r1(c)*x + r0(c)
*/

class ZeroCubicQuadratic : public ScalarFunction {
public:
	double *c_ptr, *eps_ptr;
	double tx; double ty;
	double fx, fy;

	ZeroCubicQuadratic(double *c_ptr, double *eps_ptr, V3D t, bool, bool);
	ZeroCubicQuadratic(const double &c, const double &eps, V3D t, bool, bool);
	double c_dummy;
	double eps_dummy;

	virtual ~ZeroCubicQuadratic();

	void build_locals(double &eps, double &c, double &r1, double &r0, double &m3);

	//compute \int_0^x f(t) dt
	double integrateValue(double x);

	//comptue f(x)
	double computeValue(double x);

	//compute df/dx
	double computeDerivative(double x);

	//compute ddf/dxdx
	double computeSecondDerivative(double x);

	//compute dddf/dxdxdx
	double computeThirdDerivative(double x);

	//invert f(x) for x
	double invertValue(double y);

	//invert dfdx(x) for x
	double invertDerivative(double y);

	//debugging
	void report_parameters();
	void plot_self();
	void test_self();
};
