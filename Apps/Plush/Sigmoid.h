#pragma once

#include "ScalarFunction.h"
#include <PlushHelpers/helpers_star.h>

class Sigmoid : public ScalarFunction {
public:
	double bot, top, yoz;
	double sy, ty;
	double tx;

	Sigmoid(const double &, const double &, const double &); 
	virtual ~Sigmoid();

	double computePrimal(double);
	double invertPrimal(double);
	double map_Y(double);
	double map_Ypx(double);
	double unmap_y(double);
	double map_X(double);
	double unmap_x(double);

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
