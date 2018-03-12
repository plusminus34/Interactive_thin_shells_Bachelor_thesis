#pragma once

#include "ScalarFunction.h"

/*
	if x < 0             : E = 0
	if     0 < x < e     : E = some interpolating cubic
	if             e < x : E ~ c*x*x + r1(c)*x + r0(c)
*/

class Quadratic : public ScalarFunction {
public:
	double *c_ptr;

	Quadratic(double *c_ptr); 
	virtual ~Quadratic();

	double computeValue(double x); 
	double invertValue(double y); 
	double computeDerivative(double x); 
	double computeSecondDerivative(double x);

};