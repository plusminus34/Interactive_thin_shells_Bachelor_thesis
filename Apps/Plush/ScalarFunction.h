#pragma once

class ScalarFunction {
public:
	virtual double computeValue(double x) = 0;
	virtual double computeDerivative(double x) = 0;
	virtual double computeSecondDerivative(double x) = 0;
	virtual double invertValue(double x) = 0;

};
