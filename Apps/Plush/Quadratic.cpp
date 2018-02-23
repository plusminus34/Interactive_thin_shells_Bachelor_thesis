#include "Quadratic.h"
#include <PlushHelpers/helpers_star.h>

Quadratic::Quadratic(double *c) {
	this->c_ptr = c;
}

Quadratic::~Quadratic() {
}

double Quadratic::computeValue(double x) {
	double c = *c_ptr;
	return .5*c*x*x;
}

double Quadratic::invertValue(double y) {
	double c = *c_ptr;
	return sqrt(y/(.5*c));
}

double Quadratic::computeDerivative(double x) {
	double c = *c_ptr;
	return c*x;
}

double Quadratic::computeSecondDerivative(double x) {
	double c = *c_ptr;
	return c;
}
 