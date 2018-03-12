#pragma once

/*
	if x < -e             : E = 0
	if     -e < x < e     : E = interpolating spline
	if              e < x : E = 1
*/

class SmoothestStep {
public:
	SmoothestStep(double *eps_ptr); 
	SmoothestStep(double eps); 

	double *eps_ptr;
	double eps__;

	~SmoothestStep() {}


	double g(const double &); // double computeValue(const double &);
	double g1(const double &); // double computeDerivative(const double &);
	double g2(const double &); // double computeSecondDerivative(const double &);

	void draw();

};
