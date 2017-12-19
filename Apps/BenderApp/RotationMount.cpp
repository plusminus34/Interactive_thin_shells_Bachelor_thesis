

#include <vector>
#include <cmath>
#include <iostream>

#include "MathLib/P3D.h"

#include "RotationMount.h"



P3D RotationMount::transformation(P3D const & x0, std::vector<double> const & parameters)
{
	P3D x;

	double const & alpha = parameters[0];
	double const & shift_1 = parameters[1];
	double const & shift_2 = parameters[2];

	double cosa = std::cos(alpha);
	double sina = std::sin(alpha);

	x(0) = cosa*x0(0) - sina*x0(1) + shift_1;
	x(1) = sina*x0(0) + cosa*x0(1) + shift_2;
	x(2) = x0(2);

	return(x);
}



void RotationMount::rotate(P3D const & origin, double alpha)
{
	parameters[0] += alpha;

	double cosa = std::cos(alpha);
	double sina = std::sin(alpha);

	Matrix3x3 T_rot;
	T_rot << cosa , -sina , 0,
	         sina ,  cosa , 0,
		        0 ,     0 , 1;

	V3D shift_old(parameters[1], parameters[2], 0);

	V3D shift_new = T_rot * (shift_old - origin) + origin;

	parameters[1] = shift_new[0];
	parameters[2] = shift_new[1];
}

void RotationMount::shift(V3D const & delta)
{
	parameters[1] += delta[0];
	parameters[2] += delta[1];
}


void RotationMount::dxDpar(P3D const & x0, std::vector<double> const & parameters, std::vector<V3D> & grad)
{
	int n_par = parameters.size();
	grad.resize(n_par);

	double sina = std::sin(parameters[0]);
	double cosa = std::cos(parameters[0]);

	// dx/dalpha
	grad[0] = V3D(-sina*x0[0]-cosa*x0[1],  cosa*x0[0]-sina*x0[1], 0.0);
	grad[1] = V3D(1.0,                     0.0,                   0.0);
	grad[2] = V3D(0.0,                     1.0,                   0.0);

std::cout << "compute dxdpar analytically" << std::endl;
}