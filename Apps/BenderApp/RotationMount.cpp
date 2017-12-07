

#include <vector>

#include "MathLib/P3D.h"

#include "RotationMount.h"



P3D RotationMount::transformation(P3D const & x0, std::vector<double> const & parameters)
{
	P3D x;

	double & alpha = parameters[0];
	double & shift_1 = parameters[1];
	double & shift_2 = parameters[2];

	x(0) = cos(alpha)*x0(0) - sin(alpha)*x0(1) + shift_1;
	x(1) = cos(alpha)*x0(0) + sin(alpha)*x0(1) + shift_2;
	x(2) = x0(2);

	return(x);
}