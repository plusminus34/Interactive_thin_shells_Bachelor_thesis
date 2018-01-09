

#include <vector>
#include <cmath>
#include <iostream>

#include "MathLib/P3D.h"

#include "RotationMount3D.h"



Matrix3x3 get_T_rot(double a, double b, double c)
{
	using namespace std;

	Matrix3x3 T_rot;
	T_rot <<  cos(a)*cos(c)-sin(a)*cos(b)*sin(c),  sin(a)*cos(c)+cos(a)*cos(b)*sin(c), sin(b)*sin(c),
		     -cos(a)*sin(c)-sin(a)*cos(b)*cos(c), -sin(a)*sin(c)+cos(a)*cos(b)*cos(c), sin(b)*cos(c),
		     sin(a)*sin(b),                       -cos(a)*sin(b),                      cos(b);

	return(T_rot);
}

Matrix3x3 get_dTda(double a, double b, double c) 
{
	using namespace std;

	Matrix3x3 dT;
	dT <<  -sin(a)*cos(c)-cos(a)*cos(b)*sin(c),  cos(a)*cos(c)-sin(a)*cos(b)*sin(c), sin(b)*sin(c),
		      sin(a)*sin(c)-cos(a)*cos(b)*cos(c), -cos(a)*sin(c)-sin(a)*cos(b)*cos(c), sin(b)*cos(c),
		      cos(a)*sin(b),                       sin(a)*sin(b),                      cos(b);

	return(dT);
}

Matrix3x3 get_dTdb(double a, double b, double c)
{
	using namespace std;

	Matrix3x3 dT;
	dT <<  cos(a)*cos(c)+sin(a)*sin(b)*sin(c),  sin(a)*cos(c)-cos(a)*sin(b)*sin(c),  cos(b)*sin(c),
		   -cos(a)*sin(c)+sin(a)*sin(b)*cos(c), -sin(a)*sin(c)-cos(a)*sin(b)*cos(c), cos(b)*cos(c),
		   sin(a)*cos(b),                       -cos(a)*cos(b),                      -sin(b);

	return(dT);
}

Matrix3x3 get_dTdc(double a, double b, double c)
{
	using namespace std;

	Matrix3x3 dT;
	dT <<  -cos(a)*sin(c)-sin(a)*cos(b)*cos(c), -sin(a)*sin(c)+cos(a)*cos(b)*cos(c), sin(b)*cos(c),
		   -cos(a)*cos(c)+sin(a)*cos(b)*sin(c), -sin(a)*cos(c)-cos(a)*cos(b)*sin(c), -sin(b)*sin(c),
		   sin(a)*sin(b),                       -cos(a)*sin(b),                      cos(b);

	return(dT);
}
/*
V3D mat_times_vec(Matrix3x3 const & T, V3D const & v) 
{
	V3D u;
	for(int i = 0; i < 3; ++i) {
		u[i] = T(i,0)*v[0] + T(i,1)*v[1] + T(i,1)*v[2];
	}
	return(u);
}
*/

P3D RotationMount3D::transformation(P3D const & x0, std::vector<double> const & parameters)
{
	Matrix3x3 T_rot = get_T_rot(parameters[0], parameters[1], parameters[2]);
	V3D shift(parameters[3], parameters[4], parameters[5]);

	//P3D x;
	//x = static_cast<V3D>(mat_times_vec(T_rot, static_cast<V3D>(x0)) + shift);
	V3D x = T_rot * static_cast<V3D>(x0) + shift;

	return(static_cast<P3D>(x));
}


void RotationMount3D::dxDpar(P3D const & x0, std::vector<double> const & parameters, std::vector<V3D> & grad)
{
	int n_par = parameters.size();
	grad.resize(n_par);

	Matrix3x3 dTda = get_dTda(parameters[0], parameters[1], parameters[2]);
	Matrix3x3 dTdb = get_dTdb(parameters[0], parameters[1], parameters[2]);
	Matrix3x3 dTdc = get_dTdc(parameters[0], parameters[1], parameters[2]);

	// dx/dalpha
	V3D dxda = dTda * static_cast<V3D>(x0);
	V3D dxdb = dTdb * static_cast<V3D>(x0);
	V3D dxdc = dTdc * static_cast<V3D>(x0);
	
	grad[0] = dxda;
	grad[1] = dxdb;
	grad[2] = dxdc;
	grad[3] = V3D(1.0,                     0.0,                   0.0);
	grad[4] = V3D(0.0,                     1.0,                   0.0);
	grad[5] = V3D(0.0,                     0.0,                   1.0);
}


void RotationMount3D::rotate(P3D const & origin, double alpha, double beta, double gamma)
{
	parameters[0] += alpha;
	parameters[1] += beta;
	parameters[2] += gamma;

	Matrix3x3 T_rot = get_T_rot(alpha, beta, gamma);

	V3D shift_old(parameters[3], parameters[4], parameters[5]);

	V3D shift_new = T_rot * (shift_old - origin) + origin;

	parameters[3] = shift_new[0];
	parameters[4] = shift_new[1];
	parameters[5] = shift_new[2];

}

void RotationMount3D::shift(V3D const & delta)
{
	parameters[3] += delta[0];
	parameters[4] += delta[1];
	parameters[5] += delta[2];
}


