

#include <vector>
#include <cmath>
#include <iostream>

#include "MathLib/P3D.h"

#include "RotationMount3D.h"


// euler rotation: z-y'-z'' intrinsic
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
void comput_abc_from_rot(V3D const & v, double phi, double & a, double & b, double & c)
{
	using namespace std;

	double tiny = 1.0e-10;//std::numeric_limits<double>::epsilon();

	// special case: no rotation
	if(std::fabs(phi) < tiny) {
		a = 0.0; 
		b = 0.0; 
		c = 0.0;
		return;
	}

	V3D n = v.unit();

	// special case: rotation around z-axis
	if(n[0] < tiny && n[1] < tiny) {
		a = phi;
		b = 0.0;
		c = 0.0;
		return;
	}

	// other cases
	double cphi = cos(phi);
	double sphi = sin(phi);

	// t_ij are elements of the rotation matrix T, given by the rotation of angle gamma about normal vector n
	double t_33 = n[2]*n[2] * (1.0-cphi) + cphi;
	double t_13 = n[2]*n[0] * (1.0-cphi) + n[1]*sphi;
	double t_31 = n[2]*n[0] * (1.0-cphi) - n[1]*sphi;
	double t_23 = n[2]*n[1] * (1.0-cphi) - n[0]*sphi;
	double t_32 = n[2]*n[1] * (1.0-cphi) + n[0]*sphi;
	
	double pi = 3.14159265;
//	a = atan2(-t_31, t_32);
//	c = atan2( t_13, t_23);
//	a = atan(-t_31 / t_32);
//	c = atan(t_13 / t_23);


	b = acos(t_33);

	a = asin(t_31 / sin(b));
	c = asin(t_13 / sin(b));

	
	if(sin(b) * t_31 * sin(a) >= 0.0) {
		b = b;
	}
	else {
		b = -b;
	}
	
	

std::cout << "abc = " << a << " " << b << " " << c << std::endl;
}

*/

RotationMount3D::RotationMount3D(ParameterSet * parameters) 
	: Mount(parameters) 
{
	if(! dynamic_cast<EulerRotationParameters *>(parameters)) {
		std::cerr << "Error:" << __FILE__ << ":" << __LINE__ << std::endl;
		exit(3);
	}
}

P3D RotationMount3D::transformation(P3D const & x0, ParameterSet * parameters_in)
{
	EulerRotationParameters * pars = static_cast<EulerRotationParameters * >(parameters_in);
	Matrix3x3 T_rot = get_T_rot(pars->alpha, pars->beta, pars->gamma);
	V3D shift(pars->shift);

	//P3D x;
	//x = static_cast<V3D>(mat_times_vec(T_rot, static_cast<V3D>(x0)) + shift);
	V3D x = T_rot * x0 + shift;

	return(static_cast<P3D>(x));
}


void RotationMount3D::dxDpar(P3D const & x0, ParameterSet * parameters_in, std::vector<V3D> & grad)
{
	EulerRotationParameters * pars = static_cast<EulerRotationParameters * >(parameters_in);

	int n_par = pars->getNPar();
	grad.resize(n_par);

	Matrix3x3 dTda = get_dTda(pars->alpha, pars->beta, pars->gamma);
	Matrix3x3 dTdb = get_dTdb(pars->alpha, pars->beta, pars->gamma);
	Matrix3x3 dTdc = get_dTdc(pars->alpha, pars->beta, pars->gamma);

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
	EulerRotationParameters * pars = static_cast<EulerRotationParameters * >(parameters);

	pars->alpha += alpha;
	pars->beta += beta;
	pars->gamma += gamma;

	Matrix3x3 T_rot = get_T_rot(alpha, beta, gamma);

	V3D shift_old(pars->shift);

	V3D shift_new = T_rot * (shift_old - origin) + origin;

	pars->shift = shift_new;

}

/*
void RotationMount3D::rotate(P3D const & origin, V3D const & axis, double phi)
{
	double a, b, c;

	comput_abc_from_rot(axis, phi, a, b, c);

	rotate(origin, a, b, c);
}
*/

/*
void RotationMount3D::shift(V3D const & delta)
{
	parameters[3] += delta[0];
	parameters[4] += delta[1];
	parameters[5] += delta[2];
}
*/

void RotationMount3D::shift(V3D const & delta)
{
	EulerRotationParameters * pars = static_cast<EulerRotationParameters * >(parameters);
	pars->shift += delta;
}








void EulerRotationParameters::writeToList(dVector & par, int & cursor_idx_io)
{
	par[cursor_idx_io++] = alpha;
	par[cursor_idx_io++] = beta;
	par[cursor_idx_io++] = gamma;
	for(int i = 0; i < 3; ++i) {
		par[cursor_idx_io++] = shift(i);
	}
}


void EulerRotationParameters::setFromList(dVector const & par, int & cursor_idx_io)
{
	alpha = par[cursor_idx_io++];
	beta = par[cursor_idx_io++];
	gamma = par[cursor_idx_io++];
	for(int i = 0; i < 3; ++i) {
		shift(i) = par[cursor_idx_io++];
	}
}


