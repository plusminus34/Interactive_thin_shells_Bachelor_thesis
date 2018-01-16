

#include <vector>
#include <cmath>
#include <iostream>

#include "MathLib/P3D.h"

#include "RotationMount2D.h"



RotationMount2D::RotationMount2D(ParameterSet * parameters) 
	: Mount(parameters) 
{
	if(! dynamic_cast<Rotation2DParameters *>(parameters)) {
		std::cerr << "Error:" << __FILE__ << ":" << __LINE__ << std::endl;
		exit(3);
	}
}



P3D RotationMount2D::transformation(P3D const & x0, ParameterSet * parameters_in)
{
	Rotation2DParameters * pars = static_cast<Rotation2DParameters *>(parameters_in);

	P3D x;

	double cosa = std::cos(pars->alpha);
	double sina = std::sin(pars->alpha);

	x(0) = cosa*x0(0) - sina*x0(1) + pars->shift(0);
	x(1) = sina*x0(0) + cosa*x0(1) + pars->shift(1);
	x(2) = x0(2);

	return(x);
}



void RotationMount2D::rotate(P3D const & origin, double alpha)
{
	Rotation2DParameters * pars = static_cast<Rotation2DParameters *>(parameters);

	pars->alpha += alpha;

	double cosa = std::cos(alpha);
	double sina = std::sin(alpha);

	Matrix3x3 T_rot;
	T_rot << cosa , -sina , 0,
	         sina ,  cosa , 0,
		        0 ,     0 , 1;

	V3D shift_old = pars->shift;

    V3D shift_new = T_rot * (shift_old - origin) + origin;

	pars->shift = shift_new;
}

void RotationMount2D::shift(V3D const & delta)
{
	Rotation2DParameters * pars = static_cast<Rotation2DParameters *>(parameters);

	pars->shift += delta;
}


void RotationMount2D::dxDpar(P3D const & x0, ParameterSet * parameters_in, std::vector<V3D> & grad)
{
	Rotation2DParameters * pars = static_cast<Rotation2DParameters *>(parameters_in);

	int n_par = pars->getNPar();
	grad.resize(n_par);

	double sina = std::sin(pars->alpha);
	double cosa = std::cos(pars->alpha);

	// dx/dalpha
	grad[0] = V3D(-sina*x0[0]-cosa*x0[1],  cosa*x0[0]-sina*x0[1], 0.0);
	grad[1] = V3D(1.0,                     0.0,                   0.0);
	grad[2] = V3D(0.0,                     1.0,                   0.0);

}



void Rotation2DParameters::writeToList(dVector & par, int & cursor_idx_io)
{
	par[cursor_idx_io++] = alpha;
	for(int i = 0; i < 3; ++i) {
		par[cursor_idx_io++] = shift(i);
	}
}


void Rotation2DParameters::setFromList(dVector & par, int & cursor_idx_io)
{
	alpha = par[cursor_idx_io++];
	for(int i = 0; i < 3; ++i) {
		shift(i) = par[cursor_idx_io++];
	}
}
