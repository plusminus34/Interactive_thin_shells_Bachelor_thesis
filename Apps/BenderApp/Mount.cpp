//#include <limits>


#include "MathLib/P3D.h"


#include "Mount.h"


P3D Mount::getTransformedX(P3D const & x0) 
{
	return(transformation(x0, parameters));
}


void Mount::dxDpar(P3D const & x0, std::vector<double> const & parameters, std::vector<V3D> & DxDpar)
{
	dxDparFD(x0, parameters, DxDpar);
}


void Mount::dxDparFD(P3D const & x0, std::vector<double> const & parameters, std::vector<V3D> & DxDpar)
{
	double const delta = 1e-6;
	int n_par = parameters.size();

	dxDpar.resize(n_par);

	std::vector<double> parametersPdelta(n_par);
	std::vector<double> parametersMdelta(n_par);

	for(int i = 0; i < n_par; ++i) {
		parametersPdelta = parameters;
		parametersMdelta = parameters;
		parametersPdelta[i] += delta;
		parametersMdelta[i] -= delta;

		dxDpar[i] = transformation(x0, parametersPdelta) - transformation(x0, parametersMdelta);
		dxDpar[i] /= 2.0 * delta;
	}

}

P3D Mount::getTransformedX(P3D const & x0)
{
	transformation(x0, parameters);
}

P3D Mount::getDxDpar(P3D const & x0, std::vector<V3D> & DxDpar)
{
	dxDpar(x0, parameters, DxDpar);
}