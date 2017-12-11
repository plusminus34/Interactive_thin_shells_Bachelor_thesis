
#include <algorithm>


#include "MathLib/P3D.h"


#include "Mount.h"




void Mount::dxDpar(P3D const & x0, std::vector<double> const & parameters, std::vector<V3D> & grad)
{
	dxDparFD(x0, parameters, grad);
}


void Mount::dxDparFD(P3D const & x0, std::vector<double> const & parameters, std::vector<V3D> & grad)
{
	double const delta = 1e-6;
	int n_par = parameters.size();

	grad.resize(n_par);

	std::vector<double> parametersPdelta(n_par);
	std::vector<double> parametersMdelta(n_par);

	for(int i = 0; i < n_par; ++i) {
		parametersPdelta = parameters;
		parametersMdelta = parameters;
		parametersPdelta[i] += delta;
		parametersMdelta[i] -= delta;

		grad[i] = transformation(x0, parametersPdelta) - transformation(x0, parametersMdelta);
		grad[i] /= 2.0 * delta;
	}

}

P3D Mount::getTransformedX(P3D const & x0)
{
	return(transformation(x0, parameters));
}

void Mount::getDxDpar(P3D const & x0, std::vector<V3D> & grad)
{
	dxDpar(x0, parameters, grad);
}

void Mount::reset()
{
	std::fill(parameters.begin(), parameters.end(), 0);
}