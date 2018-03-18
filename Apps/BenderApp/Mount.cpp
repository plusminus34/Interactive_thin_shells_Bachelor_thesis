
#include <algorithm>


#include "MathLib/P3D.h"


#include "Mount.h"




void Mount::dxDpar(P3D const & x0, ParameterSet * parameters_in, std::vector<V3D> & grad)
{
	dxDparFD(x0, parameters_in, grad);
}


void Mount::dxDparFD(P3D const & x0, ParameterSet * parameters_in, std::vector<V3D> & grad)
{
	ParameterSet * pars = parameters_in;
	dVector par_vec_temp;
	// save original parameter set
	pars->pullVec(par_vec_temp);

	double const delta = 1.0e-9;
	int n_par = par_vec_temp.size();

	grad.resize(n_par);

	dVector parametersPdelta(n_par);
	dVector parametersMdelta(n_par);

	for(int i = 0; i < n_par; ++i) {
		parametersPdelta = par_vec_temp;
		parametersMdelta = par_vec_temp;
		parametersPdelta[i] += delta;
		parametersMdelta[i] -= delta;

		pars->pushVec(parametersPdelta);
		P3D trans_pdelta = transformation(x0, pars);
		pars->pushVec(parametersMdelta);
		P3D trans_mdelta = transformation(x0, pars);

		grad[i] =  trans_pdelta - trans_mdelta;
		grad[i] /= 2.0 * delta;
	}
	pars->pushVec(par_vec_temp);

}

P3D Mount::getTransformedX(P3D const & x0)
{
	return(transformation(x0, parameters));
}

void Mount::getDxDpar(P3D const & x0, std::vector<V3D> & grad)
{
	dxDpar(x0, parameters, grad);
}

/*
void Mount::reset()
{
	std::fill(parameters.begin(), parameters.end(), 0);
}
*/