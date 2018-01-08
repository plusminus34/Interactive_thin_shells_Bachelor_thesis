
#include "InverseDeformationSolver.h"

#include "InverseDeformationObjectiveFunction.h"







template<int NDim>
double InverseDeformationObjectiveFunction<NDim>::computeValue(const dVector& p)
{
	return(idSolver->peekOofXi(p));
}


template<int NDim>
void InverseDeformationObjectiveFunction<NDim>::addGradientTo(dVector& grad, const dVector& p)
{
	dVector dodxi;
	idSolver->computeDoDxi(dodxi);

	grad += dodxi;
}

template<int NDim>
void InverseDeformationObjectiveFunction<NDim>::setCurrentBestSolution(const dVector& p)
{
	idSolver->xi = p;
	idSolver->pushXi();
	idSolver->solveMesh(true);
}



// instantiation of 2D & 3D
// ATTENTION: other values for NDim won't work

template class InverseDeformationObjectiveFunction<2>;
template class InverseDeformationObjectiveFunction<3>;