
#include "InverseDeformationSolver.h"

#include "InverseDeformationObjectiveFunction.h"

#include <iostream>






template<int NDim>
double InverseDeformationObjectiveFunction<NDim>::computeValue(const dVector& p)
{
	// energy of mesh objectives
	double o = idSolver->peekOofXi(p);
	// parameter constraints
	for(ObjectiveFunction * pc : parameterConstraints) {
		o += pc->computeValue(p);
	}

	// regularizer
	dVector deltap = p - p0_reg;
	o += 0.5*regularizer*deltap.dot(deltap);

	return(o);
}


template<int NDim>
void InverseDeformationObjectiveFunction<NDim>::addGradientTo(dVector& grad, const dVector& p)
{
	// actual gradient
	dVector dodxi;
	idSolver->computeDoDxi(dodxi);
	grad += dodxi;
	// parameter constraints
	for(ObjectiveFunction * pc : parameterConstraints) {
		pc->addGradientTo(grad, p);
	}


std::cout << "grad without reg: ";
for(int i = 0; i < grad.size(); ++i) {std::cout << grad[i] << " ";};
std::cout << std::endl;
	// regularizer
	grad += regularizer*(p - p0_reg);
std::cout << "grad with reg: (val = " << regularizer << ")" << std::endl;
for(int i = 0; i < grad.size(); ++i) {std::cout << grad[i] << " ";};
std::cout << std::endl;
}

template<int NDim>
void InverseDeformationObjectiveFunction<NDim>::setCurrentBestSolution(const dVector& p)
{
	idSolver->xi = p;
	updateRegularizingSolutionTo(p);
	idSolver->pushXi();
	idSolver->solveMesh(true);
}

template<int NDim>
void InverseDeformationObjectiveFunction<NDim>::updateRegularizingSolutionTo(const dVector &p0_new)
{
	p0_reg = p0_new;
};

template<int NDim>
void InverseDeformationObjectiveFunction<NDim>::setRegularizerValue(double r) {
	if(r > 0.0 && p0_reg.size() > 0) {
		use_regularizer = true;
		regularizer = r;
	}
	else {
		use_regularizer = false;
		regularizer = 0.0;
	}
}

template<int NDim>
void InverseDeformationObjectiveFunction<NDim>::setRegularizer(double r, const dVector& p0)
{
	if(r > 0.0) {
		use_regularizer = true;
		regularizer = r;
		p0_reg = p0;
	}
	else {
		use_regularizer = false;
		regularizer = 0.0;
	}
}

template<int NDim>
void InverseDeformationObjectiveFunction<NDim>::unsetRegularizer() 
{
	use_regularizer = false;
	regularizer = 0.0;
}



// instantiation of 2D & 3D
// ATTENTION: other values for NDim won't work

template class InverseDeformationObjectiveFunction<2>;
template class InverseDeformationObjectiveFunction<3>;