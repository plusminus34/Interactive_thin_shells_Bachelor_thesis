
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
	// collision avoidance
	for(ObjectiveFunction * ca : collisionAvoidance) {
		o += ca->computeValue(p);
	}

	// regularizer
	o += parameterValueRegularizer.computeValue(p);
	o += parameterStepSizeRegularizer.computeValue(p);

if(std::isnan(o) )
{
	std::cout << "is nan " << __FILE__ << ":" << __LINE__ << std::endl;
}

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
	// collision avoidance
	for(ObjectiveFunction * ca : collisionAvoidance) {
		ca->addGradientTo(grad, p);
	}


	dVector grad_no_reg = grad;

	// regularizer
	parameterValueRegularizer.addGradientTo(grad, p);
	parameterStepSizeRegularizer.addGradientTo(grad, p);

	dVector diff_grad = grad - grad_no_reg;

std::cout << "grad without reg: ";
for(int i = 0; i < grad.size(); ++i) {std::cout << grad[i] << " ";};
std::cout << std::endl;


std::cout << "grad with reg:" << std::endl;
for(int i = 0; i < grad.size(); ++i) {std::cout << grad[i] << " ";};
std::cout << std::endl;

std::cout << "grad from reg:" << std::endl;
for(int i = 0; i < diff_grad.size(); ++i) {std::cout << diff_grad[i] << " ";};
std::cout << std::endl;


}



template<int NDim>
void InverseDeformationObjectiveFunction<NDim>::setCurrentBestSolution(const dVector& p)
{
	idSolver->xi = p;
	parameterStepSizeRegularizer.setReferenceState(p);
	idSolver->pushXi();
	idSolver->solveMesh(true);
}

template<int NDim>
void InverseDeformationObjectiveFunction<NDim>::setReferenceStateP()
{
	parameterValueRegularizer.setReferenceState(idSolver->xi);
	parameterStepSizeRegularizer.setReferenceState(idSolver->xi);
}



//template<int NDim>
//void InverseDeformationObjectiveFunction<NDim>::updateRegularizingSolutionTo(const dVector &p0_new)
//{
//	p0_reg = p0_new;
//};
//
//template<int NDim>
//void InverseDeformationObjectiveFunction<NDim>::setRegularizerValue(double r) {
//	if(r > 0.0 && p0_reg.size() > 0) {
//		use_regularizer = true;
//		regularizer = r;
//	}
//	else {
//		use_regularizer = false;
//		regularizer = 0.0;
//	}
//}
//
//template<int NDim>
//void InverseDeformationObjectiveFunction<NDim>::setRegularizer(double r, const dVector& p0)
//{
//	if(r > 0.0) {
//		use_regularizer = true;
//		regularizer = r;
//		p0_reg = p0;
//	}
//	else {
//		use_regularizer = false;
//		regularizer = 0.0;
//	}
//}
//
//template<int NDim>
//void InverseDeformationObjectiveFunction<NDim>::unsetRegularizer() 
//{
//	use_regularizer = false;
//	regularizer = 0.0;
//}




//template<int NDim>
//void setReferenceState<NDim>(dVector x)
//{
//	xRef = x;
//}




template<int NDim>
void ParameterValueRegularizer<NDim>::setReferenceState(dVector const & p)
{
	pRef = p;
}


template<int NDim>
double ParameterValueRegularizer<NDim>::computeValue(const dVector& p)
{
	if(r <= 0 || pRef.size() != p.size()) {return 0.0;}

	double e = 0.5 * r * (p - pRef).squaredNorm();

	return(e);
}


template<int NDim>
void ParameterValueRegularizer<NDim>::addGradientTo(dVector& grad, const dVector& p)
{
	if(r <= 0 || pRef.size() != p.size()) {return;}

	grad += r * (p - pRef);
}



// instantiation of 2D & 3D
// ATTENTION: other values for NDim won't work

template class InverseDeformationObjectiveFunction<2>;
template class InverseDeformationObjectiveFunction<3>;

//template class MeshEnergyRegularizer<2>;
//template class MeshEnergyRegularizer<3>;
//
//template class MeshEnergyRegularizer<2>;
//template class MeshEnergyRegularizer<3>;