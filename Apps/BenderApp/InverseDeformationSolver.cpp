
#include <iostream>

#include "MountedPointSpring.h"

#include "InverseDeformationSolver.h"



template<int NDim>
InverseDeformationSolver<NDim>::InverseDeformationSolver()
{
	objectiveFunction = new InverseDeformationObjectiveFunction<NDim>(this);
}


template<int NDim>
InverseDeformationSolver<NDim>::InverseDeformationSolver(BenderSimulationMesh<NDim> * femMesh,
														 GradientBasedFunctionMinimizer * minimizer)
	: femMesh(femMesh), minimizer(minimizer)
{
	objectiveFunction = new InverseDeformationObjectiveFunction<NDim>(this);
}

template<int NDim>
InverseDeformationSolver<NDim>::~InverseDeformationSolver() 
{
	delete objectiveFunction;
}


template<int NDim>
void InverseDeformationSolver<NDim>::pullXi()
{
	// find number of parameters
	int n_parameters = 0;
	for(ParameterSet const * p: parameterSets) {
		n_parameters += p->getNPar();
	}
	// copy values, set start index for each mount
	xi.resize(n_parameters);
	int i = 0;
	for(ParameterSet * p: parameterSets) {
		p->parametersStartIndex = i;
		p->writeToList(xi, i);
	}
}


template<int NDim>
void InverseDeformationSolver<NDim>::pushXi()
{
	int i = 0;
	for(ParameterSet * p: parameterSets) {
		p->setFromList(xi, i);
	}
}


template<int NDim>
double InverseDeformationSolver<NDim>::solveOptimization(double terminationResidual, 
														 int maxIterations, 
														 double lineSearchStartValue,
														 int maxLineSearchIterations)
{
	// settings for optimization algorithm 
	minimizer->solveResidual = terminationResidual;
	minimizer->maxIterations = maxIterations;
	minimizer->lineSearchStartValue = lineSearchStartValue;
	minimizer->maxLineSearchIterations = maxLineSearchIterations;

	// update list of free optimization parameters
	pullXi();

	// minimize
	double o;
	minimizer->minimize(objectiveFunction, xi, o);

	return(o);
}


template<int NDim>
void InverseDeformationSolver<NDim>::solveMesh(bool solveStatic, double dt)
{
	Eigen::initParallel();
	if (solveStatic)
	{
		femMesh->solve_statics();
	}
	else {
		femMesh->solve_dynamics(dt);
	}
}


template<int NDim>
void InverseDeformationSolver<NDim>::computeDoDxi(dVector & dodxi)
{
	dodxi.resize(xi.size());

	// compute dO/dx [length(x) x 1]
	femMesh->computeDoDx(dOdx);

	// compute dF/dxi [length(x) x xi]
	dFdxi.resize(femMesh->x.size(),xi.size());
	dFdxi.setZero();

	for(BaseEnergyUnit* pin : femMesh->pinnedNodeElements) {
		dynamic_cast<MountedPointSpring<NDim>* >(pin)->addDeltaFDeltaXi(dFdxi);
	}

	// get dF/dx  (Hessian from FEM simulation)  [length(x) x length(x)]
	SparseMatrix H(femMesh->x.size(), femMesh->x.size());
	DynamicArray<MTriplet> hessianEntries(0);

	//double regularizer_temp = dynamic_cast<FEMEnergyFunction *>(femMesh->energyFunction)->regularizer;
	femMesh->energyFunction->setToStaticsMode(0.0);
	femMesh->energyFunction->addHessianEntriesTo(hessianEntries, femMesh->x);
	femMesh->energyFunction->setToStaticsMode(0.01);

	H.setFromTriplets(hessianEntries.begin(), hessianEntries.end());

	// solve dF/dx * y = dF/dxi		(y is dx/dxi)
	Eigen::SimplicialLDLT<SparseMatrix> linearSolver;
	H *= -1.0;
	linearSolver.compute(H);
	if (linearSolver.info() != Eigen::Success) {
		std::cerr << "Eigen::SimplicialLDLT decomposition failed." << std::endl;
		exit(1);
	}
	// solve for each parameter xi
	dxdxi = linearSolver.solve(-dFdxi);


	// do/dxi = do/dx * dx/dxi
	dodxi = dOdx.transpose() * dxdxi;
}



template<int NDim>
double InverseDeformationSolver<NDim>::peekOofXi(dVector const & xi_in)
{
	// store the current state of the mesh
	dVector x_temp = femMesh->x;
	dVector v_temp = femMesh->v;
	dVector m_temp = femMesh->m;
	dVector f_ext_temp = femMesh->f_ext;
	dVector xSolver_temp = femMesh->xSolver;

	// store current parameters xi
	dVector xi_temp = xi;

	// new parameters
	xi = xi_in;
	pushXi();
	//femMesh->solve_statics();
	solveMesh(true);
	double O = femMesh->computeO();
std::cout << "    tried new xi; O was " << O << std::endl;

	// set mesh to old state
	xi = xi_temp;
	pushXi();
	femMesh->x = x_temp;
	femMesh->v = v_temp;
	femMesh->m = m_temp;
	femMesh->f_ext = f_ext_temp;
	femMesh->xSolver = xSolver_temp;

	return(O);
}


// instantiation of 2D & 3D
// ATTENTION: other values for NDim won't work

template class InverseDeformationSolver<2>;
template class InverseDeformationSolver<3>;