
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
	for(Mount const * m: femMesh->mounts) {
		if(m->active && m->parameterOptimization) {
			n_parameters += m->parameters.size();
		}
	}
	// copy values, set start index for each mount
	xi.resize(n_parameters);
	int i = 0;
	for(Mount * m: femMesh->mounts) {
		if(m->active && m->parameterOptimization) {
			m->parametersStartIndex = i;
			for(double p : m->parameters) {
				xi[i++] = p;
			}
		}
	}
}


template<int NDim>
void InverseDeformationSolver<NDim>::pushXi()
{
	for(Mount * m: femMesh->mounts) {
		if(m->active && m->parameterOptimization) {
			int i = 0;
			for(double & p : m->parameters) {
				p = xi[m->parametersStartIndex + (i++)];
			}
		}
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
	deltaFdeltaxi.resize(xi.size());
	for(int i = 0; i < xi.size(); ++i) {
		deltaFdeltaxi[i].resize(femMesh->x.size());
		deltaFdeltaxi[i].setZero();
	}
	for(BaseEnergyUnit* pin : femMesh->pinnedNodeElements) {
		dynamic_cast<MountedPointSpring<NDim>* >(pin)->addDeltaFDeltaXi(deltaFdeltaxi);
	}

	// get dF/dx  (Hessian from FEM simulation)  [lengh(x) x length(x)]
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
	deltaxdeltaxi.resize(xi.size());
	for(int i = 0; i < xi.size(); ++i) {
		deltaxdeltaxi[i] = linearSolver.solve(-deltaFdeltaxi[i]);
	}

	// do/dxi = do/dx * dx/dxi
	for(int i = 0; i < xi.size(); ++i) {
		dodxi[i] = dOdx.transpose() * deltaxdeltaxi[i];
	}
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
	femMesh->solve_statics();
	double O = femMesh->computeO();

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