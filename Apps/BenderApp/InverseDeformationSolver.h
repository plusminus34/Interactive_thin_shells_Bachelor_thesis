#pragma once

#include <array>

#include "OptimizationLib/GradientBasedFunctionMinimizer.h"
#include "InverseDeformationObjectiveFunction.h"
#include "BenderSimulationMesh.h"
#include "Trajectory3Dplus.h"

template<int NDim>
class InverseDeformationSolver {

public:

	BenderSimulationMesh<NDim>* femMesh;

	// Optimization Algorithm and settings
	GradientBasedFunctionMinimizer * minimizer;
	InverseDeformationObjectiveFunction<NDim> * objectiveFunction;

	// Optimization Parameters
	std::vector<ParameterSet*> parameterSets;
	dVector xi;

	// helpers for optimization
	dVector dOdxi;
	dVector dOdx;
	std::vector<dVector> deltaFdeltaxi;
	std::vector<dVector> deltaxdeltaxi;

public:

	InverseDeformationSolver();
	InverseDeformationSolver(BenderSimulationMesh<NDim> * femMesh,
							 GradientBasedFunctionMinimizer * minimizer);
	~InverseDeformationSolver();

	// define optimization parameters (free parameters of simulation)
	void pullXi();
	void pushXi();

	// optimization process
	void solveMesh(bool solveStatic, double dt = 1.0/30.0);
	void computeDoDxi(dVector & dodxi);
	double peekOofXi(dVector const & xi_in);

	// solve
	double solveOptimization(double terminationResidual, 
							 int maxIterations, 
							 double lineSearchStartValue,
							 int maxLineSearchIteration);
};


