#pragma once

#include <array>

#include "OptimizationLib/GradientBasedFunctionMinimizer.h"
#include "NodePositionObjectiveFunction.h"
#include "BenderSimulationMesh.h"
#include "Trajectory3Dplus.h"

template<int NDim>
class InverseDeformationSolver {

public:
	BenderSimulationMesh<NDim>* femMesh;

	// Optimization Parameters
	dVector xi;
	// define optimization parameters (free parameters of simulation)
	void pullXi();
	void pushXi();


	// helpers for optimization
	dVector dOdxi;
	dVector dOdx;
	std::vector<dVector> deltaFdeltaxi;
	std::vector<dVector> deltaxdeltaxi;

	// Optimization Algorithm and settings
	GradientBasedFunctionMinimizer * minimizer;
	InverseDeformationObjectiveFunction * objectiveFunction;

	int maxIterations = 10;
	double solveResidual = 1e-5;
	int maxLineSearchIterations = 15;

public:

	// optimization process
	void solveMesh(bool solveStatic, double dt = 1.0/30.0);
	void computeDoDxi(dVector & dodxi);
	double peekOofXi(dVector const & xi_in);

	// solve
	void solve(int nSteps);
};


