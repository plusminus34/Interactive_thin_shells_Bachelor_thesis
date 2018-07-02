#pragma once

#include "KS_MechanicalAssembly.h"
#include "KS_AssemblyConstraintEnergy.h"
#include <MathLib/SparseMatrix.h>
#include "KS_Constraint.h"
#include <OptimizationLib/NewtonFunctionMinimizer.h>

class KS_MechanicalAssemblySimulator{
	friend class KSAssemblyOptimizer;
	friend class KS_Optimizer;
	friend class KS_GUIDesigner;
	friend class KS_ParameterizedMechanicalAssembly;
	friend class KS_ParameterizedMechanismObjectiveFunction_v2;
	friend class KS_Linkage_Optimizer;
public:
	KS_MechanicalAssemblySimulator(void);
	~KS_MechanicalAssemblySimulator(void);

	void initialize(KS_MechanicalAssembly* a, double solverResidual = 0.000001);
	//assume the driver input (or other constraints are updated elsewhere)
	double solve(double regularizer);

	void testGradientsAndHessians();

	int getNumberOfScalarConstraints();

	SparseMatrix* getCurrentConstraintJacobian();
	void getPlanarConstraintJacobianAt(Matrix &J);

	bool isInitialized(){return energyFunction != NULL;}

	int getLastSolverIterations();

	//this method computes the jacobian that tells us how the state changes with the ticker. It helps to estimate velocities of specific points on the assembly, for instance...
//	void compute_dsdTicker(dVector* assemblyState, double correspondingTickerValue, dVector* dsdTicker);
	

private:
	KS_AssemblyConstraintEnergy* energyFunction;
	NewtonFunctionMinimizer* minimizer;
	//this vector is used to store the full state of the assembly
	dVector s;

	//and a tmp matrix that holds the constraint jacobian...
//	SparseMatrix dCds, dCdst_dCds;
//	SparseLinearSolverCholMod linSolver;

};

