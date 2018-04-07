#include "SoftLocoObjectiveFunction.h"
#include "SoftLocoSolver.h"

SoftLocoObjectiveFunction::SoftLocoObjectiveFunction(SoftLocoSolver *solver){
	this->solver = solver;
}

double SoftLocoObjectiveFunction::computeValue(const dVector &yS){
	return solver->calculate_OJ(solver->unstack_Traj(yS, solver->Z));
}

void SoftLocoObjectiveFunction::addGradientTo(dVector &G, const dVector &yS) {
	// TODO: Can assumes this function is _only_ ever called @(yJ_curr, xJ_curr)
	// vector<dRowVector> GS = solver->calculate_dOdyJ(solver->yJ_curr, solver->xJ_curr);
	// --
	vector<dRowVector> GS = solver->calculate_dOdyJ(solver->unstack_Traj(yS, solver->Z), solver->xJ_curr);
	G += stack_vec_dRowVector(GS).transpose(); 
}

void SoftLocoObjectiveFunction::setCurrentBestSolution(const dVector &yS) {
	solver->yJ_curr = solver->unstack_Traj(yS, solver->Z);
	solver->xJ_curr = solver->xJ_of_yJ(solver->yJ_curr);
	// solver->projectJ();
}
