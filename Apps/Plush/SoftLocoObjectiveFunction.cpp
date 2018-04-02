#include "SoftLocoObjectiveFunction.h"
#include "SoftLocoSolver.h"

SoftLocoObjectiveFunction::SoftLocoObjectiveFunction(SoftLocoSolver *solver){
	this->solver = solver;
}

double SoftLocoObjectiveFunction::computeValue(const dVector &uS){
	return solver->calculate_OJ(solver->unstack_Traj(uS));
}

void SoftLocoObjectiveFunction::addGradientTo(dVector &G, const dVector &_) { // FORNOW: Assumes this function is _only_ ever called @(uJ_curr, xJ_curr)
	error("[SoftLocoObjectiveFunction]: NotImplementedError");
	// TODO: Consider adding check on whether _ == uJ_curr?
	// Traj GS = solver->calculate_dOduJ(solver->uJ_curr, solver->xJ_curr);
	// G += stack_vec_dVector(GS); 
}

void SoftLocoObjectiveFunction::setCurrentBestSolution(const dVector &uS) {
	// solver->uJ_curr = solver->unstack_Traj(uS);
	// solver->xJ_curr = solver->xJ_of_uJ(solver->uJ_curr);
	// solver->projectJ();
}
