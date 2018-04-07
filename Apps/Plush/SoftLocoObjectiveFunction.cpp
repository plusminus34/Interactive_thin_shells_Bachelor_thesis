#include "SoftLocoObjectiveFunction.h"
#include "SoftLocoSolver.h"

SoftLocoObjectiveFunction::SoftLocoObjectiveFunction(SoftLocoSolver *solver){
	this->solver = solver;
}

double SoftLocoObjectiveFunction::computeValue(const dVector &yS){
	return solver->calculate_OJ(solver->unstack_Traj(yS, solver->Z));
}

void SoftLocoObjectiveFunction::addGradientTo(dVector &G, const dVector &yS) {
	vector<dRowVector> GS = solver->calculate_dOdyJ(solver->yJ_curr, solver->xJ_curr);
	// --
	// auto yJ = solver->unstack_Traj(yS, solver->Z);
	// vector<dRowVector> GS = solver->calculate_dOdyJ(yJ, solver->xJ_of_yJ(yJ));

	G += stack_vec_dRowVector(GS).transpose(); 
}

void SoftLocoObjectiveFunction::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	// (*) Faking gradient descent.
	hessianEntries.clear(); 
	for (int i = 0; i < p.size(); i++) {
		hessianEntries.push_back(MTriplet(i, i, 1.0));
	} 
}

void SoftLocoObjectiveFunction::setCurrentBestSolution(const dVector &yS) {
	solver->yJ_curr = solver->unstack_Traj(yS, solver->Z);
	solver->xJ_curr = solver->xJ_of_yJ(solver->yJ_curr);
	// solver->projectJ();
}
