#include "SoftLocoObjectiveFunction.h"
#include "SoftLocoSolver.h"

SoftLocoObjectiveFunction::SoftLocoObjectiveFunction(SoftLocoSolver *solver){
	this->solver = solver;
}

double SoftLocoObjectiveFunction::computeValue(const dVector &yS){
	return solver->calculate_OJ(solver->unstack_Traj(yS, solver->Z));
}

void SoftLocoObjectiveFunction::addGradientTo(dVector &G, const dVector &ymS) {
	vector<dRowVector> GS = solver->calculate_dOdymJ(solver->ymJ_curr, solver->xJ_curr);
	// --
	// auto ymJ = solver->unstack_Traj(ymS, solver->Z);
	// vector<dRowVector> GS = solver->calculate_dOdymJ(ymJ, solver->xJ_of_ymJ(ymJ)); 

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
	solver->ymJ_curr = solver->unstack_Traj(yS, solver->Z);
	solver->xJ_curr = solver->xJ_of_ymJ(solver->ymJ_curr);
	// solver->projectJ();
}
