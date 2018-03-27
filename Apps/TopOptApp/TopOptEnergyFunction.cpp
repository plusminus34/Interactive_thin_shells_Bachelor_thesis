#include "TopOptEnergyFunction.h"

TopOptEnergyFunction::TopOptEnergyFunction(SimulationMesh* simMesh){
	this->simMesh = simMesh;
	printDebugInfo = false;
}

TopOptEnergyFunction::~TopOptEnergyFunction(void){

}

double TopOptEnergyFunction::computeValue(const dVector& p){
	double totalEnergy = 0;

	//a simple first test... try to maximize p...
	for (int i = 0; i < p.size(); i++)
		totalEnergy += (1 - p[i]) * (1 - p[i]);

	//add the regularizer contribution
	if (regularizer > 0){
		resize(tmpVec, p.size());
		if (m_p0.size() != p.size()) m_p0 = p;
		tmpVec = p - m_p0;
		totalEnergy += 0.5*regularizer*tmpVec.dot(tmpVec);
//		Logger::consolePrint("regularizer: %lf\n", regularizer);
	}

	return totalEnergy;
}

//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
void TopOptEnergyFunction::updateRegularizingSolutionTo(const dVector &currentP){
	m_p0 = currentP;
}


void TopOptEnergyFunction::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	hessianEntries.clear();

	//we will fake here a somewhat cautious gradient descent solver by hard coding the hessian to something nice-ish
	for (int i = 0; i < p.size(); i++)
		hessianEntries.push_back(MTriplet(i, i, 100.0));

}

/*
void TopOptEnergyFunction::addGradientTo(dVector& grad, const dVector& p){
	assert(p.size() == theMotionPlan->paramCount);

	theMotionPlan->setMPParametersFromList(p);
	resize(grad, theMotionPlan->paramCount);

	//add the contribution from the regularizer
	if (regularizer > 0){
		if (m_p0.size() != p.size()) m_p0 = p;
		grad = (p - m_p0) * regularizer;
	}

	//and now the contributions of the individual objectives
	for (uint i=0; i<objectives.size(); i++)
		if(objectives[i]->isActive && objectives[i]->weight != 0.0)
			objectives[i]->addGradientTo(grad, p);
}
*/

//this method gets called whenever a new best solution to the objective function is found
void TopOptEnergyFunction::setCurrentBestSolution(const dVector& p){
	updateRegularizingSolutionTo(p);

	if (printDebugInfo){
		Logger::consolePrint("-------------------------------\n");
		Logger::logPrint("-------------------------------\n");

		double totalVal = computeValue(p);
		Logger::consolePrint("=====> total cost: %lf\n", totalVal);
		Logger::logPrint("=====> total cost: %lf\n", totalVal);
	}
}
