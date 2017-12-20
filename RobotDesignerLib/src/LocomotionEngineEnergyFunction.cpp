#include <RobotDesignerLib/LocomotionEngineEnergyFunction.h>
#include <RobotDesignerLib/MPO_TorqueAngularAccelObjective.h>
#include <omp.h>
#include <stdio.h>
#include <iostream>

LocomotionEngine_EnergyFunction::LocomotionEngine_EnergyFunction(LocomotionEngineMotionPlan* mp){
	theMotionPlan = mp;
	regularizer = 1;

	printDebugInfo = true;
}

LocomotionEngine_EnergyFunction::~LocomotionEngine_EnergyFunction(void){
	for (uint i=0;i<objectives.size();i++)
		delete objectives[i];
}

void LocomotionEngine_EnergyFunction::testIndividualGradient(dVector& params){
	DynamicArray<double> origWeights;
	for (auto obj : objectives){
		origWeights.push_back(obj->weight);
		obj->weight = 0;
	}

	for (int i = 0; i < (int)objectives.size(); i++){
		if (i > 0)
			objectives[i - 1]->weight = 0;
		objectives[i]->weight = origWeights[i];

		Logger::logPrint("\n----------- Testing objective %s -------------\n", objectives[i]->description.c_str());
		Logger::print("\n----------- Testing objective %s -------------\n", objectives[i]->description.c_str());
		testGradientWithFD(params);
	}

	for (int i = 0; i < (int)objectives.size(); i++){
		objectives[i]->weight = origWeights[i];
	}
}

void LocomotionEngine_EnergyFunction::testIndividualHessian(dVector& params){
	DynamicArray<double> origWeights;
	for (auto obj : objectives){
		origWeights.push_back(obj->weight);
		obj->weight = 0;
	}

	for (int i = 0; i < (int)objectives.size(); i++){
		if (i > 0)
			objectives[i - 1]->weight = 0;
		objectives[i]->weight = origWeights[i];

		Logger::print("\n----------- Testing objective %s -------------\n", objectives[i]->description.c_str());
		Logger::logPrint("\n----------- Testing objective %s -------------\n", objectives[i]->description.c_str());
		testHessianWithFD(params);
	}

	for (int i = 0; i < (int)objectives.size(); i++){
		objectives[i]->weight = origWeights[i];
	}
}

void LocomotionEngine_EnergyFunction::addObjectiveFunction(ObjectiveFunction *obj, string groupName)
{
	objectives.push_back(obj);
	objGroups[groupName].push_back(obj);
}

double LocomotionEngine_EnergyFunction::computeValue(const dVector& p){
	assert(p.size() == theMotionPlan->paramCount);

	theMotionPlan->setMPParametersFromList(p);

	double totalEnergy = 0;

	for (uint i=0; i<objectives.size(); i++)
		if (objectives[i]->isActive)
			totalEnergy += objectives[i]->computeValue(p);

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
void LocomotionEngine_EnergyFunction::updateRegularizingSolutionTo(const dVector &currentP){
	m_p0 = currentP;
}

void LocomotionEngine_EnergyFunction::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	hessianEntries.clear();
	theMotionPlan->setMPParametersFromList(p);

	//add the contribution from the regularizer
	if (regularizer > 0){
		for (int i = 0; i < theMotionPlan->paramCount; i++)
			hessianEntries.push_back(MTriplet(i, i, regularizer));
	}


	//and now the contributions of the individual objectives

	// Sequential version
	for (uint i = 0; i < objectives.size(); i++)
		if (objectives[i]->isActive)
			objectives[i]->addHessianEntriesTo(hessianEntries, p);

	// Parallel version
// 	#pragma omp parallel num_threads(2)
// 	{
// 		std::vector<MTriplet> hessianEntries_private;
// 		#pragma omp for schedule(dynamic) nowait //fill vec_private in parallel
// 		for (uint i = 0; i < objectives.size(); i++)
// 		{ 
// 			objectives[i]->addHessianEntriesTo(hessianEntries_private, p);
// 		}
// 		#pragma omp critical
// 		hessianEntries.insert(hessianEntries.end(), hessianEntries_private.begin(), hessianEntries_private.end());
// 	}

	/////////////////////////////////////////////
	//  Switch to this when openmp 4.0 support appears
	// 	#pragma omp declare reduction (merge : std::vector<int> : omp_out.insert(omp_out.end(), omp_in.begin(), omp_in.end()))
	// 	#pragma omp parallel for reduction(merge: vec)
	// 	for (int i = 0; i < objectives.size(); i++) 
	// 		objectives[i]->addHessianEntriesTo(hessianEntries, p);
	///////////////////////////////////////////
}

void LocomotionEngine_EnergyFunction::addGradientTo(dVector& grad, const dVector& p){
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
		if(objectives[i]->isActive)
			objectives[i]->addGradientTo(grad, p);
}

//this method gets called whenever a new best solution to the objective function is found
void LocomotionEngine_EnergyFunction::setCurrentBestSolution(const dVector& p){
	updateRegularizingSolutionTo(p);
	theMotionPlan->setMPParametersFromList(p);

	//TODO: not very nice to have this here... should maybe be an updateInternalState kind of callback...
	for (uint i = 0; i<objectives.size(); i++) {
		MPO_TorqueAngularAccelObjective* tmpObj = dynamic_cast<MPO_TorqueAngularAccelObjective*>(objectives[i]);
		if (tmpObj)
			tmpObj->updateDummyMatrices();
	}


	if (printDebugInfo){
		Logger::consolePrint("-------------------------------\n");
		Logger::logPrint("-------------------------------\n");

		double totalVal = computeValue(p);
		Logger::consolePrint("=====> total cost: %lf\n", totalVal);
		Logger::logPrint("=====> total cost: %lf\n", totalVal);

		for (uint i=0; i<objectives.size(); i++){
			double w = objectives[i]->weight;
			double v = objectives[i]->computeValue(p);
			Logger::logPrint("%s: %lf (weight: %lf)\n", objectives[i]->description.c_str(), v, w);
		}
	}
}


