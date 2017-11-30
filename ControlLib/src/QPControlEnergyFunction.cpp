#include <ControlLib/QPControlEnergyFunction.h>

//TODO: implement objective that can handle (prevent actively) joint limits...

QPControlEnergyFunction::QPControlEnergyFunction(QPControlPlan* mp){
	theQPCPlan = mp;
	regularizer = 0.001;

	printDebugInfo = true;

	setupSubObjectives();
}

QPControlEnergyFunction::~QPControlEnergyFunction(void){
	for (uint i=0;i<objectives.size();i++)
		delete objectives[i];
}

void QPControlEnergyFunction::setupSubObjectives(){
	for (uint i=0;i<objectives.size();i++)
		delete objectives[i];
	objectives.clear();

	//set up the motion objectives...
	objectives.push_back(new QPC_GeneralizedAccelerationsObjective(theQPCPlan, "body linear acceleration targets", 100.0, 0, 3));
	objectives.push_back(new QPC_GeneralizedAccelerationsObjective(theQPCPlan, "body angular acceleration targets", 100.0, 3, 3));
	objectives.push_back(new QPC_GeneralizedAccelerationsObjective(theQPCPlan, "joint acceleration targets", 0.01, 6, theQPCPlan->robotRepresentation->getDimensionCount()-6));
	objectives.push_back(new QPC_EndEffectorListAccelerationObjective<QPControl_ContactEndEffector>(theQPCPlan, "foot sliding soft constraint", &theQPCPlan->contactEndEffectors, 10000.0));
	objectives.push_back(new QPC_EndEffectorListAccelerationObjective<QPControl_EndEffector>(theQPCPlan, "swing foot constraint", &theQPCPlan->generalEndEffectors, 100.0));
	objectives.push_back(new QPC_SwingLimbJointAccelerationsObjective(theQPCPlan, "joint accelerations as constraints objective", 10.0));
}

double QPControlEnergyFunction::computeValue(const dVector& p){
	assert(p.size() == theQPCPlan->paramCount);

	theQPCPlan->setParametersFromList(p);

	double totalEnergy = 0;

	for (uint i=0; i<objectives.size(); i++)
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
void QPControlEnergyFunction::updateRegularizingSolutionTo(const dVector &currentP){
	m_p0 = currentP;
	m_p0.setZero();
}

void QPControlEnergyFunction::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	hessianEntries.clear();
	theQPCPlan->setParametersFromList(p);

	for (uint i = 0; i < objectives.size(); ++i)
		objectives[i]->addHessianEntriesTo(hessianEntries, p);

	if (regularizer > 0) {
		for (int i = 0; i < p.size(); ++i) {
			MTriplet mt(i, i, regularizer);
			hessianEntries.push_back(mt);
		}
	}
}

void QPControlEnergyFunction::addGradientTo(dVector& grad, const dVector& p){
	assert(p.size() == theQPCPlan->paramCount);

	theQPCPlan->setParametersFromList(p);
	resize(grad, theQPCPlan->paramCount);

	//add the contribution from the regularizer
	if (regularizer > 0){
		if (m_p0.size() != p.size()) m_p0 = p;
		grad = (p - m_p0) * regularizer;
	}

	//and now the contributions of the individual objectives
	for (uint i=0; i<objectives.size(); i++)
		objectives[i]->addGradientTo(grad, p);
}


//this method gets called whenever a new best solution to the objective function is found
void QPControlEnergyFunction::setCurrentBestSolution(const dVector& p){
	if (printDebugInfo) {
		//this is applying no joint torques, just falling down due to gravity... what is the value of the objective?
		dVector tmpP; resize(tmpP, p.size());
		tmpP[theQPCPlan->generalizedAccelerationsParamsStartIndex + 1] = -9.8;

		double valDefault = computeValue(tmpP);
		double valOptimized = computeValue(p);

		if (valDefault < valOptimized) {
			Logger::consolePrint("=====> TESTING STARTING\n");
			Logger::consolePrint("=====> total cost NO ACTION: %lf\n", valDefault);
			Logger::consolePrint("=====> total cost OPTIMIZED: %lf\n", valOptimized);
			Logger::consolePrint("=====> DONE TESTING\n");
		}
	}

	updateRegularizingSolutionTo(p);
	theQPCPlan->setParametersFromList(p);

/*
	print("../out/M.m", theQPCPlan->M);
	print("../out/a.m", theQPCPlan->a);
	dVector tmp = (theQPCPlan->M * theQPCPlan->a);
	print("../out/Ma.m", tmp);
	print("../out/u.m", theQPCPlan->u);
	tmp = theQPCPlan->M * theQPCPlan->a - theQPCPlan->u;
	print("../out/Ma-u.m", tmp);
	print("../out/gf.m", theQPCPlan->gravitationalForces);
	tmp = theQPCPlan->M * theQPCPlan->a - theQPCPlan->u - theQPCPlan->gravitationalForces;
	print("../out/Ma-u-gf.m", tmp);
*/

	if (printDebugInfo){
//		print("../out/p.m", p);

		Logger::consolePrint("-------------------------------\n");
		Logger::logPrint("-------------------------------\n");

		double totalVal = computeValue(p);
		Logger::consolePrint("=====> total cost: %lf\n", totalVal);
		Logger::logPrint("=====> total cost: %lf\n", totalVal);

		for (uint i=0; i<objectives.size(); i++){
			double w = objectives[i]->weight;
			double v = objectives[i]->computeValue(p);
			Logger::logPrint("%s: %lf (weight: %lf)\n", objectives[i]->description.c_str(), v, w);
			Logger::consolePrint("%s: %lf (weight: %lf)\n", objectives[i]->description.c_str(), v, w);
			objectives[i]->printObjectiveDetails(p);
		}
	}
}


