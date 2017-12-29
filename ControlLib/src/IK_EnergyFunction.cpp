#include <ControlLib/IK_EnergyFunction.h>
#include <ControlLib/IK_EndEffectorsObjective.h>
#include <ControlLib/IK_RobotStateRegularizer.h>

IK_EnergyFunction::IK_EnergyFunction(IK_Plan* ikp){
	IKPlan = ikp;
	regularizer = 10;

	printDebugInfo = false;

	setupSubObjectives();
}

IK_EnergyFunction::~IK_EnergyFunction(void){
	for (uint i=0;i<objectives.size();i++)
		delete objectives[i];
}

void IK_EnergyFunction::setupSubObjectives(){
	for (uint i=0;i<objectives.size();i++)
		delete objectives[i];

	objectives.clear();

	objectives.push_back(new IK_EndEffectorsObjective(IKPlan, "end effector targets", 1000));
	objectives.push_back(new IK_RobotStateRegularizer(IKPlan, 0, 5, "root state regularizer", 100000));
	objectives.push_back(new IK_RobotStateRegularizer(IKPlan, 6, IKPlan->gcRobotRepresentation->getDimensionCount() - 1, "joint angles regularizer", 1 ));
}

void IK_EnergyFunction::setupSubObjectives_EEMatch() {
	for (uint i = 0; i<objectives.size(); i++)
		delete objectives[i];

	objectives.clear();

	objectives.push_back(new IK_EndEffectorsObjective(IKPlan, "end effector targets", 100));
//	objectives.push_back(new IK_RobotStateRegularizer(IKPlan, 0, 5, "root state regularizer", 1));
	objectives.push_back(new IK_RobotStateRegularizer(IKPlan, 6, IKPlan->gcRobotRepresentation->getDimensionCount() - 1, "joint angles regularizer", 1));
}


double IK_EnergyFunction::computeValue(const dVector& p){
	//assert(s.size() == theMotionPlan->paramCount);

	IKPlan->setParametersFromList(p);

	double totalEnergy = 0;

	for (uint i=0; i<objectives.size(); i++)
		totalEnergy += objectives[i]->computeValue(p);

	//add the regularizer contribution
	if (regularizer > 0){
		tmpVec.resize(p.size());
		if (m_p0.size() != p.size()) m_p0 = p;
		tmpVec = p - m_p0;
		totalEnergy += 0.5*regularizer*tmpVec.dot(tmpVec);
	}

	return totalEnergy;
}

//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
void IK_EnergyFunction::updateRegularizingSolutionTo(const dVector &currentP){
	m_p0 = currentP;
}

void IK_EnergyFunction::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	IKPlan->setParametersFromList(p);

	//add the contribution from the regularizer
	if (regularizer > 0){
		for (int i=0;i<p.size();i++)
			ADD_HES_ELEMENT(hessianEntries, i, i, regularizer, 1);
	}

	//and now the contributions of the individual objectives
	for (uint i = 0; i < objectives.size(); i++)
		objectives[i]->addHessianEntriesTo(hessianEntries, p);
}

void IK_EnergyFunction::addGradientTo(dVector& grad, const dVector& p) {
	IKPlan->setParametersFromList(p);

	resize(grad, p.size());

	//add the contribution from the regularizer
	if (regularizer > 0) {
		if (m_p0.size() != p.size()) m_p0 = p;
		grad = (p - m_p0) * regularizer;
	}

	//and now the contributions of the individual objectives
	for (uint i = 0; i<objectives.size(); i++)
		objectives[i]->addGradientTo(grad, p);
}

//this method gets called whenever a new best solution to the objective function is found
void IK_EnergyFunction::setCurrentBestSolution(const dVector& p){
	updateRegularizingSolutionTo(p);
	IKPlan->setParametersFromList(p);

	if (printDebugInfo){
		Logger::consolePrint("-------------------------------\n");
		Logger::logPrint("-------------------------------\n");

		double totalVal = computeValue(p);
		Logger::consolePrint("=====> total cost: %lf\n", totalVal);
		Logger::logPrint("=====> total cost: %lf\n", totalVal);

		for (uint i=0; i<objectives.size(); i++){
			double w = objectives[i]->weight;
			double v = objectives[i]->computeValue(p);
			Logger::consolePrint("%s: %lf (weight: %lf)\n", objectives[i]->description.c_str(), v, w);
			Logger::logPrint("%s: %lf (weight: %lf)\n", objectives[i]->description.c_str(), v, w);
		}
	}
}


