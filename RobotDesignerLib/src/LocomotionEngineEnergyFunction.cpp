#include <RobotDesignerLib/LocomotionEngineEnergyFunction.h>

#include <RobotDesignerLib/MPO_SmoothCOMTrajectories.h>
#include <RobotDesignerLib/MPO_DynamicStabilityObjective.h>
#include <RobotDesignerLib/MPO_SmoothStanceLegMotionObjective.h>
#include <RobotDesignerLib/MPO_StanceLegMotionRegularizer.h>
#include <RobotDesignerLib/MPO_COMTravelObjective.h>
#include <RobotDesignerLib/MPO_FeetSlidingObjective.h>
#include <RobotDesignerLib/MPO_FeetPathSmoothnessObjective.h>
#include <RobotDesignerLib/MPO_BarycentricWeightsRegularizerObjective.h>
#include <RobotDesignerLib/MPO_RobotStateRegularizer.h>
#include <RobotDesignerLib/MPO_RobotCOMObjective.h>
#include <RobotDesignerLib/MPO_RobotEndEffectorsObjective.h>
#include <RobotDesignerLib/MPO_PeriodicRobotStateTrajectoriesObjective.h>
#include <RobotDesignerLib/MPO_RobotTurningObjective.h>
#include <RobotDesignerLib/MPO_SmoothRobotMotionTrajectories.h>
#include <RobotDesignerLib/MPO_ForceAccelObjective.h>
#include <RobotDesignerLib/MPO_TorqueAngularAccelObjective.h>
#include <RobotDesignerLib/MPO_RobotCOMOrientationsObjective.h>
#include <RobotDesignerLib/MPO_COMTurningObjective.h>
#include <RobotDesignerLib/MPO_GRFSoftConstraints.h>
#include <RobotDesignerLib/MPO_NonLimbMotionRegularizer.h>
#include <RobotDesignerLib/MPO_NonLimbSmoothMotionObjective.h>
#include <RobotDesignerLib/MPO_RobotStateTransitionRegularizer.h>
#include <RobotDesignerLib/MPO_PseudoLimbLengthConstraint.h>
#include <RobotDesignerLib/MPO_PseudoPeriodicEECOMPoseConstraint.h>
#include <RobotDesignerLib/MPO_EEPoseOffsetConstraintToInitial.h>
#include <RobotDesignerLib/MPO_COMOrientationFluctuationRegularizer.h>

//TODO: should change IP - based mopt to use additional dofs of body rotation, and then have
//the objective that matches those to robot state, just the way com dofs are matched to robot's com...


LocomotionEngine_EnergyFunction::LocomotionEngine_EnergyFunction(LocomotionEngineMotionPlan* mp){
	theMotionPlan = mp;
	regularizer = 1;

	printDebugInfo = true;
}

LocomotionEngine_EnergyFunction::~LocomotionEngine_EnergyFunction(void){
	for (uint i=0;i<objectives.size();i++)
		delete objectives[i];
}

void LocomotionEngine_EnergyFunction::testIndividualGradient(dVector& params)
{
	vector<double> origWeights;
	for (auto obj : objectives)
	{
		origWeights.push_back(obj->weight);
		obj->weight = 0;
	}

	for (int i = 0; i < (int)objectives.size(); i++)
	{
		if (i > 0)
			objectives[i - 1]->weight = 0;
		objectives[i]->weight = origWeights[i];

		if (objectives[i]->description != "GRF bound constraints") continue;

		Logger::logPrint("\n----------- Testing objective %s -------------\n", objectives[i]->description.c_str());
		Logger::print("\n----------- Testing objective %s -------------\n", objectives[i]->description.c_str());
		testGradientWithFD(params);
	}

	for (int i = 0; i < (int)objectives.size(); i++)
	{
		objectives[i]->weight = origWeights[i];
	}
}

void LocomotionEngine_EnergyFunction::testIndividualHessian(dVector& params)
{
	vector<double> origWeights;
	for (auto obj : objectives)
	{
		origWeights.push_back(obj->weight);
		obj->weight = 0;
	}

	for (int i = 0; i < (int)objectives.size(); i++)
	{
		if (i > 0)
			objectives[i - 1]->weight = 0;
		objectives[i]->weight = origWeights[i];

		if (objectives[i]->description != "GRF bound constraints") continue;

		Logger::print("\n----------- Testing objective %s -------------\n", objectives[i]->description.c_str());
		Logger::logPrint("\n----------- Testing objective %s -------------\n", objectives[i]->description.c_str());
		testHessianWithFD(params);
	}

	for (int i = 0; i < (int)objectives.size(); i++)
	{
		objectives[i]->weight = origWeights[i];
	}
}

double LocomotionEngine_EnergyFunction::computeValue(const dVector& p){
	assert(p.size() == theMotionPlan->paramCount);

	theMotionPlan->setMPParametersFromList(p);

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
	for (uint i = 0; i < objectives.size(); i++)
		objectives[i]->addHessianEntriesTo(hessianEntries, p);
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
		objectives[i]->addGradientTo(grad, p);
}

//this method gets called whenever a new best solution to the objective function is found
void LocomotionEngine_EnergyFunction::setCurrentBestSolution(const dVector& p){
	updateRegularizingSolutionTo(p);
	theMotionPlan->setMPParametersFromList(p);

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

			if (objectives[i]->description == "torque angular acceleration objective")
			{
				static_cast<MPO_TorqueAngularAccelObjective*>(objectives[i])->updateDummyMatrices();
			}
		}
	}
}


