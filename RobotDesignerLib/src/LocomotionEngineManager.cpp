#include <RobotDesignerLib/LocomotionEngineManager.h>
#include <RobotDesignerLib/MPO_PeriodicRobotStateTrajectoriesObjective.h>


LocomotionEngineManager::LocomotionEngineManager(){
}

void LocomotionEngineManager::drawMotionPlan(double f, int animationCycle, bool drawRobot, bool drawSkeleton, bool drawPlanDetails, bool drawContactForces, bool drawOrientation){
	// motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);
	motionPlan->drawMotionPlan(f, animationCycle, drawRobot, drawSkeleton, drawPlanDetails, drawContactForces, drawOrientation);
}

LocomotionEngineManager::~LocomotionEngineManager(){
}

double LocomotionEngineManager::runMOPTStep() {
	motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);

	Timer timer;
	if (checkDerivatives) {
		dVector params;
		motionPlan->writeMPParametersToList(params);

		locomotionEngine->energyFunction->testGradientWithFD(params);
		locomotionEngine->energyFunction->testHessianWithFD(params);
		locomotionEngine->energyFunction->testIndividualGradient(params);
		locomotionEngine->energyFunction->testIndividualHessian(params);
	}
	locomotionEngine->energyFunction->printDebugInfo = printDebugInfo;

	double energyVal = 0;
	if (useBFGS)
		energyVal = locomotionEngine->optimizePlan_BFGS();
	else
		energyVal = locomotionEngine->optimizePlan();

	motionPlan->writeParamsToFile("..//out//MPParams.p");

	if (printDebugInfo)
		Logger::consolePrint("total time ellapsed: %lfs\n", timer.timeEllapsed());

	return energyVal;
}
