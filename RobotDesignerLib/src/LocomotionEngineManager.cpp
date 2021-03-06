#include <RobotDesignerLib/LocomotionEngineManager.h>
#include <RobotDesignerLib/MPO_PeriodicRobotStateTrajectoriesObjective.h>

#include <OptimizationLib/GradientDescentFunctionMinimizer.h>

LocomotionEngineManager::LocomotionEngineManager() : minimizerNewton(1),minimizerBFGS(10) {
}

void LocomotionEngineManager::createSolverComponents() {
	energyFunction = new LocomotionEngine_EnergyFunction(motionPlan);
	constraints = new LocomotionEngine_Constraints(motionPlan);
	constrainedObjectiveFunction = new ConstrainedObjectiveFunction(energyFunction, constraints);
}

LocomotionEngineManager::~LocomotionEngineManager(void) {
	delete energyFunction;
	delete constraints;
	delete constrainedObjectiveFunction;
}

double LocomotionEngineManager::optimizeMoptionPlan(int maxIterations) {
	double val = 0;
	dVector params;
	this->motionPlan->writeMPParametersToList(params);
	if (params.size() == 0)
		return 0;
	if (energyFunction->printDebugInfo)
		Logger::logPrint("Starting Problem size: %d\n", params.size());

	if (useObjectivesOnly) {
		if (optimizationMethod == OptMethod::Newton)
		{
			minimizerNewton.maxLineSearchIterations = 12;
			minimizerNewton.printOutput = energyFunction->printDebugInfo;
			minimizerNewton.hessCorrectionMethod = hessCorrectionMethod;
			minimizerNewton.minimize(energyFunction, params, val);
		}
			
		else if (optimizationMethod == OptMethod::lbfgs)
		{
			minimizerBFGS.maxLineSearchIterations = 40;
			minimizerBFGS.printOutput = energyFunction->printDebugInfo;
			minimizerBFGS.minimize(energyFunction, params, val);
		}
			
	}
	else {
		SQPFunctionMinimizer minimizer(maxIterations);
		minimizer.maxLineSearchIterations_ = 12;
		minimizer.printOutput_ = energyFunction->printDebugInfo;
		//Logger::printStatic("There are %d parameters\n", params.size());
		minimizer.minimize(constrainedObjectiveFunction, params, val);
	}
	return val;
}

#include <iostream>

double LocomotionEngineManager::runMOPTStep() {
	motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);

	Timer timer;
	if (checkDerivatives) {
		dVector params;
		motionPlan->writeMPParametersToList(params);

//		energyFunction->testGradientWithFD(params);
//		energyFunction->testHessianWithFD(params);
		energyFunction->testIndividualGradient(params);
		energyFunction->testIndividualHessian(params);
	}
	if (checkHessianPSD)
	{
		dVector params;
		motionPlan->writeMPParametersToList(params);
		energyFunction->testIndividualHessianPSD(params);
	}

	energyFunction->printDebugInfo = printDebugInfo;

	double energyVal = optimizeMoptionPlan();
	writeParamsToFile = printDebugInfo;
	if (writeParamsToFile) {
		motionPlan->writeParamsToFile("..//out//MPParams.p");
		motionPlan->writeRobotMotionTrajectoriesToFile("..//out//robotMotion.traj");
	}

	if (printDebugInfo)
		Logger::consolePrint("total time elapsed: %lfs\n", timer.timeEllapsed());

	if(writeVelocityProfileToFile)
	{
		std::vector<LocomotionEngineMotionPlan::JointVelocity> velProfile;
		std::string error;
		if(!motionPlan->getJointAngleVelocityProfile(velProfile, error))
		{
			Logger::consolePrint("Could not compute velocity profile: %s\n", error);
		}
		else
		{
			std::ofstream file ("../out/jointAngleVelocityProfile.txt");
			file << "time\tqIndex\tjoint angular velocity" << std::endl;

			for (const auto &v : velProfile) {
				file << v.t << "\t" << v.qIndex << "\t" << v.velocity << std::endl;
			}

			file.close();
		}
	}

	// print wheel speeds and angles
//	{
//		int i = 0;
//		int j = 0;
//		for (const auto &ee : motionPlan->endEffectorTrajectories) {
//			for(const auto &beta : ee.wheelAxisBeta)
//			{
//				std::cout << "beta (" << i << ", " << j << ")\t" << beta << std::endl;
//				j++;
//			}
//			i++;
//		}
//	}

	return energyVal;
}
