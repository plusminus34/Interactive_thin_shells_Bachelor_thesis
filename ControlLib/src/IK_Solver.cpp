#include <ControlLib/IK_Solver.h>
#include <ControlLib/BodyFrame.h>
#include <ControlLib/GenericLimb.h>

IK_Solver::IK_Solver(Robot* robot){
	ikPlan = new IK_Plan(robot);
	ikEnergyFunction = new IK_EnergyFunction(ikPlan);
	ikOptimizer = new IK_Optimizer(ikPlan, ikEnergyFunction);
}

IK_Solver::IK_Solver(Robot* robot, bool freezeRootConfiguration) {
	ikPlan = new IK_Plan(robot);
	ikPlan->optimizeRootConfiguration = !freezeRootConfiguration;
	ikEnergyFunction = new IK_EnergyFunction(ikPlan);
	ikOptimizer = new IK_Optimizer(ikPlan, ikEnergyFunction);
}


IK_Solver::~IK_Solver(void){
	delete ikPlan;
	delete ikEnergyFunction;
	delete ikOptimizer;
}

void IK_Solver::solve(int nSteps, bool resetTargetState) {
	ikPlan->setCurrentIKStateFromRobot();
	if (resetTargetState)
		ikPlan->setTargetIKStateFromRobot();
	ikOptimizer->optimizePlan(nSteps);
	ikPlan->setCurrentIKStateToRobot();
}
