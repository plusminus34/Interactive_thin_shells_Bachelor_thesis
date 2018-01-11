#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class LocomotionEngine_EnergyFunction : public ObjectiveFunction {
public:

	LocomotionEngine_EnergyFunction(LocomotionEngineMotionPlan* mp);
	virtual ~LocomotionEngine_EnergyFunction(void);

	//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
	void updateRegularizingSolutionTo(const dVector &currentP);
	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

	//this method gets called whenever a new best solution to the objective function is found
	virtual void setCurrentBestSolution(const dVector& p);

	void testIndividualGradient(dVector& params);
	void testIndividualHessian(dVector& params);

	void testIndividualHessianPSD(dVector& params);
	void addObjectiveFunction(ObjectiveFunction* obj, std::string string = "no group");
	void setWeights(const dVector& weights);
public:
	DynamicArray<ObjectiveFunction*> objectives;
	std::map<std::string, std::vector<ObjectiveFunction*>> objGroups;

	bool printDebugInfo;
	double regularizer;

private:

	//this is the configuration of the sim mesh that is used as a regularizing solution...
	dVector m_p0;
	dVector tmpVec;

public:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;
};

