#pragma once

#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>



class MPO_EEPoseOffsetConstraintToInitial : public ObjectiveFunction {
public:
	MPO_EEPoseOffsetConstraintToInitial(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_EEPoseOffsetConstraintToInitial(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);
		

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;	

};


