#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/FunctionConstraints.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class LocomotionEngine_Constraints : public FunctionConstraints {
public:
	LocomotionEngine_Constraints(LocomotionEngineMotionPlan* mp);
	virtual ~LocomotionEngine_Constraints(void);

	int weightsUnityConstraintsStartIndex = -1;
	int periodicBoundaryConstraintsIndex = -1;
	int footSlidingConstraintsStartIndex = -1;
	int frictionConeConstraintsStartIndex = -1;

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	virtual int getEqualityConstraintCount();
	virtual int getInequalityConstraintCount();

	virtual const dVector& getEqualityConstraintValues(const dVector& p);
	virtual const dVector& getInequalityConstraintValues(const dVector& p);

	// Equality constraints
	void addWeightsAPartitionOfUnityConstraints(int constraintStartIndex);
	void addPeriodicMotionConstraints(int constraintStartIndex);
	void addFootSlidingConstraints(int constraintStartIndex);

	// Inequality constraints
	void addFrictionConeConstraints(int constraintStartIndex);

	virtual const dVector& getBoundConstraintsMinValues();
	virtual const dVector& getBoundConstraintsMaxValues();



	/*! 
	*  evaluates the constraint jacobian
	*/
//	virtual void addEqualityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p);

};
