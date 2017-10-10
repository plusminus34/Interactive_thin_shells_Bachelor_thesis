#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/FunctionConstraints.h>
#include <MathLib/Matrix.h>
#include <ControlLib/QPControlPlan.h>

class QPControlConstraints : public FunctionConstraints {
public:
	QPControlConstraints(QPControlPlan* mp);
	virtual ~QPControlConstraints(void);

	int FequalsMAConstraintsStartIndex = -1;
	int eeNoSlipConstraintsStartIndex = -1;

private:
	//the energy function operates on a motion plan...
	QPControlPlan* qpPlan;

	virtual int getEqualityConstraintCount();
	virtual int getInequalityConstraintCount();

	virtual const dVector& getEqualityConstraintValues(const dVector& p);
	virtual const dVector& getInequalityConstraintValues(const dVector& p);

	void addFequalsMAConstraints(int constraintStartIndex);
	void addEndEffectorNoSlipConstraints(int constraintStartIndex);

	virtual const dVector& getBoundConstraintsMinValues();
	virtual const dVector& getBoundConstraintsMaxValues();

	/*! 
	*  evaluates the constraint jacobian
	*/
//	virtual void addEqualityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p);

};
