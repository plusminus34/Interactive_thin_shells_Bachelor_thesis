#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/FunctionConstraints.h>
#include <MathLib/Matrix.h>

#include <FEMSimLib/SimulationMesh.h>
#include <FEMSimLib/CSTSimulationMesh2D.h>
#include <FEMSimLib/CSTElement2D.h>

class TopOptConstraints : public FunctionConstraints {
public:
	TopOptConstraints(SimulationMesh* simMesh);
	virtual ~TopOptConstraints(void);

	SimulationMesh* simMesh;

	double totalMassUpperBound = 1.0;

private:
	//the energy function operates on a motion plan...
	TopOptConstraints* theMotionPlan;

	virtual int getEqualityConstraintCount();
	virtual int getInequalityConstraintCount();

	virtual const dVector& getEqualityConstraintValues(const dVector& p);
	virtual const dVector& getInequalityConstraintValues(const dVector& p);

	virtual const dVector& getBoundConstraintsMinValues();
	virtual const dVector& getBoundConstraintsMaxValues();

	/*! 
	*  evaluates the constraint jacobian
	*/
//	virtual void addEqualityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p);

/*!
*  evaluates the constraint jacobian
*/
//	virtual void addInequalityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p);

};
