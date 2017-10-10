#pragma once

#include <ControlLib/QPControlEnergyFunction.h>
#include <ControlLib/QPControlConstraints.h>
#include <OptimizationLib/ConstrainedObjectiveFunction.h>

/**
	This is a locomotion engine that works for arbitrary robot types
*/
class QPControlEngine{
public:
	QPControlEngine(QPControlPlan* qpPlan);
	virtual ~QPControlEngine(void);

	double optimizePlan();

public:
	QPControlEnergyFunction* energyFunction;
	QPControlConstraints* constraints;
	ConstrainedObjectiveFunction* constrainedObjectiveFunction;
	QPControlPlan* qpPlan;

};



