#pragma once

#include <math.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>
#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/FunctionConstraints.h>

/*!
	A multi-dimensional function that expresses linear equality constraints. 
	This can be used either to model soft constraints or hard constraints. It inherits
	both ObjectiveFunction and FunctionConstraints, and so it can be used as either. The 
	value of the objective is 1/2 C'C, where C is the set of equality constraints that 
	this function represents.
*/
class SoftConstraints : public ObjectiveFunction, public FunctionConstraints {
public:
	SoftConstraints();
	virtual ~SoftConstraints();

	// returns 1/2 C'C, where C is the current set of equality constraint values
	virtual double computeValue(const dVector& p);

	//default gradient & hessian computation, as well as evaluation of constraint jacobians is inherited from the base classes ObjectiveFunction and FunctionConstraints

};
