#pragma once

#include <OptimizationLib/FunctionConstraints.h>

class SoftLocoSolver;
 
class SoftLocoConstraints : public FunctionConstraints {
public:
	SoftLocoConstraints(SoftLocoSolver* loco);
	virtual ~SoftLocoConstraints() {} 

public:
	SoftLocoSolver* loco;

private:
	virtual int getEqualityConstraintCount()   { return 0; }
	virtual int getInequalityConstraintCount() { return 0; }
};
