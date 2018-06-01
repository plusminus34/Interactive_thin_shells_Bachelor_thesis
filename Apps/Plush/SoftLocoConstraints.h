#pragma once

#include <OptimizationLib/FunctionConstraints.h>
#include <PlushHelpers\helpers_star.h>

class SoftLocoSolver;
 
class SoftLocoConstraints : public FunctionConstraints {
public:
	SoftLocoConstraints(SoftLocoSolver *loco);
	virtual ~SoftLocoConstraints() {} 

public:
	SoftLocoSolver *loco;

private:
	virtual int getEqualityConstraintCount()   { return 0; }
	virtual int getInequalityConstraintCount() { return KT(); }
	virtual const dVector &getInequalityConstraintValues(const dVector &p);

public:
	void checkConstraints(const vector<dVector> &);

private:
	int K();
	int Z();
	int S();
	int T();
	int ZS();
	int KT();
};
