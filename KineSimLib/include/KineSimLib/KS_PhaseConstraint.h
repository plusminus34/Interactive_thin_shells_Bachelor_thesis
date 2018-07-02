#pragma once

#include "KS_Constraint.h"
#include "KS_MechanicalComponent.h"
#include "KS_LockedComponentConstraint.h"

class KS_PhaseConstraint : public KS_LockedComponentConstraint{
public:
	KS_PhaseConstraint(KS_MechanicalComponent *p_c);
	virtual KS_PhaseConstraint* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut)const;
	~KS_PhaseConstraint(void);

	void setTargetPhase(double val){alphaD = val;}
public:
};

