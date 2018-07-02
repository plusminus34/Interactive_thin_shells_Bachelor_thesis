#include "KineSimLib/KS_PhaseConstraint.h"

KS_PhaseConstraint::KS_PhaseConstraint(KS_MechanicalComponent *p_c) : KS_LockedComponentConstraint(p_c, p_c->getAlpha(), true, 0, false, 0, false, 0, false, 0, false, 0, false){
}

KS_PhaseConstraint* KS_PhaseConstraint::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut)const{
	KS_PhaseConstraint* constraint = new KS_PhaseConstraint(*this);
	constraint->c=pCompOut;
	return constraint;
}

KS_PhaseConstraint::~KS_PhaseConstraint(void){
}

