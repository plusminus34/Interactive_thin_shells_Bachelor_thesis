#include "KineSimLib/KS_PhaseToPhaseConstraint.h"
#include "KineSimLib/KS_MechanicalAssembly.h"

KS_PhaseToPhaseConstraint::KS_PhaseToPhaseConstraint(KS_MechanicalComponent *p_c1, KS_MechanicalComponent *p_c2, double p_phaseRatio, double p_relativeOffset){
	c1 = p_c1;
	c2 = p_c2;

	phaseRatio = p_phaseRatio;
	relativeOffset = p_relativeOffset;

	angleWeight=100;
}

KS_PhaseToPhaseConstraint* KS_PhaseToPhaseConstraint::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut)const{
	KS_PhaseToPhaseConstraint* constraint = new KS_PhaseToPhaseConstraint(*this);
	constraint->c1=pCompIn;
	constraint->c2=pCompOut;
	return constraint;
}

KS_PhaseToPhaseConstraint::~KS_PhaseToPhaseConstraint(void){
}

double KS_PhaseToPhaseConstraint::getEnergy(){
	return 0.5 * (SQR(c1->getAlpha() - (phaseRatio * c2->getAlpha() + relativeOffset)))*angleWeight;
}

//each constraint is composed of several scalar constraints - this is how many
int KS_PhaseToPhaseConstraint::getConstraintCount(){
	return 1;
}

//returns the current values of the constraints
dVector* KS_PhaseToPhaseConstraint::getConstraintValues(){
	C.resize(1, 0);
	C[0] = (c1->getAlpha() - (phaseRatio * c2->getAlpha() + relativeOffset))*angleWeight;
	return &C;
}

void KS_PhaseToPhaseConstraint::computeConstraintJacobian(){
	FAST_RESIZE_MAT(dCds1, getConstraintCount(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dCds2, getConstraintCount(), KS_MechanicalComponent::getStateSize());

	dCds1(0, 2) = angleWeight;
	dCds2(0, 2) = -phaseRatio*angleWeight;
}

void KS_PhaseToPhaseConstraint::computeEnergyGradient(){
	FAST_RESIZE_VEC(dE_ds1, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_VEC(dE_ds2, KS_MechanicalComponent::getStateSize());

	dE_ds1[2] = (c1->getAlpha() - (phaseRatio * c2->getAlpha() + relativeOffset))*angleWeight;
	dE_ds2[2] = -phaseRatio*(c1->getAlpha() - (phaseRatio * c2->getAlpha() + relativeOffset))*angleWeight;
	
}

void KS_PhaseToPhaseConstraint::computeEnergyHessian(){
	FAST_RESIZE_MAT(ddE_ds1ds1, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(ddE_ds1ds2, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(ddE_ds2ds1, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(ddE_ds2ds2, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());

	ddE_ds1ds1(2,2) = angleWeight;
	ddE_ds2ds2(2,2) = phaseRatio*phaseRatio*angleWeight;
	ddE_ds1ds2(2,2) = -phaseRatio*angleWeight;
	ddE_ds2ds1(2,2) = -phaseRatio*angleWeight;
}

