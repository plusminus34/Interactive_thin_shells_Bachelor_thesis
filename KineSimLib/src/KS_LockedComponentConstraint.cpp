#include "KineSimLib/KS_LockedComponentConstraint.h"

KS_LockedComponentConstraint::KS_LockedComponentConstraint(KS_MechanicalComponent *p_c, double p_alphaD, bool p_freezeAlpha, double p_betaD, bool p_freezeBeta, double p_gammaD, bool p_freezeGamma, double p_pxD, bool p_freezePx, double p_pyD, bool p_freezePy, double p_pzD, bool p_freezePz){
	c = p_c;
	alphaD = p_alphaD;
	freezeAlpha = p_freezeAlpha?1:0;
	betaD = p_betaD;
	freezeBeta = p_freezeBeta?1:0;
	gammaD = p_gammaD;
	freezeGamma = p_freezeGamma?1:0;
	pxD = p_pxD;
	freezePx = p_freezePx?1:0;
	pyD = p_pyD;
	freezePy = p_freezePy?1:0;
	pzD = p_pzD;
	freezePz = p_freezePz?1:0;

	anglesWeight = 100;
	positionWeight=100;
	PositionWeight = P3D(positionWeight, positionWeight, positionWeight);
}

KS_LockedComponentConstraint* KS_LockedComponentConstraint::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut)const{
	KS_LockedComponentConstraint* constraint = new KS_LockedComponentConstraint(*this);
	constraint->c=pCompOut;
	return constraint;	
}

KS_LockedComponentConstraint::~KS_LockedComponentConstraint(void){
}

void KS_LockedComponentConstraint::setTargetPosition(const P3D& pos){
	// replaced x  y z
	pxD = pos[0];
	pyD = pos[1];
	pzD = pos[2];
}

double KS_LockedComponentConstraint::getEnergy(){
	return 0.5 * (	freezeAlpha * SQR(c->getAlpha() - alphaD) * anglesWeight + 
					freezeBeta * SQR(c->getBeta() - betaD) * anglesWeight +
					freezeGamma * SQR(c->getGamma() - gammaD) * anglesWeight +

					freezePx * SQR(c->getWorldCenterPosition()[0] - pxD) * PositionWeight[0] +
					freezePy * SQR(c->getWorldCenterPosition()[1] - pyD) * PositionWeight[1] +
					freezePz * SQR(c->getWorldCenterPosition()[2] - pzD) * PositionWeight[2]
				);	
	//replaced x y z
}

//each constraint is composed of several scalar constraints - this is how many
int KS_LockedComponentConstraint::getConstraintCount(){
	return freezeAlpha + freezeBeta + freezeGamma + freezePx + freezePy + freezePz;
}

void KS_LockedComponentConstraint::computeEnergyGradient(){
	FAST_RESIZE_VEC(dE_ds, KS_MechanicalComponent::getStateSize());
	//replaced x y z
	dE_ds[0] = freezeGamma * (c->getGamma() - gammaD) * anglesWeight;
	dE_ds[1] = freezeBeta * (c->getBeta() - betaD) * anglesWeight;
	dE_ds[2] = freezeAlpha * (c->getAlpha() - alphaD) * anglesWeight;
	dE_ds[3] = freezePx * (c->getWorldCenterPosition()[0] - pxD)*PositionWeight[0];
	dE_ds[4] = freezePy * (c->getWorldCenterPosition()[1] - pyD)*PositionWeight[1];
	dE_ds[5] = freezePz * (c->getWorldCenterPosition()[2] - pzD)*PositionWeight[2];
}

void KS_LockedComponentConstraint::computeEnergyHessian(){
	FAST_RESIZE_MAT(ddE_dsds, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
	
	if (freezeGamma) ddE_dsds(0,0) = anglesWeight;
	if (freezeBeta) ddE_dsds(1,1) =  anglesWeight;
	if (freezeAlpha) ddE_dsds(2,2) = anglesWeight;

	if (freezePx) ddE_dsds(3,3) = PositionWeight[0];
	if (freezePy) ddE_dsds(4,4) = PositionWeight[1];
	if (freezePz) ddE_dsds(5,5) = PositionWeight[2];
}
