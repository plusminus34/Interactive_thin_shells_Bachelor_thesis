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

					freezePx * SQR(c->getWorldCenterPosition()[0] - pxD) * positionWeight +
					freezePy * SQR(c->getWorldCenterPosition()[1] - pyD) * positionWeight+
					freezePz * SQR(c->getWorldCenterPosition()[2] - pzD) * positionWeight
				);	
	//replaced x y z
}

//each constraint is composed of several scalar constraints - this is how many
int KS_LockedComponentConstraint::getConstraintCount(){
	return freezeAlpha + freezeBeta + freezeGamma + freezePx + freezePy + freezePz;
}

//returns the current values of the constraints
dVector* KS_LockedComponentConstraint::getConstraintValues(){
	//C.clear();
	C.resize(getConstraintCount()); C.setZero();
	int cIndex = 0;

	if (freezeGamma) { C[cIndex]=(c->getGamma() - gammaD) * anglesWeight; cIndex++; }
	if (freezeBeta)  { C[cIndex]=(c->getBeta() - betaD) * anglesWeight; cIndex++; }
	if (freezeAlpha) { C[cIndex]=(c->getAlpha() - alphaD) * anglesWeight; cIndex++; }
	//replaced x y z
	if (freezePx) { C[cIndex]=(c->getWorldCenterPosition()[0] - pxD)*positionWeight; cIndex++; }
	if (freezePy) { C[cIndex]=(c->getWorldCenterPosition()[1] - pyD)*positionWeight; cIndex++; }
	if (freezePz) { C[cIndex]=(c->getWorldCenterPosition()[2] - pzD)*positionWeight; cIndex++; }

	return &C;
}

void KS_LockedComponentConstraint::computeEnergyGradient(){
	FAST_RESIZE_VEC(dE_ds, KS_MechanicalComponent::getStateSize());

	FAST_RESIZE_MAT(dCds, getConstraintCount(), KS_MechanicalComponent::getStateSize());
	//replaced x y z
	dE_ds[0] = freezeGamma * (c->getGamma() - gammaD) * anglesWeight;
	dE_ds[1] = freezeBeta * (c->getBeta() - betaD) * anglesWeight;
	dE_ds[2] = freezeAlpha * (c->getAlpha() - alphaD) * anglesWeight;
	dE_ds[3] = freezePx * (c->getWorldCenterPosition()[0] - pxD)*positionWeight;
	dE_ds[4] = freezePy * (c->getWorldCenterPosition()[1] - pyD)*positionWeight;
	dE_ds[5] = freezePz * (c->getWorldCenterPosition()[2] - pzD)*positionWeight;
}

void KS_LockedComponentConstraint::computeEnergyHessian(){
	FAST_RESIZE_MAT(ddE_dsds, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
	
	if (freezeGamma) ddE_dsds(0,0) = anglesWeight;
	if (freezeBeta) ddE_dsds(1,1) =  anglesWeight;
	if (freezeAlpha) ddE_dsds(2,2) = anglesWeight;

	if (freezePx) ddE_dsds(3,3) = positionWeight;
	if (freezePy) ddE_dsds(4,4) = positionWeight;
	if (freezePz) ddE_dsds(5,5) = positionWeight;
}

void KS_LockedComponentConstraint::computeConstraintJacobian(){
	FAST_RESIZE_MAT(dCds, getConstraintCount(), KS_MechanicalComponent::getStateSize());

	int cIndex = 0;
	if (freezeGamma){dCds(cIndex, 0) =  anglesWeight; cIndex++;}
	if (freezeBeta){dCds(cIndex, 1) =  anglesWeight; cIndex++;}
	if (freezeAlpha){dCds(cIndex, 2) =  anglesWeight; cIndex++;}
	if (freezePx){dCds(cIndex, 3) = positionWeight; cIndex++;}
	if (freezePy){dCds(cIndex, 4) = positionWeight; cIndex++;}
	if (freezePz){dCds(cIndex, 5) = positionWeight; cIndex++;}
}
