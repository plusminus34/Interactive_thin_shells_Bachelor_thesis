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

void KS_LockedComponentConstraint::setTargetPosition(const Point3d& pos){
	pxD = pos.x;
	pyD = pos.y;
	pzD = pos.z;
}

double KS_LockedComponentConstraint::getEnergy(){
	return 0.5 * (	freezeAlpha * SQR(c->getAlpha() - alphaD) * anglesWeight + 
					freezeBeta * SQR(c->getBeta() - betaD) * anglesWeight +
					freezeGamma * SQR(c->getGamma() - gammaD) * anglesWeight +

					freezePx * SQR(c->getWorldCenterPosition().x - pxD) * positionWeight +
					freezePy * SQR(c->getWorldCenterPosition().y - pyD) * positionWeight+
					freezePz * SQR(c->getWorldCenterPosition().z - pzD) * positionWeight
				);	
}

//each constraint is composed of several scalar constraints - this is how many
int KS_LockedComponentConstraint::getConstraintCount(){
	return freezeAlpha + freezeBeta + freezeGamma + freezePx + freezePy + freezePz;
}

//returns the current values of the constraints
dVector* KS_LockedComponentConstraint::getConstraintValues(){
	C.clear();
	if (freezeGamma) C.push_back((c->getGamma() - gammaD) * anglesWeight);
	if (freezeBeta) C.push_back((c->getBeta() - betaD) * anglesWeight);
	if (freezeAlpha) C.push_back((c->getAlpha() - alphaD) * anglesWeight);

	if (freezePx) C.push_back((c->getWorldCenterPosition().x - pxD)*positionWeight);
	if (freezePy) C.push_back((c->getWorldCenterPosition().y - pyD)*positionWeight);
	if (freezePz) C.push_back((c->getWorldCenterPosition().z - pzD)*positionWeight);

	return &C;
}

void KS_LockedComponentConstraint::computeEnergyGradient(){
	FAST_RESIZE_VEC(dE_ds, KS_MechanicalComponent::getStateSize());

	FAST_RESIZE_MAT(dCds, getConstraintCount(), KS_MechanicalComponent::getStateSize());

	dE_ds[0] = freezeGamma * (c->getGamma() - gammaD) * anglesWeight;
	dE_ds[1] = freezeBeta * (c->getBeta() - betaD) * anglesWeight;
	dE_ds[2] = freezeAlpha * (c->getAlpha() - alphaD) * anglesWeight;
	dE_ds[3] = freezePx * (c->getWorldCenterPosition().x - pxD)*positionWeight;
	dE_ds[4] = freezePy * (c->getWorldCenterPosition().y - pyD)*positionWeight;
	dE_ds[5] = freezePz * (c->getWorldCenterPosition().z - pzD)*positionWeight;
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
