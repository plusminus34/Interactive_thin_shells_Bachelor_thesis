#include "KineSimLib/KS_NonLinearPhaseConstraint.h"

KS_NonLinearPhaseConstraint::KS_NonLinearPhaseConstraint(KS_MechanicalComponent *p_c1, KS_MechanicalComponent *p_c2, double f1, double f2, double f3, double f4){
	c1 = p_c1;
	c2 = p_c2;

	phaseProfile.setRBF1DFunctionData(f1, f2, f3, f4);
}

KS_NonLinearPhaseConstraint* KS_NonLinearPhaseConstraint::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut)const{
	KS_NonLinearPhaseConstraint* constraint = new KS_NonLinearPhaseConstraint(*this);
	constraint->c1=pCompIn;
	constraint->c2=pCompOut;
	return constraint;
}

double KS_NonLinearPhaseConstraint::get_f(double a){
	double t = a / (2*PI);
	int extraRevolutions = (int)floor(t);
	t -= extraRevolutions;
	return (phaseProfile.evaluate(t) + extraRevolutions) * 2 * PI;
}

double KS_NonLinearPhaseConstraint::get_dfda(double a){
	double t = a / (2*PI);
	int extraRevolutions = (int)floor(t);
	t -= extraRevolutions;
	return phaseProfile.evaluateDerivative(t);
}

double KS_NonLinearPhaseConstraint::get_dda_dfda(double a){
	double t = a / (2*PI);
	int extraRevolutions = (int)floor(t);
	t -= extraRevolutions;
	return phaseProfile.evaluateSecondDerivative(t);
}


KS_NonLinearPhaseConstraint::~KS_NonLinearPhaseConstraint(void){
}

double KS_NonLinearPhaseConstraint::getEnergy(){
	return 0.5 * (SQR(c2->getAlpha() + get_f(c1->getAlpha())));
}

void KS_NonLinearPhaseConstraint::computeConstraintJacobian(){
	FAST_RESIZE_MAT(dCds1, getConstraintCount(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(dCds2, getConstraintCount(), KS_MechanicalComponent::getStateSize());

	dCds1(0, 2) = get_dfda(c1->getAlpha());
	dCds2(0, 2) = 1;
}

void KS_NonLinearPhaseConstraint::computeEnergyGradient(){
	FAST_RESIZE_VEC(dE_ds1, KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_VEC(dE_ds2, KS_MechanicalComponent::getStateSize());

	dE_ds1[2] = (c2->getAlpha() + get_f(c1->getAlpha())) * get_dfda(c1->getAlpha());
	dE_ds2[2] = (c2->getAlpha() + get_f(c1->getAlpha()));

}

void KS_NonLinearPhaseConstraint::computeEnergyHessian(){
	FAST_RESIZE_MAT(ddE_ds1ds1, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(ddE_ds1ds2, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(ddE_ds2ds1, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
	FAST_RESIZE_MAT(ddE_ds2ds2, KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
	
	ddE_ds1ds1(2,2) = get_dfda(c1->getAlpha())*get_dfda(c1->getAlpha()) + (c2->getAlpha() + get_f(c1->getAlpha())) * get_dda_dfda(c1->getAlpha());
	ddE_ds2ds2(2,2) = 1;
	ddE_ds1ds2(2,2) = get_dfda(c1->getAlpha());
	ddE_ds2ds1(2,2) = get_dfda(c1->getAlpha());
}


//each constraint is composed of several scalar constraints - this is how many
int KS_NonLinearPhaseConstraint::getConstraintCount(){
	return 1;
}

//returns the current values of the constraints
dVector* KS_NonLinearPhaseConstraint::getConstraintValues(){
	C.resize(1, 0);
	C[0] = c2->getAlpha() + get_f(c1->getAlpha());
	return &C;
}
