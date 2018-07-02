#pragma once

#include "KS_Constraint.h"
#include "KS_MechanicalComponent.h"
#include <MathLib/RBF1D_4Point01Interval.h>

class KS_NonLinearPhaseConstraint : public KS_Constraint{
public:
	KS_NonLinearPhaseConstraint(KS_MechanicalComponent *p_c1, KS_MechanicalComponent *p_c2, double f1, double f2, double f3, double f4);
	virtual KS_NonLinearPhaseConstraint* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut)const;
	~KS_NonLinearPhaseConstraint(void);

	virtual double getEnergy();
	virtual void computeConstraintJacobian();
	virtual void computeEnergyGradient();
	virtual void computeEnergyHessian();

	virtual int getNumberOfAffectedComponents() {return 2;}
	virtual KS_MechanicalComponent* getIthAffectedComponent(int i){if (i==0) return c1; else return c2;}

	//for testing purposes only...
	virtual dVector* get_dE_dsi(int i) {if (i==0) return &dE_ds1; else return &dE_ds2;}
	virtual Matrix* get_ddE_dsidsj(int i, int j) {if (i == 0 && j == 0) return &ddE_ds1ds1; if (i==0 && j==1) return &ddE_ds1ds2; if (i==1 && j==0) return &ddE_ds2ds1; return &ddE_ds2ds2;}

	//each constraint is composed of several scalar constraints - this is how many
	virtual int getConstraintCount();
	//returns the current values of the constraints
	virtual dVector* getConstraintValues();
	//returns the jacobian that tells us how the values of the constraint change with the state of the ith component involved in the constraint
	virtual Matrix* getConstraintJacobian(int i) {if (i == 0) return &dCds1; return &dCds2;}



private:
	//we need to ensure that the relationship between the phases of the two components is satisfied: c1.alpha = phaseRatio * c2.alpha + phaseOffset
	KS_MechanicalComponent *c1;
	KS_MechanicalComponent *c2;

	//gradient of the energy...
	dVector dE_ds1, dE_ds2;
	//these are the blocks for the energy hessian
	Matrix ddE_ds1ds1, ddE_ds1ds2, ddE_ds2ds2, ddE_ds2ds1, tmpMat;

	//and these are the blocks for the constraint jacobian
	Matrix dCds1, dCds2;

	//scalar constraints
	dVector C;


	RBF1D_4Point01Interval phaseProfile;


	double get_f(double a);
	double get_dfda(double a);
	double get_dda_dfda(double a);

};

