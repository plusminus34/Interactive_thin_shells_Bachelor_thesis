#pragma once

#include "KS_Constraint.h"
#include "KS_MechanicalComponent.h"

/*
	ensures that point x (in world coords) on component 1 lies on the line p+t*l, where p and l are a point and a vector on component 2
*/
class KS_PointOnLineConstraint : public KS_Constraint{
private:
	P3D x;									//on c1
	P3D p;									//on c2
	V3D l;									//on c2
	KS_MechanicalComponent *c1, *c2;

	V3D getErrorVector();
	P3D wx, wp;
	V3D wl;

	//tmp variables
	Matrix dwx_ds1, dwp_ds2, dwl_ds2;
	dVector tmpV, tmpY;
	Matrix wlCross, wvCross, tmpCross;

	//these are the blocks for the energy hessian
	Matrix ddE_ds1ds1, ddE_ds1ds2, ddE_ds2ds2, ddE_ds2ds1, dy_ds1, dy_ds2, tmpMat;
	dVector dE_ds1, dE_ds2, tmpE;

	//and these are the blocks for the constraint jacobian
	Matrix dCds1, dCds2;

	//scalar constraints
	dVector C;
public:
	KS_PointOnLineConstraint(const P3D& xOnC1_, KS_MechanicalComponent* c1_, const P3D& pOnC2_, const V3D& lOnC2_, KS_MechanicalComponent* c2_);
	virtual KS_PointOnLineConstraint* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut)const;
	~KS_PointOnLineConstraint(void);

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




};

