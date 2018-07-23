#pragma once

#include "KS_Constraint.h"
#include "KS_MechanicalComponent.h"

class KS_LockedComponentConstraint : public KS_Constraint{
	friend class KS_xMobileBaseToWorldConnection;
	friend class KS_MobileBaseToWorldConnection;
public:
	KS_LockedComponentConstraint(KS_MechanicalComponent *p_c, double p_alphaD, bool p_freezeAlpha, double p_betaD, bool p_freezeBeta, double p_gammaD, bool p_freezeGamma, double p_pxD, bool p_freezePx, double p_pyD, bool p_freezePy, double p_pzD, bool p_freezePz);
	virtual KS_LockedComponentConstraint* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut)const;
	~KS_LockedComponentConstraint(void);

	virtual double getEnergy();
	virtual void computeEnergyGradient();
	virtual void computeEnergyHessian();

	virtual int getNumberOfAffectedComponents() {return 1;}
	virtual KS_MechanicalComponent* getIthAffectedComponent(int i){return c;}

	//for testing purposes only...
	virtual dVector* get_dE_dsi(int i) {return &dE_ds;}
	virtual MatrixNxM* get_ddE_dsidsj(int i, int j) {return &ddE_dsds;}

	void setTargetPosition(const P3D& pos);


	//each constraint is composed of several scalar constraints - this is how many
	virtual int getConstraintCount();


protected:
	//we need to ensure that the state of c has the desired value
	KS_MechanicalComponent *c;
	double alphaD, betaD, gammaD, pxD, pyD, pzD;
	int freezeAlpha, freezeBeta, freezeGamma, freezePx, freezePy, freezePz;
	//tmp variables
	dVector dE_ds;
	MatrixNxM ddE_dsds;

	double anglesWeight;
	double positionWeight;
	V3D PositionWeight;
};

