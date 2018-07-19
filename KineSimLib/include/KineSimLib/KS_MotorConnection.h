#pragma once

#include "KS_Connection.h"
#include "KS_BindComponentsConnection.h"
#include "KS_P2PConstraint.h"
#include "KS_V2VConstraint.h"
#include "KS_Constraint.h"

class KS_MotorConnection : public KS_Connection {
	friend class KineSimApp;
	friend class KS_IKConstraintEnergy;
public:
	KS_MotorConnection(void);
	~KS_MotorConnection(void);
	
	virtual KS_MotorConnection* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut) const = 0;


	void setOffset(double offset) {m_offset = offset;}
	double getOffset() { return m_offset; }

	virtual void updateConnection()=0;

	virtual void computeddAE_ds_dp()=0;
	virtual bool isMotorized() { return isActuated; }


	MatrixNxM& getdAE_ds_dp1() { return  ddAE_ds_dp1; }
	MatrixNxM& getdAE_ds_dp2() { return  ddAE_ds_dp2; }


protected:
	//offset of the relative angle;
	double m_offset;
	bool isActuated;

	MatrixNxM ddAE_ds_dp1, ddAE_ds_dp2;

};

