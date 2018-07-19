#pragma once

#include "KS_MobileBaseToWorldConnection.h"
#include "KS_LockedComponentConstraint.h"
#include "KS_Constraint.h"
#include "KS_LoaderUtils.h"


class KS_xMobileBaseToWorldConnection : public KS_MobileBaseToWorldConnection {
private:
	
public:
	KS_xMobileBaseToWorldConnection(void);
	~KS_xMobileBaseToWorldConnection(void);

	virtual bool loadFromFile(FILE* f, KS_MechanicalAssembly* ma);
	virtual bool writeToFile(FILE* f);
	virtual KS_xMobileBaseToWorldConnection* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut) const;
	virtual void connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut);

	virtual void updateConnection();

	virtual void computeddAE_ds_dp();

protected:

};

