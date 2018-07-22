#pragma once

#include "KS_Connection.h"
#include "KS_MotorConnection.h"
#include "KS_LockedComponentConstraint.h"
#include "KS_Constraint.h"
#include "KS_LoaderUtils.h"


class KS_MobileBaseToWorldConnection : public KS_MotorConnection{
private:
	
public:
	 KS_MobileBaseToWorldConnection(void);
	~KS_MobileBaseToWorldConnection(void);

	virtual void addConstraintsToList(std::vector<KS_Constraint*>& constraints) {constraints.push_back(lcCon);}

protected:

	KS_LockedComponentConstraint* lcCon;
};

