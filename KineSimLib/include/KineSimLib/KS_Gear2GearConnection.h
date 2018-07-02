#pragma once
#include "KS_Phase2PhaseConnection.h"

class KS_Gear;

class KS_Gear2GearConnection : public KS_Phase2PhaseConnection {
public:
	KS_Gear2GearConnection(void);
	virtual void connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut);
	~KS_Gear2GearConnection(void);


	virtual bool loadFromFile(FILE* f, KS_MechanicalAssembly* ma);
	virtual bool writeToFile(FILE* f);
	virtual KS_Gear2GearConnection* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut, KS_Ticker* clock) const;

protected:
	KS_Gear* m_pGearIn;
	KS_Gear* m_pGearOut;

};

