#pragma once

#include "KS_Connection.h"
#include "KS_PhaseConstraint.h"
#include "KS_Ticker.h"
#include <MathLib/RBF1D_4Point01Interval.h>


class KS_PhaseDriverConnection : public KS_Connection{
	friend class KS_WMStaticBalanceObjective;
	friend class KS_WalkingMachinesOptimizer;
private:
	KS_Ticker* m_ticker;
	KS_PhaseConstraint* phaseConstraint;
	
	//we connect components directly to the ticker in two ways:
		//alpha = angleStep * tickerCount + offset
		//or
		//alpha = amplitude * sin(frequency * tickerCount + phaseOffset) + offset
	bool useSinuosoidalSignal, useNonConstantSpeedProfile;
	double angleStep, offset, amplitude, frequency, phaseOffset;
	RBF1D_4Point01Interval nonConstantSpeedProfile;

public:
	KS_PhaseDriverConnection(KS_Ticker* ticker);
	~KS_PhaseDriverConnection(void);

	virtual void connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut);

	virtual bool loadFromFile(FILE* f, KS_MechanicalAssembly* ma);
	virtual bool writeToFile(FILE* f);
	virtual KS_PhaseDriverConnection* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut, KS_Ticker* clock) const;

	virtual void addConstraintsToList(std::vector<KS_Constraint*>& constraints) {constraints.push_back(phaseConstraint);}

	virtual void updateConnection();
};

