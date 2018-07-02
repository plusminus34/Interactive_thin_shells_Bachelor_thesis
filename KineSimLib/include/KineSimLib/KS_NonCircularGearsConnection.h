#pragma once
#include "KS_Connection.h"
#include "KS_NonCircularGear.h"
#include <MathLib/RBF1D_4Point01Interval.h>
#include "KS_NonLinearPhaseConstraint.h"
#include "KS_V2VConstraint.h"



class KS_NonCircularGearsConnection : public KS_Connection {
public:
	KS_NonCircularGearsConnection(void);
	virtual void connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut);
	~KS_NonCircularGearsConnection(void);

	virtual void addConstraintsToList(std::vector<KS_Constraint*>& constraints) {constraints.push_back(nonLinearPhaseConstraint); constraints.push_back(v2vConstraint);}

	virtual bool loadFromFile(FILE* f, KS_MechanicalAssembly* ma);
	virtual bool writeToFile(FILE* f);
	virtual KS_NonCircularGearsConnection* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut, KS_Ticker* clock) const;

	//the profile that relates the phase of the two gears is specified by four numbers, which are used in the RBF interpolation
	double s1, s2, s3, s4;
	RBF1D_4Point01Interval nonConstantSpeedProfile;

protected:
	KS_NonCircularGear* m_pGearIn;
	KS_NonCircularGear* m_pGearOut;

	KS_NonLinearPhaseConstraint* nonLinearPhaseConstraint;
	KS_V2VConstraint* v2vConstraint;
};

