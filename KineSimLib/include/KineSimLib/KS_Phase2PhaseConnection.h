#pragma once
#include "KS_Connection.h"
#include "KS_PhaseToPhaseConstraint.h"


class KS_Phase2PhaseConnection : public KS_Connection {
public:
	KS_Phase2PhaseConnection(void);
	~KS_Phase2PhaseConnection(void);
	virtual void connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut);
	void setSign(bool positive) { m_sign = positive?1.0:-1.0; }
	void setRatio(double ratio) { m_ratio = ratio; }
	void setOffset(double offset) { m_offset = offset; }

	virtual bool loadFromFile(FILE* f, KS_MechanicalAssembly* ma);
	virtual bool writeToFile(FILE* f);
	virtual KS_Phase2PhaseConnection* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut, KS_Ticker* clock) const;
	virtual void addConstraintsToList(std::vector<KS_Constraint*>& constraints) {constraints.push_back(p2pConstraint);}

protected:
	//1.0 if direction of out is same as in, -1.0 otherwise
	double m_sign;
	//a unit step of in will cause m_sign*m_ratio steps of out
	double m_ratio;
	//an offset phase
	double m_offset;

	KS_PhaseToPhaseConstraint* p2pConstraint;
	void createPhaseToPhaseConstraint();
};

