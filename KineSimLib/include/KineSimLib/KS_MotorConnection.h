#pragma once
#include <MathLib/RBF1D.h>
#include "KS_Connection.h"
#include "KS_BindComponentsConnection.h"
#include "KS_PhaseToPhaseConstraint.h"
#include "KS_P2PConstraint.h"
#include "KS_V2VConstraint.h"
#include "KS_Constraint.h"
#include "KS_Ticker.h"
#include <MathLib/Trajectory.h>


class KS_MotorConnection : public KS_Connection {
public:
	KS_MotorConnection(void);
	~KS_MotorConnection(void);
	virtual void connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut);
	void setAmplitude(double angle) { m_amplitude = angle;}
	void setFrequency(double f) { m_frequency = f;}
	void setOffset(double off) {m_offset = off;}
	double getPhase(double t, double tMax);
	double getCurrentPhase(){return m_phase;}
	void setPhase(double ph){m_phase = ph; p2pConstraint->setOffset(m_phase);}
	void linkWithTicker(KS_Ticker* ticker){m_ticker=ticker;}
	void setRBF(RBF1D* rbf, int frames){m_rbf=rbf; useRBF=true; m_frames=frames;}
	double getPhaseConstraintValue();
	int getPhaseConstraintIndex(){if(p2pConstraint!=NULL) return p2pConstraint->getConstraintStartIndex(); else return -1;}
	virtual void updateConnection();

	bool isOn(){return isActivated;}
	void activate(){isActivated=true; m_compOut->getMesh(m_pin)->setColour(1.0,0.0,0.0,1.0);}
	void desactivate(){isActivated=false; m_compOut->getMesh(m_pin)->setColour(0.5,0.5,0.5,1.0);}

	bool loadFromFile(FILE* f, KS_MechanicalAssembly* ma);
	virtual bool writeToFile(FILE* f);
	virtual KS_MotorConnection* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut, KS_Ticker* clock) const;
	virtual void addConstraintsToList(std::vector<KS_Constraint*>& constraints);

protected:
	//current phase
	double m_phase;
	//max_angle
	double m_amplitude;
	//frequency of the movement
	double m_frequency;
	//when using RBF motor angle profile, use this to speed up or slow down the whole motion
	double m_stepScale;
	//offset of the relqtive angle;
	double m_offset, m_sinPhaseOffset;
	int m_frames;
	double m_rate;

	KS_Ticker* m_ticker;
	RBF1D* m_rbf;

	Trajectory1D motorPhaseAngle;

	bool isActivated;
	bool useRBF;

	Point3d pOnC1, pOnC2;
	Vector3d nOnC1, nOnC2;

	KS_PhaseToPhaseConstraint* p2pConstraint;
	KS_P2PConstraint* pt2ptConstraint;
	KS_V2VConstraint* v2vConstraint;

	bool useCSP;
};

