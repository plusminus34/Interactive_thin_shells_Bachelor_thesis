#pragma once
//#include <MathLib/RBF1D.h>
#include "KS_Connection.h"
#include "KS_BindComponentsConnection.h"
//#include "KS_PhaseToPhaseConstraint.h"
#include "KS_P2PConstraint.h"
#include "KS_V2VConstraint.h"
#include "KS_Constraint.h"
#include <MathLib/Trajectory.h>


class KS_rMotorConnection : public KS_Connection {
public:
	KS_rMotorConnection(void);
	~KS_rMotorConnection(void);
	virtual void connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut);
	
	void setOffset(double offset) {m_offset = offset;}
	double getOffset() { return m_offset; }

	bool isOn(){return isActivated;}
	//void activate(){isActivated=true; m_compOut->getMesh(m_pin)->setColour(1.0,0.0,0.0,1.0);}
	//void desactivate(){isActivated=false; m_compOut->getMesh(m_pin)->setColour(0.5,0.5,0.5,1.0);}

	bool loadFromFile(FILE* f, KS_MechanicalAssembly* ma);
	virtual bool writeToFile(FILE* f);
	virtual KS_rMotorConnection* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut) const;
	virtual void addConstraintsToList(std::vector<KS_Constraint*>& constraints);

	virtual void updateConnection();

	virtual bool isMotorized() { return isActivated; }


protected:
	//offset of the relqtive angle;
	double m_offset;
		
	Trajectory1D motorAngle;

	bool isActivated;
	
	P3D pOnC1, pOnC2;
	V3D nOnC1, nOnC2, vOnC1, vOnC2;

	KS_V2VConstraint* rMotorAngleConstraint;
	KS_P2PConstraint* pt2ptConstraint;
	KS_V2VConstraint* v2vConstraint;

};

