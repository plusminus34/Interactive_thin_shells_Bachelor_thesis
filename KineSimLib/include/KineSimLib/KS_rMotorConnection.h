#pragma once
//#include <MathLib/RBF1D.h>
#include "KS_Connection.h"
#include "KS_BindComponentsConnection.h"
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

	//bool isOn(){return isActivated;}

	bool loadFromFile(FILE* f, KS_MechanicalAssembly* ma);
	virtual bool writeToFile(FILE* f);
	virtual KS_rMotorConnection* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut) const;
	virtual void addConstraintsToList(std::vector<KS_Constraint*>& constraints);

	virtual void updateConnection();

	//virtual bool isMotorized() { return isActuated; }


protected:
	//offset of the relative angle;
	double m_offset;
		
	//bool isActivated;
	
	P3D pOnC1, pOnC2;
	V3D nOnC1, nOnC2, vOnC1, vOnC2;// firs two correspond to the rotation axes and the last two should correspond to vectors normal to the rotation axes

	KS_V2VConstraint* rMotorAngleConstraint;
	KS_P2PConstraint* pt2ptConstraint;
	KS_V2VConstraint* v2vConstraint;

};

