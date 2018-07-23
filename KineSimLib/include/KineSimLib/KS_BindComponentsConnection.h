#pragma once

#include "KS_Connection.h"
#include "KS_P2PConstraint.h"
#include "KS_V2VConstraint.h"
#include "KS_Constraint.h"

/**
	this type of connection only allows 1DOF relative motion between two components. It creates a point-to-point constraint
	and, depending on the option passed in, one or more vector-to-vector and point to point constraints.

	By default, this connection allows the components to rotate relative to each other about the constrained vector axis. If this
	is not desireable (i.e. the components should be welded together), then use the weldComponents flag, which freezes the
	relative orientation between the two components
*/

class KS_BindComponentsConnection : public KS_Connection{
private:
	P3D pOnC1, pOnC2;
	V3D nOnC1, nOnC2;
	KS_P2PConstraint* p2pConstraint;
	KS_V2VConstraint* v2vConstraint;
	KS_V2VConstraint* v2vConstraintForWelding;
	bool weldComponents;
	bool allowArbitraryRelativeRotation;

public:
	KS_BindComponentsConnection(void);
	~KS_BindComponentsConnection(void);

	virtual bool loadFromFile(FILE* f, KS_MechanicalAssembly* ma);
	virtual bool writeToFile(FILE* f);
	virtual KS_BindComponentsConnection* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut) const;
	virtual void connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut);

	void setWeldComponentsFlag(bool flag){
		weldComponents = flag;
	}

	virtual void addConstraintsToList(std::vector<KS_Constraint*>& constraints) {
		constraints.push_back(p2pConstraint);
		if (v2vConstraint != NULL)
			constraints.push_back(v2vConstraint);
		if (v2vConstraintForWelding!=NULL)
			constraints.push_back(v2vConstraintForWelding);
	}

	void setArbitraryRelativeRotation(bool allow) { allowArbitraryRelativeRotation = allow; }

	P3D getWorldCoordsPinPosition(){
		return (this->m_compIn->get_w(pOnC1) + this->m_compOut->get_w(pOnC2))/2.0;
	}

};

