H#pragma once

#include "KS_Connection.h"
#include "KS_PointOnLineConstraint.h"
#include "KS_V2VConstraint.h"
#include "KS_Constraint.h"

/**
	this type of connection is used to ensure that one point on the in component is restricted to a line on the out component. In addition,
	a vector-to-vector constraint can be added to additionaly place constraints on the relative orientation between two objects.
*/

class KS_PointOnLineConnection : public KS_Connection{
private:
	Point3d xOnC1, pOnC2;
	Vector3d lOnC2, nOnC1, nOnC2;
	KS_PointOnLineConstraint* pOnlConstraint;
	KS_V2VConstraint* v2vConstraint;
	bool constrainedNormal;
public:
	KS_PointOnLineConnection(void);
	~KS_PointOnLineConnection(void);

	virtual bool loadFromFile(FILE* f, KS_MechanicalAssembly* ma);
	virtual bool writeToFile(FILE* f);
	virtual KS_PointOnLineConnection* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut, KS_Ticker* clock) const;
	virtual void connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut);

	virtual void addConstraintsToList(std::vector<KS_Constraint*>& constraints) {
		constraints.push_back(pOnlConstraint);
		if (v2vConstraint != NULL)
			constraints.push_back(v2vConstraint);
	}
};

