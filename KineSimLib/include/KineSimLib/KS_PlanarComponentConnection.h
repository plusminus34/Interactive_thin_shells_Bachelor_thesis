#pragma once

#include "KS_Connection.h"
#include "KS_LockedComponentConstraint.h"
#include "KS_Constraint.h"
#include "KS_LoaderUtils.h"


//this connection only has a component out, which is locked in world space (i.e. no DOF). This is useful when building the walls of a 
//mechanical assembly, for instance. Shafts, bars, etc can then be attached (hinge, point-on-line, lockedToComponent, etc) to these components
class KS_PlanarComponentConnection : public KS_Connection{
private:
	KS_LockedComponentConstraint* lcCon;
public:
	KS_PlanarComponentConnection(void);
	~KS_PlanarComponentConnection(void);

	virtual bool loadFromFile(FILE* f, KS_MechanicalAssembly* ma);
	virtual bool writeToFile(FILE* f);
	virtual KS_PlanarComponentConnection* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut) const;
	virtual void connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut);

	virtual void addConstraintsToList(std::vector<KS_Constraint*>& constraints) {constraints.push_back(lcCon);}

};

