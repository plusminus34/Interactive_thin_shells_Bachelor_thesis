#include "MathLib/P3D.h"


#include "MeshObjective.h"


class NodePositionObjective : public MeshObjective
{

	int nodeID;
	P3D targetPosition;

	virtual void addO(const dVector & x, const dVector & X, double & o) const;
	virtual void addDoDx(const dVector & x, const dVector & X, dVector & dodx) const;

};