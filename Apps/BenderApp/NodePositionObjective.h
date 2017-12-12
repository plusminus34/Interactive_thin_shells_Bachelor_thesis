#pragma once

#include "MathLib/P3D.h"


#include "MeshObjective.h"


class NodePositionObjective : public MeshObjective
{
public:
	Node * node;
	P3D targetPosition;

public:
	NodePositionObjective(Node * node, P3D const & targetPosition);

	virtual void addO(const dVector & x, const dVector & X, double & o) const;
	virtual void addDoDx(const dVector & x, const dVector & X, dVector & dodx) const;

};