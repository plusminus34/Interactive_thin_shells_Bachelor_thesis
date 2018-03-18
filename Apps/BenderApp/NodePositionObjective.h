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

	virtual void addO(const dVector & x, const dVector & X, double & o);
	virtual void addDoDx(const dVector & x, const dVector & X, dVector & dodx);

	virtual void addError(const dVector & x, double & e);

	void draw(dVector const & x, HighlightLevel level = HighlightLevel::NONE);

};