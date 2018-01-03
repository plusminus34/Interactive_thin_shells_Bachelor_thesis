#pragma once

#include "MathLib/P3D.h"


#include "MeshObjective.h"

class LinePositionObjective : public MeshObjective
{
public:
	Node * node1;
	Node * node2;
	P3D targetPosition;

public:
	LinePositionObjective(Node * node1, Node * node2, P3D const & targetPosition);

	virtual void addO(const dVector & x, const dVector & X, double & o) const;
	virtual void addDoDx(const dVector & x, const dVector & X, dVector & dodx) const;

	virtual void addError(const dVector & x, double & e) const;

	void draw(dVector const & x);

};