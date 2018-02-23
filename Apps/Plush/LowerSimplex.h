#pragma once
// --
#include <PlushHelpers/helpers_star.h>
// --
#include "Node.h"
#include "SimMeshElement.h"

/*
 * Triangles if D() == 3, Line segments if D() == 2.
 */

class LowerSimplex {

public:
	LowerSimplex(vector<Node *>, SimMeshElement*);
	~LowerSimplex();

public:
	vector<Node *> nodes;
	SimMeshElement *higher_simplex;

public:
	int D();
	void draw(const dVector &y);

};

