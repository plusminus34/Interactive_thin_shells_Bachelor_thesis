#include <vector>
#pragma once

#include "LazyFEMSimLib/SimulationMesh.h"


class MeshObjective {

public:
	//SimulationMesh * mesh;
	
	enum class HighlightLevel {NONE, HOVERED, SELECTED, HIDE};

public:
	//MeshObjective() {};
	//MeshObjective(SimulationMesh * mesh) : mesh(mesh) {};


	virtual void addO(const dVector & x, const dVector & X, double & o) = 0;
	virtual void addDoDx(const dVector & x, const dVector & X, dVector & dodx) = 0;

	virtual void addError(const dVector & x, double & e) = 0;

	virtual void draw(dVector const & x, HighlightLevel level = HighlightLevel::NONE) = 0;

};