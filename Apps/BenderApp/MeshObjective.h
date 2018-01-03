#include <vector>
#pragma once

#include "FEMSimLib/SimulationMesh.h"


class MeshObjective {

public:
	//SimulationMesh * mesh;
public:
	//MeshObjective() {};
	//MeshObjective(SimulationMesh * mesh) : mesh(mesh) {};

	virtual void addO(const dVector & x, const dVector & X, double & o) = 0;
	virtual void addDoDx(const dVector & x, const dVector & X, dVector & dodx) = 0;

	virtual void addError(const dVector & x, double & e) = 0;

};