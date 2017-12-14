#pragma once

#include <array>

#include "FEMSimLib/CSTSimulationMesh2D.h"
#include "Mount.h"
#include "MeshObjective.h"
#include "NodePositionObjective.h"


class BenderSimulationMesh2D : public CSTSimulationMesh2D {

public:
	std::array<Mount*, 2> mounts;
	std::vector<MeshObjective *> objectives;

public:
	BenderSimulationMesh2D();
	~BenderSimulationMesh2D();

	// mesh manipulation
	void setMountedNode(int nodeID, const P3D & x0, int mountID);
	void setNodePositionObjective(int nodeID, const P3D & x0);
	void setNodeGlobalNodePositionObjective(dVector const & x);

	void clearObjectives();

	double computeO();
	double computeOofx(dVector const & x_in);

	void computeDoDx(dVector & dodx);

	// diagnostics
	double computeTargetPositionError();

	void drawSimulationMesh();

};
