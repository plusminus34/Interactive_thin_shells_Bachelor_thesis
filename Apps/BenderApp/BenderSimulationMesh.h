#pragma once

#include <array>
#include <type_traits>

#include "FEMSimLib/CSTSimulationMesh2D.h"
#include "FEMSimLib/CSTSimulationMesh3D.h"
#include "Mount.h"


#include "MeshObjective.h"
#include "NodePositionObjective.h"


template<int NDim>
class BenderSimulationMesh : public std::conditional<NDim == 2, CSTSimulationMesh2D, CSTSimulationMesh3D> {

public:
	std::vector<Mount*> mounts;
	std::vector<MeshObjective *> objectives;

public:
	BenderSimulationMesh();
	~BenderSimulationMesh();

	// mesh manipulation
	template<typename TMount> void addMount();
	void removeMount(int mountID);

	void setMountedNode(int nodeID, const P3D & x0, int mountID);
	void unmountNode(int nodeID, int mountID);

	void setNodePositionObjective(int nodeID, const P3D & x0);
	void setNodeGlobalNodePositionObjective(dVector const & x);

	

	int getMountIdOfNode(int nodeID);

	void clearObjectives();

	double computeO();
	double computeOofx(dVector const & x_in);

	void computeDoDx(dVector & dodx);

	// diagnostics
	double computeTargetPositionError();

	void drawSimulationMesh();

};
