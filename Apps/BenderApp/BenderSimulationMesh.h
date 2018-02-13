#pragma once

#include <array>
#include <type_traits>

#include "LazyFEMSimLib/CSTSimulationMesh2D.h"
#include "LazyFEMSimLib/CSTSimulationMesh3D.h"
#include "Mount.h"


#include "MeshObjective.h"
#include "NodePositionObjective.h"


template<int NDim>
class BenderSimulationMesh : public std::conditional<NDim == 2, CSTSimulationMesh2D, CSTSimulationMesh3D>::type {

public:
	std::vector<Mount*> mounts;
	std::vector<MeshObjective *> objectives;

public:
	BenderSimulationMesh();
	~BenderSimulationMesh();

	// mesh manipulation
	template<typename TMount> 
	void addMount(ParameterSet * parameters) {mounts.push_back(new TMount(parameters));}
	void removeMount(int mountID);

	void setMountedNode(int nodeID, const P3D & x0, int mountID);
	void unmountNode(int nodeID, int mountID);

	int getSelectedNodePositionObjectiveID(Ray const & ray);

	int setNodePositionObjective(int nodeID, const P3D & x0);
	int setNodePositionObjectiveNoDuplicate(int nodeID, const P3D & x0);
	void setNodeGlobalNodePositionObjective(dVector const & x);

	//void scaleAll(double s);
	//void moveAll(V3D v);
	

	int getMountIdOfNode(int nodeID);

	void clearObjectives();
	void clearNodePositionObjectives();
	void removeNodePositionObjectivesOfNode(int nodeID);
	

	double computeO();
	double computeOofx(dVector const & x_in);

	void computeDoDx(dVector & dodx);

	// diagnostics
	double computeTargetPositionError();

	void drawSimulationMesh();

};
