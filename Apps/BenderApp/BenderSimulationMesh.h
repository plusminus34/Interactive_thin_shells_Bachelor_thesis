#pragma once

#include <array>
#include <type_traits>

#include "LazyFEMSimLib/CSTSimulationMesh2D.h"
#include "LazyFEMSimLib/CSTSimulationMesh3D.h"
#include "Mount.h"


#include "MeshObjective.h"
#include "NodePositionObjective.h"



class MeshPositionRegularizer;


template<int NDim>
class BenderSimulationMesh : public std::conditional<NDim == 2, CSTSimulationMesh2D, CSTSimulationMesh3D>::type {

public:
	std::vector<Mount*> mounts;
	std::vector<MeshObjective *> objectives;
	// regularizers for top-level optimization
	MeshPositionRegularizer meshPositionRegularizer;
	MeshEnergyRegularizer meshEnergyRegularizer;

public:
	BenderSimulationMesh();
	~BenderSimulationMesh();

	// mesh manipulation
	template<typename TMount> 
	void addMount(ParameterSet * parameters) {mounts.push_back(new TMount(parameters));}
	template<typename TMount> 
	void addMount(TMount * mount) {mounts.push_back(mount);}
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
	void removeObjective(int objectiveID);
	void removeNodePositionObjectivesOfNode(int nodeID);

	

	double computeO();
	//double computeOofx(dVector const & x_in);

	void computeDoDx(dVector & dodx);

	// diagnostics
	double computeTargetPositionError();

	void drawSimulationMesh(bool drawSurface, V3D color);

};




class MeshPositionRegularizer : public MeshObjective {
public: 
	double r = 0;

public:

	virtual void addO(const dVector & x, const dVector & X, double & o) ;
	virtual void addDoDx(const dVector & x, const dVector & X, dVector & dodx);


	virtual void addError(const dVector & x, double & e) {}
	virtual void draw(dVector const & x, HighlightLevel level = HighlightLevel::NONE) {}
};



class MeshEnergyRegularizer : public MeshObjective {
public: 
	double r = 0;

	SimulationMesh * femMesh;

public:
	MeshEnergyRegularizer(SimulationMesh* femMesh) : femMesh(femMesh) {}

	virtual void addO(const dVector & x, const dVector & X, double & o) ;
	virtual void addDoDx(const dVector & x, const dVector & X, dVector & dodx);


	virtual void addError(const dVector & x, double & e) {}
	virtual void draw(dVector const & x, HighlightLevel level = HighlightLevel::NONE) {}
};