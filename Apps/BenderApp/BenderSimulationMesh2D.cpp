#include "BenderSimulationMesh2D.h"
#include "RotationMount.h"
#include "MountedPointSpring2D.h"


BenderSimulationMesh2D::BenderSimulationMesh2D()
{
	int parameterStartIndex = 0;
	for(Mount*& m : mounts) {
		m = new RotationMount(0.0, 0.0, 0.0, parameterStartIndex);
		parameterStartIndex += 3;
	}
}

BenderSimulationMesh2D::~BenderSimulationMesh2D()
{
	for(Mount* m : mounts) {
		delete m;
	}
}

void BenderSimulationMesh2D::setMountedNode(int nodeID, const P3D & x0, int mountID)
{
	P3D rp = x0;
	rp[2] = 0;
	pinnedNodeElements.push_back(new MountedPointSpring2D(this, nodes[nodeID], rp, mounts[mountID] ));
}

void BenderSimulationMesh2D::setNodePositionObjective(int nodeID, const P3D & x0) 
{
	objectives.push_back(new NodePositionObjective(nodes[nodeID], x0));
}


void BenderSimulationMesh2D::setNodeGlobalNodePositionObjective(dVector const & x)
{
	clearObjectives();

	for(int i = 0; i < nodes.size(); ++i) {
		setNodePositionObjective(i, nodes[i]->getCoordinates(x));
	}
}


void BenderSimulationMesh2D::clearObjectives()
{
	for(MeshObjective * obj: objectives) {
		delete obj;
	}
	objectives.resize(0);
}

double BenderSimulationMesh2D::computeO()
{
	double o = 0;
	for(MeshObjective const * obj : objectives)
	{
		obj->addO(x, X, o);
	}
	return(o);
}

double BenderSimulationMesh2D::computeOofx(dVector const & x_in) {
	double o = 0;
	for(MeshObjective const * obj : objectives)
	{
		obj->addO(x_in, X, o);
	}
	return(o);
}



void BenderSimulationMesh2D::computeDoDx(dVector & dodx)
{
	dodx.resize(x.size());
	dodx.setZero();
	for(MeshObjective const * obj : objectives)
	{
		obj->addDoDx(x, X, dodx);
	}
}


double BenderSimulationMesh2D::computeTargetPositionError()
{
	double e = 0;
	int n_obj = 0;
	for(MeshObjective const * obj : objectives)
	{
		obj->addError(x, e);
		++n_obj;
	}
	double e_rel = e / static_cast<double>(n_obj);
	return(e_rel);
}



void BenderSimulationMesh2D::drawSimulationMesh() 
{
	SimulationMesh::drawSimulationMesh();

	for(MeshObjective * obj : objectives) {
		dynamic_cast<NodePositionObjective *>(obj)->draw(x);
	}
}