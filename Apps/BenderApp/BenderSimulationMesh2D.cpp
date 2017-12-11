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
	for(MeshObjective const * obj : objectives)
	{
		obj->addDoDx(x, X, dodx);
	}
}