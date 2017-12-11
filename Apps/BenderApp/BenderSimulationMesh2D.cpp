#include "BenderSimulationMesh2D.h"
#include "RotationMount.h"
#include "MountedPointSpring2D.h"


BenderSimulationMesh2D::BenderSimulationMesh2D()
{
	for(Mount*& m : mounts) {
		m = new RotationMount(0.0, 0.0, 0.0);
		std::cout << "just breaking..." << std::endl;
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

