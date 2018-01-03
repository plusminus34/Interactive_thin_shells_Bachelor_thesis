#include "BenderSimulationMesh2D.h"
#include "RotationMount.h"
#include "MountedPointSpring2D.h"


BenderSimulationMesh2D::BenderSimulationMesh2D()
{
	/*
	int parameterStartIndex = 0;
	for(Mount*& m : mounts) {
		m = new RotationMount(0.0, 0.0, 0.0, parameterStartIndex);
		parameterStartIndex += 3;
	}
	*/
}

BenderSimulationMesh2D::~BenderSimulationMesh2D()
{
	for(Mount* m : mounts) {
		delete m;
	}
}

void BenderSimulationMesh2D::addRotationMount() 
{
	mounts.push_back(new RotationMount);
}

void BenderSimulationMesh2D::removeMount(int mountID) 
{

	if(mountID < 0 || mountID >= mounts.size()) {
		return;
	}

	// remove all pins to the mount
	for(int i = pinnedNodeElements.size()-1; i >= 0; --i) {
		if(dynamic_cast<MountedPointSpring2D *>(pinnedNodeElements[i])->mount == mounts[mountID]) {
			delete pinnedNodeElements[i];
			pinnedNodeElements.erase(pinnedNodeElements.begin()+i);
		}
	}

	// remove the mount
	delete mounts[mountID];
	mounts.erase(mounts.begin()+mountID);
}


void BenderSimulationMesh2D::setMountedNode(int nodeID, const P3D & x0, int mountID)
{
	P3D rp = x0;
	rp[2] = 0;
	pinnedNodeElements.push_back(new MountedPointSpring2D(this, nodes[nodeID], rp, mounts[mountID] ));
}

void BenderSimulationMesh2D::unmountNode(int nodeID, int mountID)
{
	Node * node = nodes[nodeID];
	Mount * mount = mounts[mountID];
	for(int i = pinnedNodeElements.size()-1; i >= 0; --i) {
		MountedPointSpring2D * pin = dynamic_cast<MountedPointSpring2D *>(pinnedNodeElements[i]);
		if(pin->node == node && pin->mount == mount) {
			delete pinnedNodeElements[i];
			pinnedNodeElements.erase(pinnedNodeElements.begin()+i);
		}
	}
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


int BenderSimulationMesh2D::getMountIdOfNode(int nodeID) {
	Node * node = nodes[nodeID];
	// search pins for node id
	int pinnedNodeElementID = -1;
	for(int j = 0; j < pinnedNodeElements.size(); ++j) {
		if(dynamic_cast<FixedPointSpring2D *>(pinnedNodeElements[j])->node == node) {
			pinnedNodeElementID = j;
			break;
		}
	}
	// search for mount id
	if(pinnedNodeElementID >=0) {
		for(int j = 0; j < mounts.size(); ++j) {
			if(mounts[j] == dynamic_cast<MountedPointSpring2D *>(pinnedNodeElements[pinnedNodeElementID])->mount) {
				return(j);
			}
		}
	}
	else {
		return(-1);
	}

	return(-1);
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
	for(MeshObjective * obj : objectives)
	{
		obj->addO(x, X, o);
	}
	return(o);
}

double BenderSimulationMesh2D::computeOofx(dVector const & x_in) {
	double o = 0;
	for(MeshObjective * obj : objectives)
	{
		obj->addO(x_in, X, o);
	}
	return(o);
}



void BenderSimulationMesh2D::computeDoDx(dVector & dodx)
{
	dodx.resize(x.size());
	dodx.setZero();
	for(MeshObjective * obj : objectives)
	{
		obj->addDoDx(x, X, dodx);
	}
}


double BenderSimulationMesh2D::computeTargetPositionError()
{
	double e = 0;
	int n_obj = 0;
	for(MeshObjective * obj : objectives)
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

	//for(MeshObjective * obj : objectives) {
	//	dynamic_cast<NodePositionObjective *>(obj)->draw(x);
	//}
}