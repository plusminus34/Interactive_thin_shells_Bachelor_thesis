#include "BenderSimulationMesh.h"
#include "RotationMount2D.h"
#include "MountedPointSpring2D.h"

template<int NDim>
BenderSimulationMesh<NDim>::BenderSimulationMesh()
{
}

template<int NDim>
BenderSimulationMesh<NDim>::~BenderSimulationMesh()
{
	for(Mount* m : mounts) {
		delete m;
	}
	for(MeshObjective* o : objectives) {
		delete o;
	}
}

template<int NDim>
template<typename TMount>
void BenderSimulationMesh<NDim>::addMount() 
{
	mounts.push_back(new TMount);
}

template<int NDim>
void BenderSimulationMesh<NDim>::removeMount(int mountID) 
{

	if(mountID < 0 || mountID >= mounts.size()) {
		return;
	}

	// remove all pins to the mount
	for(int i = pinnedNodeElements.size()-1; i >= 0; --i) {
		if(dynamic_cast<MountedPointSpring<nDim> *>(pinnedNodeElements[i])->mount == mounts[mountID]) {
			delete pinnedNodeElements[i];
			pinnedNodeElements.erase(pinnedNodeElements.begin()+i);
		}
	}

	// remove the mount
	delete mounts[mountID];
	mounts.erase(mounts.begin()+mountID);
}

template<int NDim>
void BenderSimulationMesh<NDim>::setMountedNode(int nodeID, const P3D & x0, int mountID)
{
	P3D rp = x0;
	//rp[2] = 0;
	pinnedNodeElements.push_back(new MountedPointSpring<nDim>(this, nodes[nodeID], rp, mounts[mountID] ));
}

template<int NDim>
void BenderSimulationMesh<NDim>::unmountNode(int nodeID, int mountID)
{
	Node * node = nodes[nodeID];
	Mount * mount = mounts[mountID];
	for(int i = pinnedNodeElements.size()-1; i >= 0; --i) {
		MountedPointSpring<NDim> * pin = dynamic_cast<MountedPointSpring<nDim> *>(pinnedNodeElements[i]);
		if(pin->node == node && pin->mount == mount) {
			delete pinnedNodeElements[i];
			pinnedNodeElements.erase(pinnedNodeElements.begin()+i);
		}
	}
}

template<int NDim>
void BenderSimulationMesh<NDim>::setNodePositionObjective(int nodeID, const P3D & x0) 
{
	objectives.push_back(new NodePositionObjective(nodes[nodeID], x0));
}

template<int NDim>
void BenderSimulationMesh<NDim>::setNodeGlobalNodePositionObjective(dVector const & x)
{
	clearObjectives();

	for(int i = 0; i < nodes.size(); ++i) {
		setNodePositionObjective(i, nodes[i]->getCoordinates(x));
	}
}

template<int NDim>
int BenderSimulationMesh<NDim>::getMountIdOfNode(int nodeID) {
	Node * node = nodes[nodeID];
	// search pins for node id
	int pinnedNodeElementID = -1;
	for(int j = 0; j < pinnedNodeElements.size(); ++j) {
		BaseEnergyUnit * pin = pinnedNodeElements[j];
		if(dynamic_cast<MountedPointSpring<NDim> *>(pin) {	// check if the pinned node is pinned to a mount
			if(dynamic_cast<MountedPointSpring<NDim> *>(pin)->node == node) {
				pinnedNodeElementID = j;
				break;
			}
		}
	}
	// search for mount id
	if(pinnedNodeElementID >=0) {
		for(int j = 0; j < mounts.size(); ++j) {
			if(mounts[j] == dynamic_cast<MountedPointSpring<NDim> *>(pinnedNodeElements[pinnedNodeElementID])->mount) {
				return(j);
			}
		}
	}
	else {
		return(-1);
	}

	return(-1);
}

template<int NDim>
void BenderSimulationMesh<NDim>::clearObjectives()
{
	for(MeshObjective * obj: objectives) {
		delete obj;
	}
	objectives.resize(0);
}

template<int NDim>
double BenderSimulationMesh<NDim>::computeO()
{
	double o = 0;
	for(MeshObjective * obj : objectives)
	{
		obj->addO(x, X, o);
	}
	return(o);
}

template<int NDim>
double BenderSimulationMesh<NDim>::computeOofx(dVector const & x_in) {
	double o = 0;
	for(MeshObjective * obj : objectives)
	{
		obj->addO(x_in, X, o);
	}
	return(o);
}


template<int NDim>
void BenderSimulationMesh<NDim>::computeDoDx(dVector & dodx)
{
	dodx.resize(x.size());
	dodx.setZero();
	for(MeshObjective * obj : objectives)
	{
		obj->addDoDx(x, X, dodx);
	}
}

template<int NDim>
double BenderSimulationMesh<NDim>::computeTargetPositionError()
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


template<int NDim>
void BenderSimulationMesh<NDim>::drawSimulationMesh() 
{
	SimulationMesh::drawSimulationMesh();

	//for(MeshObjective * obj : objectives) {
	//	dynamic_cast<NodePositionObjective *>(obj)->draw(x);
	//}
}