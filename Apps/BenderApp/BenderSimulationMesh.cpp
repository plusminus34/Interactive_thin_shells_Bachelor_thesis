#include "BenderSimulationMesh.h"
#include "RotationMount2D.h"
#include "MountedPointSpring.h"


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
void BenderSimulationMesh<NDim>::removeMount(int mountID) 
{

	if(mountID < 0 || mountID >= mounts.size()) {
		return;
	}

	// remove all pins to the mount
	for(int i = this->pinnedNodeElements.size()-1; i >= 0; --i) {
		if(dynamic_cast<MountedPointSpring<NDim> *>(this->pinnedNodeElements[i])->mount == mounts[mountID]) {
			delete this->pinnedNodeElements[i];
			this->pinnedNodeElements.erase(this->pinnedNodeElements.begin()+i);
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
	constexpr double K = (NDim == 2) ? 10000 : 1000;
	this->pinnedNodeElements.push_back(new MountedPointSpring<NDim>(this, this->nodes[nodeID], rp, mounts[mountID], K));
}

template<int NDim>
void BenderSimulationMesh<NDim>::unmountNode(int nodeID, int mountID)
{
	Node * node = this->nodes[nodeID];
	Mount * mount = mounts[mountID];
	for(int i = this->pinnedNodeElements.size()-1; i >= 0; --i) {
		MountedPointSpring<NDim> * pin = dynamic_cast<MountedPointSpring<NDim> *>(this->pinnedNodeElements[i]);
		if(pin->node == node && pin->mount == mount) {
			delete this->pinnedNodeElements[i];
			this->pinnedNodeElements.erase(this->pinnedNodeElements.begin()+i);
		}
	}
}

template<int NDim>
void BenderSimulationMesh<NDim>::setNodePositionObjective(int nodeID, const P3D & x0) 
{
	objectives.push_back(new NodePositionObjective(this->nodes[nodeID], x0));
}

template<int NDim>
void BenderSimulationMesh<NDim>::setNodeGlobalNodePositionObjective(dVector const & x)
{
	clearObjectives();

	for(int i = 0; i < this->nodes.size(); ++i) {
		setNodePositionObjective(i, this->nodes[i]->getCoordinates(x));
	}
}
/*
template<int NDim>
void BenderSimulationMesh<NDim>::scaleAll(double s)
{
	for(size_t i = 0; i < x.size(); ++i) {
		x[i] *= s;
	}
	for(size_t i = 0; i < X.size(); ++i) {
		X[i] *= s;
	}
	for(size_t i = 0; i < m.size(); ++i) {
		m[i] *= std::pow(s, NDim);
	}
	for(size_t i = 0; i < f_ext.size(); ++i) {
		f_ext[i] *= std::pow(s, NDim + 1);
	}
	for(size_t i = 0; i < xSolver.size(); ++i) {
		xSolver[i] *= s;
	}
	energyFunction->initialize(this);
}

template<int NDim>
void BenderSimulationMesh<NDim>::moveAll(V3D v)
{
	int n = x.size() / NDim;
	for(int i = 0; i < n; ++i) {
		for(int j = 0; j < NDim; ++j) {
			x[i*NDim+j] += v[j];
		}
	}
	n = X.size() / NDim;
	for(int i = 0; i < n; ++i) {
		for(int j = 0; j < NDim; ++j) {
			X[i*NDim+j] += v[j];
		}
	}
	n = xSolver.size() / NDim;
	for(int i = 0; i < n; ++i) {
		for(int j = 0; j < NDim; ++j) {
			xSolver[i*NDim+j] += v[j];
		}
	}
	energyFunction->initialize(this);
}
*/

template<int NDim>
int BenderSimulationMesh<NDim>::getMountIdOfNode(int nodeID) {
	Node * node = this->nodes[nodeID];
	// search pins for node id
	int pinnedNodeElementID = -1;
	for(int j = 0; j < this->pinnedNodeElements.size(); ++j) {
		BaseEnergyUnit * pin = this->pinnedNodeElements[j];
		if(dynamic_cast<MountedPointSpring<NDim> *>(pin)) {	// check if the pinned node is pinned to a mount
			if(dynamic_cast<MountedPointSpring<NDim> *>(pin)->node == node) {
				pinnedNodeElementID = j;
				break;
			}
		}
	}
	// search for mount id
	if(pinnedNodeElementID >=0) {
		for(int j = 0; j < mounts.size(); ++j) {
			if(mounts[j] == dynamic_cast<MountedPointSpring<NDim> *>(this->pinnedNodeElements[pinnedNodeElementID])->mount) {
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
		obj->addO(this->x, this->X, o);
	}
	return(o);
}

template<int NDim>
double BenderSimulationMesh<NDim>::computeOofx(dVector const & x_in) {
	double o = 0;
	for(MeshObjective * obj : objectives)
	{
		obj->addO(x_in, this->X, o);
	}
	return(o);
}


template<int NDim>
void BenderSimulationMesh<NDim>::computeDoDx(dVector & dodx)
{
	dodx.resize(this->x.size());
	dodx.setZero();
	for(MeshObjective * obj : objectives)
	{
		obj->addDoDx(this->x, this->X, dodx);
	}
}

template<int NDim>
double BenderSimulationMesh<NDim>::computeTargetPositionError()
{
	double e = 0;
	int n_obj = 0;
	for(MeshObjective * obj : objectives)
	{
		obj->addError(this->x, e);
		++n_obj;
	}
	double e_rel = e / static_cast<double>(n_obj);
	return(e_rel);
}


template<int NDim>
void BenderSimulationMesh<NDim>::drawSimulationMesh() 
{
	SimulationMesh::drawSimulationMesh(V3D(0.9,0.97,1.0), 1.0,
									   V3D(1.0,0.0,0.0), 1.0,
									   V3D(1.0,0.0,0.0), 0.002);

	//for(MeshObjective * obj : objectives) {
	//	dynamic_cast<NodePositionObjective *>(obj)->draw(x);
	//}
}


// instantiation of 2D & 3D
// ATTENTION: other values for NDim won't work

template class BenderSimulationMesh<2>;
template class BenderSimulationMesh<3>;
