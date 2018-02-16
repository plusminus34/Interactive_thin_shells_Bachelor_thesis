#include "BenderSimulationMesh.h"
#include "RotationMount2D.h"
#include "MountedPointSpring.h"


template<int NDim>
BenderSimulationMesh<NDim>::BenderSimulationMesh()
 : meshEnergyRegularizer(this)
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
int BenderSimulationMesh<NDim>::getSelectedNodePositionObjectiveID(Ray const & ray)
{
    int ID = -1;
    double dis = std::numeric_limits<double>::max();
    for (uint i = 0; i < objectives.size(); i++) {
		NodePositionObjective * nodePosObj = dynamic_cast<NodePositionObjective *>(objectives[i]);
		if(nodePosObj) {
			P3D tp = nodePosObj->targetPosition;
			double tDis = ray.getDistanceToPoint(tp) / (sqrt((tp - ray.origin).dot(tp - ray.origin)));
			//Logger::consolePrint("%lf %lf %lf %lf\n", tp.x(), tp.y(), tp.z(), tDis);
			if (tDis < 0.01 && tDis < dis) {
				dis = tDis;
				ID = i;
			}
		}
    }
    return ID;
}


template<int NDim>
int BenderSimulationMesh<NDim>::setNodePositionObjective(int nodeID, const P3D & x0) 
{
	objectives.push_back(new NodePositionObjective(this->nodes[nodeID], x0));
	return(objectives.size() - 1);
}

template<int NDim>
int BenderSimulationMesh<NDim>::setNodePositionObjectiveNoDuplicate(int nodeID, const P3D & x0)
{
	removeNodePositionObjectivesOfNode(nodeID);
	int objectiveID = setNodePositionObjective(nodeID, x0);
	return(objectiveID);
}

template<int NDim>
void BenderSimulationMesh<NDim>::setNodeGlobalNodePositionObjective(dVector const & x)
{
	clearObjectives();

	for(int i = 0; i < this->nodes.size(); ++i) {
		setNodePositionObjective(i, this->nodes[i]->getCoordinates(x));
	}
}


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
void BenderSimulationMesh<NDim>::clearNodePositionObjectives()
{
	for (int i = objectives.size()-1; i >= 0; --i) {
		NodePositionObjective * obj = dynamic_cast<NodePositionObjective *>(objectives[i]);
		if (obj) {
			delete obj;
			objectives.erase(objectives.begin() + i);
		}
	}
}

template<int NDim>
void BenderSimulationMesh<NDim>::removeObjective(int objectiveID)
{
	if(objectiveID >= 0 && objectiveID < (int)objectives.size()) {
		delete objectives[objectiveID];
		objectives.erase(objectives.begin()+objectiveID);
	}
}

template<int NDim>
void BenderSimulationMesh<NDim>::removeNodePositionObjectivesOfNode(int nodeID)
{
	for (int i = objectives.size() - 1; i >= 0; --i) {
		NodePositionObjective * obj = dynamic_cast<NodePositionObjective *>(objectives[i]);
		if (obj && obj->node->nodeIndex == nodeID) {
			delete obj;
			objectives.erase(objectives.begin() + i);
		}
	}
}

template<int NDim>
double BenderSimulationMesh<NDim>::computeO()
{
	double o = 0;
	for(MeshObjective * obj : objectives)
	{
		obj->addO(this->x, this->X, o);
	}
	// regularizer:
	meshPositionRegularizer.addO(this->x, this->X, o);
	meshEnergyRegularizer.addO(this->x, this->X, o);
	return(o);
}

/*
template<int NDim>
double BenderSimulationMesh<NDim>::computeOofx(dVector const & x_in) {
	double o = 0;
	for(MeshObjective * obj : objectives)
	{
		obj->addO(x_in, this->X, o);
	}
	return(o);
}
*/

template<int NDim>
void BenderSimulationMesh<NDim>::computeDoDx(dVector & dodx)
{
	dodx.resize(this->x.size());
	dodx.setZero();
	for(MeshObjective * obj : objectives)
	{
		obj->addDoDx(this->x, this->X, dodx);
	}
	// regularizer:
	meshPositionRegularizer.addDoDx(this->x, this->X, dodx);
	meshEnergyRegularizer.addDoDx(this->x, this->X, dodx);
}

template<int NDim>
double BenderSimulationMesh<NDim>::computeTargetPositionError()
{
	if(objectives.size() == 0) {return(0.0);}

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
	/*
	SimulationMesh::drawSimulationMesh(V3D(0.9,0.97,1.0), 1.0,
									   V3D(1.0,0.0,0.0), 1.0,
									   V3D(1.0,0.0,0.0), 0.002);
	*/
	SimulationMesh::drawMeshSurface(this->x);


	//for(MeshObjective * obj : objectives) {
	//	dynamic_cast<NodePositionObjective *>(obj)->draw(x);
	//}
}





/*
template<int NDim>
double MeshEnergyRegularizer<NDim>::computeValue(const dVector & p) 
{
	if(r > 0.0) {
		return(r * idSolver->femMesh->energy);
	}
	else {
		return(0.0);
	}
}


template<int NDim>
void MeshEnergyRegularizer<NDim>::addGradientTo(dVector& grad, const dVector& p)
{
	if(r < 0.0) {
		int n_x = idSolver->femMesh->x.size();

		dVector & dEDx = idSolver->femMesh->gradient;

		dVector dEDp = dEDx.transpose() * idSolver->dxdxi;

		grad += r * dEDp;
	}
}
*/


void  MeshPositionRegularizer::addO(const dVector & x, const dVector & X, double & o)
{
	if(r <= 0.0) {return;}

	o += 0.5 * r * (x - X).squaredNorm();
}


void MeshPositionRegularizer::addDoDx(const dVector & x, const dVector & X, dVector & dodx)
{
	if(r <= 0.0) {return;}

	dodx += r * (x - X);

}

void  MeshEnergyRegularizer::addO(const dVector & x, const dVector & X, double & o)
{
	if(r <= 0.0) {return;}

	o += r * femMesh->energy;
}


void MeshEnergyRegularizer::addDoDx(const dVector & x, const dVector & X, dVector & dodx)
{
	if(r <= 0.0) {return;}

	dodx += r * femMesh->gradient;

}



















// instantiation of 2D & 3D
// ATTENTION: other values for NDim won't work

template class BenderSimulationMesh<2>;
template class BenderSimulationMesh<3>;
