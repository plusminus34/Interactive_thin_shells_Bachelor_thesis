#pragma once

#include <FEMSimLib/BaseEnergyUnit.h>
#include <FEMSimLib/SimMeshElement.h>
#include <FEMSimLib/Node.h>
#include <FEMSimLib/FEMEnergyFunction.h>
#include <MathLib/Ray.h>

/**
	This class implements a generic sim mesh for deformable objects: collection of nodes connected to each other using different types of elements
*/
class SimulationMesh{
	friend class FEMEnergyFunction;
	friend class TopOptApp;
	friend class TopOptConstraints;
	friend class Node;
	friend class FEMSimApp;
    friend class FEMSim3DApp;
	friend class CSTElement2D;
	friend class CSTElement3D;
	friend class BilateralSpring3D;
	friend class BenderApp2D;
	friend class BenderApp3D;
	template<int NDim> 
	friend class InverseDeformationSolver;

protected:
	//this is the list of nodes in the structure
	DynamicArray<Node*> nodes;
	//for each node we will store the position and velocity, rest configuration, mass and external forces acting on it
	dVector x, v, m, f_ext, X, xSolver;
	//list of elements that connects the nodes to each other. These elements are 
	DynamicArray<SimMeshElement*> elements;
	//a list of temporary used to pin points to locations that are fixed in space...
	DynamicArray<BaseEnergyUnit*> pinnedNodeElements;
	//this is the objective function that we use for simulations...
	FEMEnergyFunction* energyFunction;

	bool checkDerivatives;

	void clear();

public:
	SimulationMesh();
	~SimulationMesh();

	virtual void prepareForDraw() {	}

	void drawNodes();
	void drawSimulationMesh(V3D const & edgeColor = V3D(1.0,1.0,1.0), double edgeWidth = 1, 
							V3D const & pinnedNodeColor = V3D(1.0,0.0,0.0), double pinnedNodeSize = 1,
							V3D const & nodeColor = V3D(1.0,0.0,0.0), double nodeSize = 0.005);
	void drawExternalForces();
	void drawRestConfiguration();

	void solve_dynamics(double dt);
	void solve_statics();

	void addGravityForces(const V3D & g);

	//projects positions of nodes to lie on the plane, rather than under it, and kills normal component of their velocities
	void fakeContactWithPlane(const Plane& plane);

	void setCheckDerivativesFlag(bool flag){
		checkDerivatives = flag;
	}

	Node* getNodeIntersectedBy(const Ray& ray);

	virtual void readMeshFromFile(const char* fName) = 0;
	
	virtual int getSelectedNodeID(Ray ray) { return -1; }
	virtual int getSelectedElementID(Ray ray) { return -1; }

	virtual void setPinnedNode(int ID, const P3D& target) = 0;
	virtual void unpinNode(int ID) {};
	virtual void removePinnedNodeConstraints() {
		for (auto it = pinnedNodeElements.begin(); it != pinnedNodeElements.end(); ++it)
			delete *it;
		pinnedNodeElements.clear();
	}

	double getCurrentDeformationEnergy() {
		double totalEnergy;
		for (uint i = 0; i<elements.size(); i++)
			totalEnergy += elements[i]->getEnergy(x, X);

		for (uint i = 0; i<pinnedNodeElements.size(); i++)
			totalEnergy += pinnedNodeElements[i]->getEnergy(x, X);

		return totalEnergy;
	}

};
