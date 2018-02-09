#pragma once

#include <LazyFEMSimLib/BaseEnergyUnit.h>
#include <LazyFEMSimLib/SimMeshElement.h>
#include <LazyFEMSimLib/Node.h>
#include <LazyFEMSimLib/FEMEnergyFunction.h>
#include <MathLib/Ray.h>

/**
	This class implements a generic sim mesh for deformable objects: collection of nodes connected to each other using different types of elements
*/
class SimulationMesh{
	friend class FEMEnergyFunction;
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

	// working data: structur-dependent //////////////////
	// set in initializeStructure()
	int n_elements;
	std::vector<std::array<int, 4> > elementNodes;		// indices of the nodes of each element
	std::vector<std::array<int, 4> > elementNodeStarts; // indices at which the data for each node of the element starts (e.g. in x, X etc)


	// working data: state dependent /////////////////////
	double energy;
	dVector gradient;
	//int n_triplets_element = ;
	std::vector<MTriplet> hessianTriplets;

	std::vector<Matrix3x3> dxdX;	// deformation gradient for each element
	std::vector<double> dxdX_norm2;
	std::vector<double> dxdX_logdet;
	std::vector<Matrix3x3> dxdX_inv;
	std::vector<Matrix3x3> dxdX_invT;



public:
	SimulationMesh();
	~SimulationMesh();

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
	
	virtual int getSelectedNodeID(Ray ray) = 0;
	virtual void setPinnedNode(int ID, const P3D& target) = 0;
	virtual void removePinnedNodeConstraints() {
		for (auto it = pinnedNodeElements.begin(); it != pinnedNodeElements.end(); ++it)
			delete *it;
		pinnedNodeElements.clear();
	}

	//////////////////////////////////
	// LazyFEM functions
	void initializeStructure();
	void initializeState_xSolver();

	// for structure


	// for state

	void prepare_upto_energy(dVector const & x);
	void prepare_upto_hessian(dVector const & x);

	double energyElement_i(int i, dVector const & x);
	double energyElements(dVector const & x);
	double energyPinnedNodeElements(dVector const & x);

	void addGradientElement_i(int i, dVector const & x, dVector & grad);
	void addGradientElements(dVector const & x, dVector & grad);
	void addGradientPinnedNodeElements(dVector const & x, dVector & grad);

	void addHessianElement_i(int i, dVector const & x, std::vector<MTriplet> & hessianTriplets);
	void addHessianElements(dVector const & x, std::vector<MTriplet> & hessianTriplets);
	void addHessianPinnedNodeElements(dVector const & x, std::vector<MTriplet> & hessianTriplets);

	void computeDeformationGradients(dVector const & x);


};
