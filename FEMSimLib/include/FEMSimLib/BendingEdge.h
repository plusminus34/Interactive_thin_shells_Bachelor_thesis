#pragma once
#include <FEMSimLib/SimMeshElement.h>
#include <FEMSimLib/Node.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>

/*
Element that defines a preferred angle between 2 triangles
 4 nodes are required and not all of them are equal:
  nodes 0 and 1 are on the actual edge
  nodes 2 and 3 are the other triangle corners

    0 - 2
   / \ /
  3 - 1
*/
class BendingEdge : public SimMeshElement {
	friend class Paper3DApp;
	friend class Paper3DMesh;
	friend class ShapeWindow;
private:
	//material parameters...
	double k = 0.05;
	//keep track of the rest shape
	double restAngle = 0;
	double restEdgeLength = 1;
	double restArea = 1;

	//the collection of nodes
	Node* n[4];

	//sets important properties of the rest shape using the set of points passed in as parameters
	virtual void setRestShapeFromCurrentConfiguration();

	virtual double getMass();
	virtual double getAngle(const dVector& x);

	virtual double getEnergy(const dVector& x, const dVector& X);
	virtual void addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad);
	virtual void addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries);
	virtual void draw(const dVector& x);
	virtual void drawRestConfiguration(const dVector& X);

public:
	BendingEdge(SimulationMesh* simMesh, Node* n1, Node* n2, Node* n3, Node* n4);
	~BendingEdge();

};