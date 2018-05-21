#pragma once
#include <FEMSimLib/SimMeshElement.h>
#include <FEMSimLib/Node.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>

/*
	Spring that pins two points (given as a linear combination of three nodes each) together
*/
class BarycentricZeroLengthSpring : public SimMeshElement {
	friend class Paper3DApp;
	friend class Pin;
protected:
	//material parameters...
	double k = 0.5;

	//the collection of nodes that define the element
	Node* n[6];
	//and the weight of each node
	double weight[6];

	//parameters needed for gradient and hessian of the energy
	V3D dEdx[6];
	Matrix3x3 ddEdxdx[6][6];

	P3D getCurrentPoint(int point, const dVector& x, const dVector& X);
	V3D getCurrentEdgeVector(const dVector& x, const dVector& X);
	double getCurrentEdgeLength(const dVector& x, const dVector& X);

	void computeGradientComponents(const dVector& x, const dVector& X);
	void computeHessianComponents();

	virtual double getMass();

	virtual double getEnergy(const dVector& x, const dVector& X);
	virtual void addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad);
	virtual void addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries);
	virtual void draw(const dVector& x);
	virtual void drawRestConfiguration(const dVector& X);

	virtual void setWeights(int triangle, double w1, double w2, double w3);

public:
	BarycentricZeroLengthSpring(SimulationMesh* simMesh, Node* n11, Node* n12, Node* n13, Node* n21, Node* n22, Node* n23);
	~BarycentricZeroLengthSpring();

};

