#pragma once
#include <FEMSimLib/SimMeshElement.h>
#include <FEMSimLib/Node.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>

/*
	Spring that pins two nodes together
*/
class ZeroLengthSpring3D : public SimMeshElement {
	friend class Paper3DApp;
protected:
	//material parameters...
	double k = 0.5;

	//the collection of nodes that define the edge element
	Node* n[2];
	//parameters needed for gradient and hessian of the energy
	V3D dEdx[2];
	Matrix3x3 ddEdxdx[2][2];

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

public:
	ZeroLengthSpring3D(SimulationMesh* simMesh, Node* n1, Node* n2);
	~ZeroLengthSpring3D();

};

