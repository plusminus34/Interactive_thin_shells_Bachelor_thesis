#pragma once

#include <FEMSimLib/SimMeshElement.h>
#include <FEMSimLib/Node.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>

//zero rest length spring connected to a target position (could be the mouse, or something else...)
class FixedPointSpring2D : public BaseEnergyUnit{
public:
	double K;
	Node *node;
	P3D targetPosition;
public:
	FixedPointSpring2D(SimulationMesh* simMesh, Node* n, P3D mousePos);
	~FixedPointSpring2D();
	virtual double getEnergy(const dVector& x, const dVector& X);
	virtual void addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad);
	virtual void addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries);
	virtual void draw(const dVector& x);
};

