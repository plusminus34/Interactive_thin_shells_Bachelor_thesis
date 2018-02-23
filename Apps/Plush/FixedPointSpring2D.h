#pragma once

#include "SimMeshElement.h"
#include "Node.h"
#include <MathLib/mathLib.h>
#include <MathLib/Matrix.h>

//zero rest length spring connected to a target position (could be the mouse, or something else...)
class FixedPointSpring2D : public SimMeshElement{
public:
	double K = 1e12;
	Node *node;
	P3D targetPosition;
public:
	FixedPointSpring2D(SimulationMesh* simMesh, Node* n, P3D mousePos);
	~FixedPointSpring2D();
	virtual double getEnergy(const dVector& x, const dVector& X);
	virtual void addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad);
	virtual void addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries);
	virtual void draw(const dVector& x);

public:
	// (Unused) virtual functions to override.
	inline double getMass() { return 0.0; };
};

