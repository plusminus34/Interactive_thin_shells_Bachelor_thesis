#pragma once
#include <FEMSimLib/SimMeshElement.h>
#include <FEMSimLib/Node.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>
#include "BarycentricZeroLengthSpring.h"

typedef BarycentricZeroLengthSpring ZLSpring;

/*
	Spring that pins two nodes together
*/
class Pin : public SimMeshElement {
	friend class Paper3DApp;
protected:
	//A Pin consists of three barycentric 0-length springs
	ZLSpring* springs[3];

	int id;

	virtual double getMass();

	virtual double getEnergy(const dVector& x, const dVector& X);
	virtual void addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad);
	virtual void addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries);
	virtual void draw(const dVector& x);
	virtual void drawRestConfiguration(const dVector& X);

public:
	Pin(SimulationMesh* simMesh, int id, ZLSpring* s1, ZLSpring* s2, ZLSpring* s3);
	~Pin();

	virtual void setStiffness(double k);

	virtual const int getID() { return id; }
};
