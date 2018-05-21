#pragma once
#include <FEMSimLib/SimMeshElement.h>
#include <FEMSimLib/Node.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>

#include "BarycentricZeroLengthSpring.h"

/*
	Spring that pins two nodes together
*/
class Pin : public SimMeshElement {
	friend class Paper3DApp;
protected:
	//A Pin consists of three barycentric 0-length springs
	BarycentricZeroLengthSpring* springs[3];

	virtual double getMass();

	virtual double getEnergy(const dVector& x, const dVector& X);
	virtual void addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad);
	virtual void addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries);
	virtual void draw(const dVector& x);
	virtual void drawRestConfiguration(const dVector& X);

public:
	Pin(SimulationMesh* simMesh, BarycentricZeroLengthSpring* s1, BarycentricZeroLengthSpring* s2, BarycentricZeroLengthSpring* s3);
	~Pin();

};

