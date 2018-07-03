#pragma once
#include <FEMSimLib/SimMeshElement.h>
#include <FEMSimLib/Node.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>
#include <FEMSimLib/BarycentricZeroLengthSpring.h>

typedef BarycentricZeroLengthSpring ZLSpring;

/*
	Pin connecting two triangles together using zero-length springs on the triangle corners

	Created for Paper3DApp, the general usefulness of this class is questionable
*/
class Pin : public SimMeshElement {
	friend class Paper3DApp;
protected:
	//A Pin consists of three barycentric 0-length springs
	ZLSpring* springs[3];

	int id;

	virtual double getMass();

	// NOTE: Energy is computed by summing over the component springs. A pin and its springs should not be part of the same mesh
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

