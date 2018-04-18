#pragma once
#include <FEMSimLib/SimMeshElement.h>
#include <FEMSimLib/Node.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>

/*
Spring that defines a preferred angle between 3 nodes
*/
class AngleSpring : public SimMeshElement {
private:
	//material parameters...
	double k = 500;
	//relates length of spring to mass of elements
	//double massDensity = 1;
	//keep track of the rest length of the spring
	double restAngle = 0;

	//the collection of nodes, the middle node n[1] being where the angle is defined
	Node* n[3];

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
	AngleSpring(SimulationMesh* simMesh, Node* n1, Node* n2, Node* n3);
	~AngleSpring();

};