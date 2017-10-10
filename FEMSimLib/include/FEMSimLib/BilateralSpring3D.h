#pragma once
#include <FEMSimLib/SimMeshElement.h>
#include <FEMSimLib/Node.h>
#include <MathLib/mathLib.h>
#include <MathLib/Matrix.h>

/*
	Spring that connects two nodes
*/
class BilateralSpring3D : public SimMeshElement {
private:
    //material parameters...
    double k = 500;
    //relates length of spring to mass of elements
    double massDensity = 1;
    //keep track of the rest length of the spring
    double restLength = 0;

    //the collection of nodes that define the edge element
    Node* n[2];
    //parameters needed for gradient and hessian of the energy
    V3D dEdx[3];
    Matrix3x3 ddEdxdx[2][2];
	Matrix3x3 ddsdx1dx1, ddsdx2dx2, ddsdx1dx2;

	V3D getCurrentEdgeVector(const dVector& x, const dVector& X);
	double getCurrentEdgeLength(const dVector& x, const dVector& X);
	double getEdgeStrain(const dVector& x, const dVector& X);

    double computeRestShapeLength(const dVector& X);
    void computeGradientComponents(const dVector& x, const dVector& X);
    void computeHessianComponents(const dVector& x, const dVector& X);

    //sets important properties of the rest shape using the set of points passed in as parameters
    virtual void setRestShapeFromCurrentConfiguration();

    virtual double getMass();

    virtual double getEnergy(const dVector& x, const dVector& X);
    virtual void addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad);
    virtual void addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries);
    virtual void draw(const dVector& x);
    virtual void drawRestConfiguration(const dVector& X);

public:
    BilateralSpring3D(SimulationMesh* simMesh, Node* n1, Node* n2);
    ~BilateralSpring3D();

};

