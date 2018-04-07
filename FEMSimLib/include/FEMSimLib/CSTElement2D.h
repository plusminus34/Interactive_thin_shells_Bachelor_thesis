#pragma once

#include <FEMSimLib/SimMeshElement.h>
#include <FEMSimLib/Node.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>

enum MaterialModel2D {MM_LINEAR_ISOTROPIC=0, MM_STVK, MM_NEO_HOOKEAN};

/**
	This class implements Constant Strain Triangles elements in 2D
*/
class CSTElement2D : public SimMeshElement {
	friend class TopOptApp;
	friend class TopOptConstraints;
	friend class TopOptEnergyFunction;
	friend class CSTSimulationMesh2D;
private:
	//material parameters...
	double shearModulus = 50, bulkModulus = 50;
	//relates area/volume to the mass of the element
	double massDensity = 1;
	//material model used
	MaterialModel2D matModel;
	//keep track of the rest shape area
	double restShapeArea = 0;

	double defEnergyForDrawing = 0;

	double topOptInterpolationDensity = 1;

	//the collection of nodes that define the triangle element
	Node* n[3];
	//parameters needed for gradient and hessian of the energy
	V3D dEdx[3];
	Matrix2x2 ddEdxdx[3][3];
	//tmp matrices used to speed up computation of the deformation gradient, green strain, etc
	Matrix2x2 dx, dXInv, F, strain, dEdF, Finv, FinvT;

	double computeRestShapeArea(const dVector& X);

	//as a deformation measure, we need to compute the deformation gradient F. F maps deformed vectors dx to undeformed coords dX: dx = F*dX.
	//for linear basis functions, an easy way to compute it is by looking at the matrix that maps deformed traingle/tet edges to their underformed counterparts (F = dx * inv(dX)).
	void computeDeformationGradient(const dVector& x, const dVector& X, Matrix2x2& dxdX);

	void computeGradientComponents(const dVector& x, const dVector& X);
	void computeHessianComponents(const dVector& x, const dVector& X);

	//sets important properties of the rest shape using the set of points passed in as parameters
	virtual void setRestShapeFromCurrentConfiguration();

	void setYoungsModulusAndPoissonsRatio(double E, double nu){
		shearModulus = E/(2*(1 + nu));
		bulkModulus = E/(3*(1-2*nu));
	}

	void setShearModulusAndBulkModulus(double G, double K){
		shearModulus = G;
		bulkModulus = K;
	}

	virtual double getMass();

	virtual double getEnergy(const dVector& x, const dVector& X);
	virtual void addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad);
    virtual void addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries);
	virtual void draw(const dVector& x);
	virtual void drawRestConfiguration(const dVector& X);

	
public:
	CSTElement2D(SimulationMesh* simMesh, Node* n1, Node* n2, Node* n3);
	~CSTElement2D();

};


