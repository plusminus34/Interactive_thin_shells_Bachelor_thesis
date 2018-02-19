#pragma once

#include "SimMeshElement.h"
#include "Node.h"
#include <MathLib/mathLib.h>
#include <MathLib/Matrix.h>

enum class MaterialModel2D {MM_LINEAR_ISOTROPIC, MM_STVK, MM_NEO_HOOKEAN};

/**
	This class implements Constant Strain Triangles elements in 2D
*/
class CSTElement2D : public SimMeshElement {
	friend class SimulationMesh;
	friend class CSTSimulationMesh2D;
 
public:
	const double W = 06. / 100.;
    double shearModulus = W*1e6;
	double bulkModulus  = W*(2.6/1.2)*shearModulus;
	double massDensity = W * 5;// 0;

	MaterialModel2D matModel;
	double restShapeArea = 0;

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

	virtual double getEnergy(const dVector& x, const dVector&);
	void computeGradientComponents(const dVector& x, const dVector&);
	void computeHessianComponents(const dVector& x, const dVector&);

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

	virtual void addEnergyGradientTo(const dVector& x, const dVector&, dVector& grad);
    virtual void addEnergyHessianTo(const dVector& x, const dVector&, std::vector<MTriplet>& hesEntries);
	virtual void draw(const dVector& x);
	void drawRestConfiguration(const dVector& X);

	
public:
	CSTElement2D(SimulationMesh* simMesh, Node* n1, Node* n2, Node* n3);
	~CSTElement2D();

};


