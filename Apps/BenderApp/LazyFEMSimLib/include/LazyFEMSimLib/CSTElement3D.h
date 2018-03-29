#pragma once

#include <array>

#include <LazyFEMSimLib/SimMeshElement.h>
#include <LazyFEMSimLib/Node.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>

enum MaterialModel3D { MM_LINEAR_ISOTROPIC, MM_STVK, MM_NEO_HOOKEAN };

class SimulationMesh;

/**
	This class implements Constant Strain Tetrahedral elements in 3D
*/
class CSTElement3D : public SimMeshElement {
friend SimulationMesh;

protected:
    //material parameters...
    double shearModulus, bulkModulus; // -> default values set in constructor (50/50)
    //relates area/volume to the mass of the element
    double massDensity;// = 1;	-> set as default value in constructor
    //material model used
    MaterialModel3D matModel;
    //keep track of the rest shape volume
    double restShapeVolume = 0;

    //the collection of nodes that define the tet element
    Node* n[4];

    // Energy and it's derivatives
	double E;
    V3D dEdx[4];
    Matrix3x3 ddEdxdx[4][4];
    // precomputed for new rest shape (see also below)
    Matrix3x3 dXInv;
	Matrix3x3 dXInv_dXInvT;
	V3D dXInv_colsum;
	V3D dXInv_dxInvT_colsum;
	//V3D dxInvT_rowsum;
	// precomputed for new node positions (computeDeformationGradient());
	Matrix3x3 F, /*Finv,*/ FinvT;
	double F_norm2, F_logdet;
	// temporary helpers used within some computations
	Matrix3x3 dx, dEdF, strain;

	// also precomputed for new rest shape
	//std::array<std::array<Matrix3x3, 3 >, 4> dFdXij;

protected:
    double computeRestShapeVolume(const dVector& X);

    //as a deformation measure, we need to compute the deformation gradient F. F maps deformed vectors dx to undeformed coords dX: dx = F*dX.
    //for linear basis functions, an easy way to compute it is by looking at the matrix that maps deformed traingle/tet edges to their underformed counterparts (F = dx * inv(dX)).
    void computeDeformationGradient(const dVector& x, const dVector& X);
	void computeCommonGradHess();

	void computeEnergy();
    void computeGradientComponents();
    void computeHessianComponents();

    //sets important properties of the rest shape using the set of points passed in as parameters
    virtual void setRestShapeFromCurrentConfiguration();

    void setYoungsModulusAndPoissonsRatio(double E, double nu) {
        shearModulus = E / (2 * (1 + nu));
        bulkModulus = E / (3 * (1 - 2 * nu));
    }

    void setShearModulusAndBulkModulus(double G, double K) {
        shearModulus = G;
        bulkModulus = K;
    }

    virtual double getMass();

    virtual double getEnergy(const dVector& x, const dVector& X);
    virtual void addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad);
    virtual void addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries);
	void addEnergyHessianTo_fixedPosition(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries, int i);
    virtual void draw(const dVector& x);
    virtual void drawRestConfiguration(const dVector& X);

public:
    CSTElement3D(SimulationMesh* simMesh, Node* n1, Node* n2, Node* n3, Node* n4, 
				 double massDensity = 1.0, double shearModulus = 50, double bulkModulus = 50);
    ~CSTElement3D();

};

