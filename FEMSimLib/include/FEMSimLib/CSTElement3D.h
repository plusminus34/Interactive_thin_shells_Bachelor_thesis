#pragma once

#include <FEMSimLib/SimMeshElement.h>
#include <FEMSimLib/Node.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>

enum MaterialModel3D { MM_LINEAR_ISOTROPIC, MM_STVK, MM_NEO_HOOKEAN };

/**
	This class implements Constant Strain Tetrahedral elements in 3D
*/
class CSTElement3D : public SimMeshElement {
private:
    //material parameters...
    double shearModulus = 50, bulkModulus = 50;
    //relates area/volume to the mass of the element
    double massDensity = 1;
    //material model used
    MaterialModel3D matModel;
    //keep track of the rest shape volume
    double restShapeVolume = 0;

    //the collection of nodes that define the tet element
    Node* n[4];
    //parameters needed for gradient and hessian of the energy
    V3D dEdx[4];
    Matrix3x3 ddEdxdx[4][4];
    //tmp matrices used to speed up computation of the deformation gradient, green strain, etc
    Matrix3x3 dx, dXInv, F, strain, dEdF, Finv, FinvT;

    double computeRestShapeVolume(const dVector& X);

    //as a deformation measure, we need to compute the deformation gradient F. F maps deformed vectors dx to undeformed coords dX: dx = F*dX.
    //for linear basis functions, an easy way to compute it is by looking at the matrix that maps deformed traingle/tet edges to their underformed counterparts (F = dx * inv(dX)).
    void computeDeformationGradient(const dVector& x, const dVector& X, Matrix3x3& dxdX);

    void computeGradientComponents(const dVector& x, const dVector& X);
    void computeHessianComponents(const dVector& x, const dVector& X);

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
    virtual void draw(const dVector& x);
    virtual void drawRestConfiguration(const dVector& X);

public:
    CSTElement3D(SimulationMesh* simMesh, Node* n1, Node* n2, Node* n3, Node* n4);
    ~CSTElement3D();

};


