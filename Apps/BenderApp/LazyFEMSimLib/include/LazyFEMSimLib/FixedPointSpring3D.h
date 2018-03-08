#pragma once

#include <LazyFEMSimLib/SimMeshElement.h>
#include <LazyFEMSimLib/Node.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>


//zero rest length spring connected to a target position (could be the mouse, or something else...)
class FixedPointSpring3D : public BaseEnergyUnit {
public:
    double K;
    Node *node;
    P3D targetPosition;
public:
    FixedPointSpring3D(SimulationMesh* simMesh, Node* node, P3D targetPosition, double K = 100);
    ~FixedPointSpring3D();
    virtual double getEnergy(const dVector& x, const dVector& X);
    virtual void addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad);
    virtual void addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries);
    virtual void draw(const dVector& x);
};

