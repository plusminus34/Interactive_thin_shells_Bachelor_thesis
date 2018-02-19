#pragma once

#include <MathLib/mathLib.h>
#include <MathLib/Matrix.h>

class SimulationMesh;

class SimMeshElement {

//convenience variables
public: 
	std::vector<int> i_vec_; // nodal indices in an array

public:
	SimulationMesh* simMesh;

public:
	SimMeshElement(SimulationMesh* simMesh);
	~SimMeshElement() {}
 
	virtual void draw(const dVector &x) {}

	virtual double getEnergy(const dVector &x, const dVector &X) = 0;
	virtual void addEnergyGradientTo(const dVector &x, const dVector &X, dVector &grad) = 0;
    virtual void addEnergyHessianTo(const dVector &x, const dVector &X, std::vector<MTriplet> &hesEntries) = 0;

	virtual double getMass() { return 0; }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


};


