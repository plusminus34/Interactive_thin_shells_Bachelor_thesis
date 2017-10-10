#pragma once

#include <MathLib/mathLib.h>
#include <MathLib/Matrix.h>

/**
	This class implements the interface for an elementary energy unit. As a function of deformed, undeformed, 
	and other parameters, such as boundary conditions, each class that extends this one will define a potential energy.
*/
class BaseEnergyUnit{

public:
	BaseEnergyUnit();
	virtual ~BaseEnergyUnit();

	virtual double getEnergy(const dVector& x, const dVector& X) = 0;
	virtual void addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) = 0;
    virtual void addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries) = 0;
	virtual void draw(const dVector& x) = 0;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


