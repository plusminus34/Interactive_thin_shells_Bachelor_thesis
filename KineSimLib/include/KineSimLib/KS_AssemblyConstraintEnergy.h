#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
//#include "KS_Constraint.h"
//#include "KS_MechanicalAssembly.h"
//#include <MathLib/SparseMatrix.h>

class KS_MechanicalAssembly;
class KS_Constraint;

class KS_AssemblyConstraintEnergy : public ObjectiveFunction {
friend class KS_MechanicalAssemblySimulator;
public:
	KS_AssemblyConstraintEnergy(void);
	virtual ~KS_AssemblyConstraintEnergy(void);

	void initialize(KS_MechanicalAssembly* a);

	//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
	void updateRegularizingSolutionTo(const dVector &currentS);
	virtual double computeValue(const dVector& s);

	//virtual SparseMatrix* getHessianAt(const dVector& s);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& s);
	//virtual dVector* getGradientAt(const dVector& s);
	virtual void addGradientTo(dVector& grad, const dVector& s);


	//this method gets called whenever a new best solution to the objective function is found
	virtual void setCurrentBestSolution(const dVector& s);

	virtual dVector* getConstraintVectorAt(const dVector& s);
	//virtual SparseMatrix* getConstraintJacobianAt(const dVector& s);
	virtual void getConstraintJacobianAt(const dVector& s);

	//virtual void getPlanarConstraintJacobianAt(const dVector& s, Matrix &J);

	void setRegularizer(double val){
		regularizer = val;
	}

	int getScalarConstraintCount(){return scalarConstraintCount;}

private:
	//this is the mechanical assembly that the simulator acts on
	KS_MechanicalAssembly* assembly;
	//this is the array of constraints in the simulator
	std::vector<KS_Constraint*> constraints;

	//constraint values
	dVector C;
	//jacobian of the constraints
	//SparseMatrix dCds;
	DynamicArray<MTriplet>& dCdsEntries= DynamicArray<MTriplet>();

	//The total energy of the system is the sum of the individual energy terms of the constraint. The gradient of this scalar function is this
	dVector dE_ds;
	//and the hessian of the sum of the energy terms is a sparse matrix...
	SparseMatrix ddE_dsds;
	dVector m_s0;
	double regularizer;

	int scalarConstraintCount;
};

