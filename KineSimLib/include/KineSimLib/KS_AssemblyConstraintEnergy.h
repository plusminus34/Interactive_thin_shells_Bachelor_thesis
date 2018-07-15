#pragma once

#include <OptimizationLib/ObjectiveFunction.h>

class KS_MechanicalAssembly;
class KS_Constraint;

class KS_AssemblyConstraintEnergy : public ObjectiveFunction {
	friend class KS_IKConstraintEnergy;
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

	void setRegularizer(double val){
		regularizer = val;
	}

	int getScalarConstraintCount(){return scalarConstraintCount;}

private:
	//this is the mechanical assembly that the simulator acts on
	KS_MechanicalAssembly* assembly;
	//this is the array of constraints in the simulator
	std::vector<KS_Constraint*> constraints;
	
	
	
	dVector m_s0;
	double regularizer;
    // gradient and the hessian
	dVector dE_ds;
	DynamicArray<MTriplet> hessianEntries= DynamicArray<MTriplet>();// need this to pass over to the iKConstrainEnergy;
	SparseMatrix ddE_dsds;
	int scalarConstraintCount;
};

