#pragma once

#include <OptimizationLib/ObjectiveFunction.h>

class KS_MechanicalAssembly;
class KS_IKMechanismController;

class KS_IKConstraintEnergy : public ObjectiveFunction {
	friend class KS_IKMechanismController;
public:
	KS_IKConstraintEnergy(void);
	virtual ~KS_IKConstraintEnergy(void);

	void initialize(KS_MechanicalAssembly* a, KS_IKMechanismController* ikCon);

	
	virtual double computeValue(const dVector& p);

    virtual void addGradientTo(dVector& grad, const dVector& p);

	//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.

	void updateRegularizingSolutionTo(const dVector &currentP);
	//this method gets called whenever a new best solution to the objective function is found
	virtual void setCurrentBestSolution(const dVector& p);
	void setRegularizer(double val){
		regularizer = val;
	}


	void computedAE_ds(const dVector& p);
	void computeddAE_ds_dp(const dVector& p);
	void computedO_ds(const dVector& p);
	void computedS_dp(const dVector& p);

	void computeddAE_ds_dpAnalytic(const dVector & p);

private:
	//this is the mechanical assembly that the simulator acts on
	KS_MechanicalAssembly* mechanism;
	KS_IKMechanismController* ikMechanismController;
	
	dVector m_p0;
	double regularizer;
	dVector dO_ds; //computeGradofEEObjectiveWithMechanismState
	dVector dAE_ds; // gradient of the assembly constraint energy with respect to the mechanism state
	MatrixNxM ddAE_ds_dp, ddAE_ds_dpA;//gradiant of the gradient with respect to the control parameters with fixed mechanism state
	MatrixNxM dS_dp;//compute the gradient of the mech state with respect to the control variables 
	int stateSize, stateCount, actCount;
	dVector currentMechState;
};

