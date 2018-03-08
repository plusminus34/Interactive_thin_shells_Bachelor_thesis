#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>

class SimulationMesh;

class FEMEnergyFunction : public ObjectiveFunction {
public:
	FEMEnergyFunction(void);
	virtual ~FEMEnergyFunction(void);

	void initialize(SimulationMesh* simMesh);

	//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
	void updateRegularizingSolutionTo(const dVector &currentS);

	virtual double computeValue(const dVector& s);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& s);
	virtual void addGradientTo(dVector& grad, const dVector& s);

	//this method gets called whenever a new best solution to the objective function is found
	virtual void setCurrentBestSolution(const dVector& s);

	void setToStaticsMode(double regularizer){
		useDynamics = false;
		this->regularizer = regularizer;
	}

	void setToDynamicsMode(double dt){
		useDynamics = true;
		this->timeStep = dt;
		this->regularizer = 0;
	}


	//estimate accelerations given new estimated positions...
	void estimateNodalAccelerations(const dVector& xNew, dVector& acc);

private:
	//this is the mechanical assembly that the simulator acts on
	SimulationMesh* simMesh;

	//this is the configuration of the sim mesh that is used as a regularizing solution...
	dVector m_s0;
	dVector tmpVec;
	double regularizer;
	bool useDynamics;
	double timeStep;
};

