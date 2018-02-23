#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>

class SimulationMesh;

class FEMEnergyFunction : public ObjectiveFunction {
public:
	FEMEnergyFunction(void);
	virtual ~FEMEnergyFunction(void);

	void initialize(SimulationMesh* simMesh);

	void updateRegularizingSolutionTo(const dVector &currentS);

	virtual double computeValue(const dVector &x); 
	virtual void addGradientTo(dVector &grad, const dVector &x);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet> &hessianEntries, const dVector &x);

	virtual void setCurrentBestSolution(const dVector& x);

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

public:
	//this is the mechanical assembly that the simulator acts on
	SimulationMesh* simMesh;

	//this is the configuration of the sim mesh that is used as a regularizing solution...
	dVector m_x0;
	dVector tmpVec;
	double regularizer;
	bool useDynamics;
	double timeStep;
};

