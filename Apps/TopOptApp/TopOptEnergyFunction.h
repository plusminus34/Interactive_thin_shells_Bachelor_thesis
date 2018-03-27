#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <FEMSimLib/SimulationMesh.h>
#include <FEMSimLib/CSTSimulationMesh2D.h>
#include <FEMSimLib/CSTElement2D.h>

class TopOptEnergyFunction : public ObjectiveFunction {
public:
	TopOptEnergyFunction(SimulationMesh* simMesh);
	virtual ~TopOptEnergyFunction(void);

	//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
	void updateRegularizingSolutionTo(const dVector &currentP);
	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
//	virtual void addGradientTo(dVector& grad, const dVector& p);

	//this method gets called whenever a new best solution to the objective function is found
	virtual void setCurrentBestSolution(const dVector& p);

	bool printDebugInfo;
	double regularizer = 0.001;
private:

	//this is the configuration of the sim mesh that is used as a regularizing solution...
	dVector m_p0;
	dVector tmpVec;

	SimulationMesh* simMesh;
};

