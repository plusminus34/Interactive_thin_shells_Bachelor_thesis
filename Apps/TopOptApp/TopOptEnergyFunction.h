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

	double computeDeformationEnergyObjective(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

	//this method gets called whenever a new best solution to the objective function is found
	virtual void setCurrentBestSolution(const dVector& p);


	void applyDensityParametersToSimMesh(const dVector& densityParams) {
		for (uint i = 0; i < simMesh->elements.size(); i++) {
			if (CSTElement2D* e = dynamic_cast<CSTElement2D*>(simMesh->elements[i])) {
				e->topOptInterpolationDensity = pow(densityParams[i], 1);
			}
		}
	}

	bool printDebugInfo;
	double regularizer = 0.001;

	double smoothnessObjectiveWeight = 0.001;
	double binaryDensityObjectiveWeight = 0.001;
	double complianceObjectiveWeight = 1.0;

	bool minimizeOriginalCompliance = true;


private:

	//this is the configuration of the sim mesh that is used as a regularizing solution...
	dVector m_p0;
	dVector tmpVec;

	SimulationMesh* simMesh;
};

