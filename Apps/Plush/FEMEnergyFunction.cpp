#include "FEMEnergyFunction.h"
#include "SimulationMesh.h"

FEMEnergyFunction::FEMEnergyFunction(void){
	setToStaticsMode(0.001);
}

FEMEnergyFunction::~FEMEnergyFunction(void){
}

void FEMEnergyFunction::initialize(SimulationMesh* simMesh){
	assert(simMesh != NULL);
	assert(simMesh->nodes.size() >= 1);
	this->simMesh = simMesh;

	m_x0 = this->simMesh->x;
}

// Regularizer looks like: r/2 * (p-p0)'*(p-p0).
// This function can update p0 if desired, given the current value of s.
void FEMEnergyFunction::updateRegularizingSolutionTo(const dVector &currentP){
	m_x0 = currentP;
}

//estimate accelerations given new estimated positions...
void FEMEnergyFunction::estimateNodalAccelerations(const dVector& xNew, dVector& acc){
	// NOTE: a = (v_{t+h} - v_t)/ h = x_{t+h} / h^2 - x_t / h^2 - v_t / h;
	acc = xNew / (timeStep*timeStep) - simMesh->x / (timeStep*timeStep) - simMesh->v / timeStep;
}

double FEMEnergyFunction::computeValue(const dVector &x){
	double totalEnergy = 0;

	// -- // E_internal
	for (uint i=0;i<simMesh->elements.size();i++)
		totalEnergy += simMesh->elements[i]->getEnergy(x, simMesh->balphac);
	
	// -- // h^2/2 aTMa
	if (useDynamics){
		int nDim = x.size();
		//estimate the accelerations...
		estimateNodalAccelerations(x, tmpVec);
		//and now add a term to the energy that, when taken the derivative of, results in Ma (1/2 a'Ma)
		for (int i=0;i<nDim;i++)
			totalEnergy += 0.5 * tmpVec[i] * simMesh->m[i] * tmpVec[i] * timeStep*timeStep;
	}

	// -- // -f x
	totalEnergy -= x.dot(simMesh->f_ext);
	totalEnergy -= x.dot(simMesh->f_ctc);
 
	// -- // .5 r x^2
	if (regularizer > 0){
		tmpVec = x - m_x0;
		totalEnergy += 0.5*regularizer*tmpVec.dot(tmpVec);
	}

	return totalEnergy;
}

void FEMEnergyFunction::addGradientTo(dVector &grad, const dVector &x) {

	// estimateGradientAt(dE_ds, s);
	// return &dE_ds;

	if (grad.size() != x.size())
		resize(grad, x.size());
	
	int nDim = x.size();

	// -- // g_internal
	for (auto &element : simMesh->elements) {
		element->addEnergyGradientTo(x, simMesh->balphac, grad);
	}

	// -- // Ma
	if (useDynamics) {
		estimateNodalAccelerations(x, tmpVec);
		for (int i=0; i<nDim; i++)
			grad[i] += simMesh->m[i] * tmpVec[i];
	}

	// -- // -f
	grad -= simMesh->f_ext;
	grad -= simMesh->f_ctc;
 
	// -- // r x
	if (regularizer > 0) {
		tmpVec = x - m_x0;
		grad += tmpVec * regularizer;
	}
	
}

void FEMEnergyFunction::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector &x) {

	int nDim = x.size(); 

	// -- // H_internal
	for (auto &element : simMesh->elements) {
		element->addEnergyHessianTo(x, simMesh->balphac, hessianEntries);
	}

	// -- // M / h^2
	if (useDynamics) {
		for (int i = 0; i < nDim; i++) {
			hessianEntries.push_back(MTriplet(i, i, simMesh->m[i] * 1.0 / (timeStep*timeStep)));
		}
	}

	// -- // r
	if (regularizer > 0) {
		for (int i = 0; i < nDim; i++) {
			hessianEntries.push_back(MTriplet(i, i, regularizer));
		}
	}

}

//this method gets called whenever a new best solution to the objective function is found
void FEMEnergyFunction::setCurrentBestSolution(const dVector& x){
	updateRegularizingSolutionTo(x);
}
