#include <LazyFEMSimLib/FEMEnergyFunction.h>
#include <LazyFEMSimLib/SimulationMesh.h>
#include <iostream>
#include <GUILib/GLApplication.h>

FEMEnergyFunction::FEMEnergyFunction(void){
	setToStaticsMode(0.001);
}

FEMEnergyFunction::~FEMEnergyFunction(void){
}

void FEMEnergyFunction::initialize(SimulationMesh* simMesh){
	assert(simMesh != NULL);
	assert(simMesh->nodes.size() >= 1);
	this->simMesh = simMesh;

	m_s0 = this->simMesh->x;
}

//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
void FEMEnergyFunction::updateRegularizingSolutionTo(const dVector &currentP){
	m_s0 = currentP;
}

//estimate accelerations given new estimated positions...
void FEMEnergyFunction::estimateNodalAccelerations(const dVector& xNew, dVector& acc){
	//a = (v_{t+h} - v_t)/ h = (x_{t+h} - x_t)/(h*h) - v_t/h;
	acc = xNew / (timeStep*timeStep) - simMesh->x / (timeStep*timeStep) - simMesh->v / timeStep;
}


//The net energy is: 1/2 a'M a + E + x'F, where E is the potential energy stored in the various elements
double FEMEnergyFunction::computeValue(const dVector& s)
{
	double totalEnergy = 0.0;

	// static part of the energy
	simMesh->prepare_upto_energy(s);
	totalEnergy += simMesh->energy;

//if(std::isnan(totalEnergy) )
//{
//	std::cout << "is nan " << __FILE__ << ":" << __LINE__ << std::endl;
//	//GLApplication::getGLAppInstance()->appIsRunning = false;
//}

	if (useDynamics){
		int nDim = simMesh->x.size();
		//estimate the accelerations...
		estimateNodalAccelerations(s, tmpVec);
		//and now add a term to the energy that, when taken the derivative of, results in Ma (1/2 a'Ma)
		for (int i=0;i<nDim;i++)
			totalEnergy += 0.5 * tmpVec[i] * simMesh->m[i] * tmpVec[i] * timeStep*timeStep;
	}
	totalEnergy -= s.dot(simMesh->f_ext);

	//add the regularizer contribution
	if (regularizer > 0){
		tmpVec = s - m_s0;
		totalEnergy += 0.5*regularizer*tmpVec.dot(tmpVec);
	}

//if(std::isnan(totalEnergy) )
//{
//	std::cout << "is nan " << __FILE__ << ":" << __LINE__ << std::endl;
//	//GLApplication::getGLAppInstance()->appIsRunning = false;
//}

	return totalEnergy;
}

void FEMEnergyFunction::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& s) {

	hessianEntries.reserve(simMesh->hessianTriplets.size());
	hessianEntries.insert(hessianEntries.end(), simMesh->hessianTriplets.begin(), simMesh->hessianTriplets.end());

	int nDim = s.size();

	//add the regularizer now...

	if (useDynamics)
		for (int i = 0; i < nDim; i++)
			hessianEntries.push_back(MTriplet(i, i, simMesh->m[i] * 1.0 / (timeStep*timeStep)));

	if (regularizer > 0)
		for (int i = 0; i < nDim;i++)
			hessianEntries.push_back(MTriplet(i, i, regularizer));
}

void FEMEnergyFunction::addGradientTo(dVector& grad, const dVector& s) {
//	estimateGradientAt(dE_ds, s);
//	return &dE_ds;
	
	grad += simMesh->gradient;

	int nDim = simMesh->x.size();

	grad -= simMesh->f_ext;

	if (useDynamics){
		//estimate the accelerations...
		estimateNodalAccelerations(s, tmpVec);
		for (int i=0; i<nDim; i++)
			grad[i] += simMesh->m[i] * tmpVec[i];
	}

	//add the regularizer now... 
	if (regularizer > 0){
		tmpVec = s - m_s0;
		grad += tmpVec * regularizer;
	}
}

//this method gets called whenever a new best solution to the objective function is found
void FEMEnergyFunction::setCurrentBestSolution(const dVector& s){
	simMesh->prepare_upto_energy(s);	// note: this MAYBE could be skipped, since this will usually have been computed already at this point
	simMesh->prepare_upto_hessian(s);
	updateRegularizingSolutionTo(s);
}





