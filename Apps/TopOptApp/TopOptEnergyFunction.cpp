#include "TopOptEnergyFunction.h"

TopOptEnergyFunction::TopOptEnergyFunction(SimulationMesh* simMesh){
	this->simMesh = simMesh;
	printDebugInfo = false;

	regularizer = 0;
}

TopOptEnergyFunction::~TopOptEnergyFunction(void){

}

//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
void TopOptEnergyFunction::updateRegularizingSolutionTo(const dVector &currentP){
	m_p0 = currentP;
}

void TopOptEnergyFunction::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	hessianEntries.clear();

	//we will fake here a somewhat cautious gradient descent solver by hard coding the hessian to something nice-ish
	for (int i = 0; i < p.size(); i++)
		hessianEntries.push_back(MTriplet(i, i, complianceObjectiveWeight / 1.0));

	//================================================================================
	//-- smoothness - favor larger material clusters rather than small, isolated islands
	for (SimMeshElement* sme : simMesh->elements) {
		ADD_HES_ELEMENT(hessianEntries, sme->elementIndex, sme->elementIndex, 1.0, smoothnessObjectiveWeight);
		for (SimMeshElement* nsme : sme->adjacentElements) {
			ADD_HES_ELEMENT(hessianEntries, sme->elementIndex, nsme->elementIndex, -1.0 / sme->adjacentElements.size(), smoothnessObjectiveWeight);
			for (SimMeshElement* nsme2 : sme->adjacentElements) {
				if (nsme2->elementIndex >= nsme->elementIndex)
					ADD_HES_ELEMENT(hessianEntries, nsme2->elementIndex, nsme->elementIndex, 1.0 / (sme->adjacentElements.size() * sme->adjacentElements.size()), smoothnessObjectiveWeight);
			}
		}
	}
}

//this method gets called whenever a new best solution to the objective function is found
void TopOptEnergyFunction::setCurrentBestSolution(const dVector& p){
	updateRegularizingSolutionTo(p);

	if (printDebugInfo){
		Logger::consolePrint("-------------------------------\n");
		Logger::logPrint("-------------------------------\n");

		double totalVal = computeValue(p);
		Logger::consolePrint("=====> total cost: %lf\n", totalVal);
		Logger::logPrint("=====> total cost: %lf\n", totalVal);
	}
}

double TopOptEnergyFunction::computeDeformationEnergyObjective(const dVector& p) {
	if (minimizeOriginalCompliance)
		applyConstantDensityParametersToSimMesh();

	double totalEnergy = 0;
	for (uint i = 0; i < simMesh->elements.size(); i++)
		totalEnergy += simMesh->elements[i]->getEnergy(simMesh->x, simMesh->X);

	if (minimizeOriginalCompliance)
		applyDensityParametersToSimMesh(p);

	return totalEnergy;
}

double TopOptEnergyFunction::computeValue(const dVector& p) {
	double totalEnergy = 0;

	//================================================================================
	//-- compliance/overall deformation energy of the simulation mesh
	applyDensityParametersToSimMesh(p);
	simMesh->solve_statics();

	totalEnergy += computeDeformationEnergyObjective(p) * complianceObjectiveWeight;

	//================================================================================
	//-- smoothness - favor larger material clusters rather than small, isolated islands
	for (SimMeshElement* sme : simMesh->elements) {
		double avgNeighbourhoodDensity = 0;
		for (SimMeshElement* nsme : sme->adjacentElements) {
			CSTElement2D* e = dynamic_cast<CSTElement2D*>(nsme);
			avgNeighbourhoodDensity += p[e->elementIndex] / sme->adjacentElements.size();
		}

		CSTElement2D* e = dynamic_cast<CSTElement2D*>(sme);
		totalEnergy += 0.5 * (p[e->elementIndex] - avgNeighbourhoodDensity) * (p[e->elementIndex] - avgNeighbourhoodDensity) * smoothnessObjectiveWeight;
	}

	//================================================================================
	//-- black-or-white solution - each element should decide to either have density 1 or 0
	for (int i = 0; i < p.size();i++)
		totalEnergy += p[i] * (1-p[i]) * binaryDensityObjectiveWeight;

	//================================================================================
	//-- regularizer contribution
	if (regularizer > 0) {
		resize(tmpVec, p.size());
		if (m_p0.size() != p.size()) m_p0 = p;
		tmpVec = p - m_p0;
		totalEnergy += 0.5*regularizer*tmpVec.dot(tmpVec);
	}

	return totalEnergy;
}

void TopOptEnergyFunction::addGradientTo(dVector& grad, const dVector& p) {
	assert(p.size() == theMotionPlan->paramCount);

	resize(grad, p.size());

	int xDim = simMesh->x.size();
	int pDim = p.size();

	applyDensityParametersToSimMesh(p);
	simMesh->solve_statics();


	//================================================================================
	//-- compliance/overall deformation energy

	//we write the total deformation energy as E = P' E0, where P(p) is a vector of scalars cooresponding to the topopt parameters, E0 is a vector of per-element energy terms evaluated at P(p)=1 (e.g. full material)
	//what we want is to know how the gradient of the total deformation energy changes with respect to p: dE / dp = dE / dx * dx / dp + partial E / partial p. The first term is simply:
	//dE / dx = dE0/dx' * P
	if (minimizeOriginalCompliance)
		applyConstantDensityParametersToSimMesh();
	dVector dEdx; resize(dEdx, xDim);
	for (uint i = 0; i<simMesh->elements.size(); i++)
		simMesh->elements[i]->addEnergyGradientTo(simMesh->x, simMesh->X, dEdx);
	if (minimizeOriginalCompliance)
		applyDensityParametersToSimMesh(p);

	//now, we need to know how x changes wrt to p. To compute dx/dp, we need to know how the gradient G of the entire energy driving the statics solve changes wrt x and p: dx/dp = (dG/dx)^-1 * dG/dp
	SparseMatrix dGdx; resize(dGdx, xDim, xDim);
	DynamicArray<MTriplet> triplets;
	simMesh->energyFunction->addHessianEntriesTo(triplets, simMesh->x);
	dGdx.setFromTriplets(triplets.begin(), triplets.end());

	//ok, now we need to get the Jacobian dG/dp = d dE0/dx dP * dP/dp, where dE0/dx is a jacobian where the block at (i,j) tells us how the energy of the jth element changes with respect to the coordinates of the ith node... all other energy terms are independent of p... Note that all other terms in the energy we use for static solves are not a function of p, and therefore their gradient wrt p vanishes
	SparseMatrix dGdP; resize(dGdP, xDim, pDim);
	dVector tmp; resize(tmp, xDim);
	dVector dPdpDiag; resize(dPdpDiag, pDim);
	applyConstantDensityParametersToSimMesh(); 
	dVector partialEpartialP; resize(partialEpartialP, pDim);
	triplets.clear();
	for (uint i = 0; i < simMesh->elements.size(); i++) {
		tmp.setZero();
		simMesh->elements[i]->addEnergyGradientTo(simMesh->x, simMesh->X, tmp);

		if (CSTElement2D* e = dynamic_cast<CSTElement2D*>(simMesh->elements[i])) {
			for (int nIndex = 0; nIndex<3; nIndex++) {
				for (int j = e->n[nIndex]->dataStartIndex; j < e->n[nIndex]->dataStartIndex + 2; j++)
					triplets.push_back(MTriplet(j, i, tmp[j]));
			}
		}

		dPdpDiag[i] = rho * pow(p[i], rho - 1);

		partialEpartialP[i] += simMesh->elements[i]->getEnergy(simMesh->x, simMesh->X);
	}

	//NOTE: G is the gradient of the entire energy used for statics solves, but only the simulation elements are a function of P
	dGdP.setFromTriplets(triplets.begin(), triplets.end());
	applyDensityParametersToSimMesh(p);

	//all right, we now have all the ingredients...

	//dE/dp = dE/dx * dG/dx^-1 * dG/dp + partial E/ partial p, all up to transposes...
	Eigen::SimplicialLDLT<SparseMatrix, Eigen::Lower> solver;
	//	Eigen::SparseLU<SparseMatrix> solver;
	solver.compute(dGdx);
	dVector dEdx_times_dxdp = (solver.solve(dEdx).transpose() * dGdP).transpose() * -1;

	if (minimizeOriginalCompliance)
		grad = (dEdx_times_dxdp) * complianceObjectiveWeight;
	else
		grad = (dEdx_times_dxdp + partialEpartialP) * complianceObjectiveWeight;

	grad = dPdpDiag.asDiagonal() * grad;

	//================================================================================
	//-- black-or-white solution - each element should decide to either have density 1 or 0
	for (int i = 0; i < p.size(); i++)
		grad[i] += (1 - 2 * p[i]) * binaryDensityObjectiveWeight;

	//================================================================================
	//-- smoothness - favor larger material clusters rather than small, isolated islands
	for (SimMeshElement* sme : simMesh->elements) {
		double avgNeighbourhoodDensity = 0;
		for (SimMeshElement* nsme : sme->adjacentElements) {
			CSTElement2D* e = dynamic_cast<CSTElement2D*>(nsme);
			avgNeighbourhoodDensity += p[e->elementIndex] / sme->adjacentElements.size();
		}

		CSTElement2D* e = dynamic_cast<CSTElement2D*>(sme);
		grad[sme->elementIndex] += (p[e->elementIndex] - avgNeighbourhoodDensity) * smoothnessObjectiveWeight;

		for (SimMeshElement* nsme : sme->adjacentElements) {
			grad[nsme->elementIndex] += -1.0 / sme->adjacentElements.size() * (p[e->elementIndex] - avgNeighbourhoodDensity) * smoothnessObjectiveWeight;
		}
	}

   //================================================================================
   //-- regularizer contribution
	if (regularizer > 0) {
		if (m_p0.size() != p.size()) m_p0 = p;
		grad += (p - m_p0) * regularizer;
	}
}


/*
double TopOptEnergyFunction::computeValue(const dVector& p){
double totalEnergy = 0;

//a simple first test... try to maximize p...
//	for (int i = 0; i < p.size(); i++)
//		totalEnergy += (1 - p[i]) * (1 - p[i]);


//================================================================================
//-- compliance/overall deformation energy of the simulation mesh
applyDensityParametersToSimMesh(p);
double tmpResidual = simMesh->targetSolverResidual;
simMesh->targetSolverResidual = 1e-50;
simMesh->solve_statics();
simMesh->targetSolverResidual = tmpResidual;

for (uint i = 0; i<simMesh->elements.size(); i++)
totalEnergy += simMesh->elements[i]->getEnergy(simMesh->x, simMesh->X);

//================================================================================
//-- regularizer contribution
if (regularizer > 0){
resize(tmpVec, p.size());
if (m_p0.size() != p.size()) m_p0 = p;
tmpVec = p - m_p0;
totalEnergy += 0.5*regularizer*tmpVec.dot(tmpVec);
//		Logger::consolePrint("regularizer: %lf\n", regularizer);
}

return totalEnergy;
}

void TopOptEnergyFunction::addGradientTo(dVector& grad, const dVector& p) {
assert(p.size() == theMotionPlan->paramCount);

resize(grad, p.size());

int xDim = simMesh->x.size();
int pDim = p.size();

simMesh->energyFunction->setToStaticsMode(0);

applyDensityParametersToSimMesh(p);
double tmpResidual = simMesh->targetSolverResidual;
simMesh->targetSolverResidual = 1e-50;
simMesh->solve_statics();
simMesh->targetSolverResidual = tmpResidual;

//================================================================================
//-- compliance/overall deformation energy

//we write the total deformation energy as E = P' E0, where P' is a vector of scalars cooresponding to the topopt parameters, E0 is a vector of per-element energy terms evaluated at p=1 (e.g. full material)
//what we want is to know how the gradient of the total deformation energy changes with respect to x...
//dE / dx = dE0/dx' * p + dEp/dx

dVector dEdx; resize(dEdx, xDim);
for (uint i = 0; i<simMesh->elements.size(); i++)
simMesh->elements[i]->addEnergyGradientTo(simMesh->x, simMesh->X, dEdx);

//now, we need to know how x changes wrt to p to know how E changes with p: dE/dp = dE/dx * dx/dp. To compute dx/dp, we need to know how the gradient G of the entire energy driving the statics solve changes wrt x and p...

//we will also need to know the derivative of this gradient wrt x and p...
SparseMatrix dGdx; resize(dGdx, xDim, xDim);
DynamicArray<MTriplet> triplets;
simMesh->energyFunction->addHessianEntriesTo(triplets, simMesh->x);
dGdx.setFromTriplets(triplets.begin(), triplets.end());

//ok, now we need to get the Jacobian dG/dp = dE0/dx * dP/dp, where dE0/dx is a jacobian where the block at (i,j) tells us how the energy of the jth element changes with respect to the coordinates of the ith node... all other energy terms are independent of p... Note that all other terms in the energy we use for static solves are not a function of p, and therefore their gradient wrt p vanishes
SparseMatrix dGdp; resize(dGdp, xDim, pDim);
dVector tmp; resize(tmp, xDim);
triplets.clear();
for (uint i = 0; i < simMesh->elements.size(); i++) {
tmp.setZero();
simMesh->elements[i]->addEnergyGradientTo(simMesh->x, simMesh->X, tmp);

if (CSTElement2D* e = dynamic_cast<CSTElement2D*>(simMesh->elements[i])) {
for (int nIndex=0; nIndex<3;nIndex++){
for (int j = e->n[nIndex]->dataStartIndex; j < e->n[nIndex]->dataStartIndex + 2; j++)
triplets.push_back(MTriplet(j, i, tmp[j] / p[i]));
}
}
//		for (uint j = 0; j < xDim; j++) {
//			if (!IS_ZERO(tmp[j]))
//				triplets.push_back(MTriplet(j, i, tmp[j] / p[i]));
//		}
}
dGdp.setFromTriplets(triplets.begin(), triplets.end());

// we do need to multiply the matrix above by dP/dp - but it's just an identity when the SIMP method uses a rho of 1...

//all right, we now have all the ingredients...

//dE/dp = dE/dx * dG/dx^-1 * dG/dp, all up to transposes...
Eigen::SimplicialLDLT<SparseMatrix, Eigen::Lower> solver;
//	Eigen::SparseLU<SparseMatrix> solver;
solver.compute(dGdx);
dVector tmp2 = solver.solve(dEdx);

//	SparseMatrix tmpSparseMat = dGdx.triangularView<Eigen::StrictlyLower>().transpose();
//	dGdx = dGdx + tmpSparseMat;
//	dVector res = dGdx * tmp2 - dEdx;
//	Logger::consolePrint("residual: %lf\n", res.norm());

grad += (tmp2.transpose() * dGdp).transpose() * -1;


//---------------------
SparseMatrix dxdp = solver.solve(dGdp * -1);
SparseMatrix dxdpTest;  resize(dxdpTest, xDim, pDim);
triplets.clear();

double dp = 10e-4;
dVector pSet = p;

dVector C_P(xDim), C_M(xDim), H_i_col(xDim);

for (int i = 0; i<p.size(); i++) {
C_P.setZero();
C_M.setZero();
double tmpVal = p[i];
pSet[i] = tmpVal + dp;
applyDensityParametersToSimMesh(pSet);
double tmpResidual = simMesh->targetSolverResidual;
simMesh->targetSolverResidual = 1e-50;
simMesh->solve_statics();
simMesh->targetSolverResidual = tmpResidual;
C_P = simMesh->x;

pSet[i] = tmpVal - dp;
applyDensityParametersToSimMesh(pSet);
tmpResidual = simMesh->targetSolverResidual;
simMesh->targetSolverResidual = 1e-50;
simMesh->solve_statics();
simMesh->targetSolverResidual = tmpResidual;

C_M = simMesh->x;

//now reset the ith param value
pSet[i] = tmpVal;
applyDensityParametersToSimMesh(pSet);
tmpResidual = simMesh->targetSolverResidual;
simMesh->targetSolverResidual = 1e-50;
simMesh->solve_statics();
simMesh->targetSolverResidual = tmpResidual;

H_i_col = (C_P - C_M) / (2 * dp);

//each vector is a column vector of the hessian, so copy it in place...
for (int j = i; j < xDim; j++)
if (!IS_ZERO(H_i_col[j]))
triplets.push_back(MTriplet(j, i, H_i_col[j]));
}
dxdpTest.setFromTriplets(triplets.begin(), triplets.end());

print("..\\out\\dxdp_fd.m", dxdpTest);
print("..\\out\\dxdp.m", dxdp);

dVector testResult = dEdx.transpose() * dxdp;
dVector testResult2 = dEdx.transpose() * dxdpTest;

print("..\\out\\test_dxdp_fd.m", testResult2.transpose());
print("..\\out\\test_dxdp.m", testResult.transpose());

double tol = 1e-4;
double eps = 1e-10;

Logger::logPrint("checking dxdp\n");
for (int i = 0; i<dGdp.rows(); i++) {
for (int j = 0; j<dGdp.cols(); j++) {
double absErr = std::abs(dxdp.coeff(i, j) - dxdpTest.coeff(i, j));
double relError = 2 * absErr / (eps + std::abs(dxdp.coeff(i, j)) + std::abs(dxdpTest.coeff(i, j)));
if (relError > tol && absErr > 1e-5) {
Logger::logPrint("() aval\t%lf\tFD\t%lf\tError\t%lf\t\n", dxdp.coeff(i, j), dxdpTest.coeff(i, j), absErr);
}
}
}

exit(0);

//---------------------



//================================================================================
//-- regularizer contribution
if (regularizer > 0) {
if (m_p0.size() != p.size()) m_p0 = p;
grad += (p - m_p0) * regularizer;
}

}

*/