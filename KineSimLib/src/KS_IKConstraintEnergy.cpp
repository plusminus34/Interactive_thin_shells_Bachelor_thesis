#include <KineSimLib/KS_IKConstraintEnergy.h>
#include <KineSimLib/KS_MechanicalAssembly.h>
#include <KineSimLib/KS_AssemblyConstraintEnergy.h>
#include <KineSimLib/KS_IKMechanismController.h>
#include <KineSimLib/KS_Constraint.h>



KS_IKConstraintEnergy::KS_IKConstraintEnergy(void){
	regularizer = 0;
}

KS_IKConstraintEnergy::~KS_IKConstraintEnergy(void){
}

void KS_IKConstraintEnergy::initialize(KS_MechanicalAssembly* a, KS_IKMechanismController* ikMechController){
	assert(a != NULL);
	assert(a->getComponentCount() >= 1);
	mechanism = a;
	ikMechanismController = ikMechController;

	stateCount = mechanism->getComponentCount() * KS_MechanicalComponent::getStateSize();
	stateSize = KS_MechanicalComponent::getStateSize();
	actCount = ikMechanismController->getActuatedConnectionCount();

	m_p0.resize(actCount); m_p0.setZero(); 
	//lastValidMotorAngleValues.resize(actCount); lastValidMotorAngleValues.setZero();lastValidMechState = mechanism->s;

	dO_ds.resize(stateCount); dO_ds.setZero(); //computeGradofEEObjectiveWithMechanismState
	dAE_ds.resize(stateCount); dAE_ds.setZero(); // gradient of the assembly constraint energy with respect to the mechanism state
	ddAE_ds_dp.resize(stateCount, actCount); ddAE_ds_dp.setZero();//gradiant of the gradient with respect to the control parameters with fixed mechanism state
	ddAE_ds_dpA.resize(stateCount, actCount); ddAE_ds_dpA.setZero();// analytic gradiant of the gradient with respect to the control parameters with fixed mechanism state
	dS_dp.resize(stateCount, actCount); dS_dp.setZero();//compute the gradient of the mech state with respect to the control variables 

}

//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
void KS_IKConstraintEnergy::updateRegularizingSolutionTo(const dVector &currentP){
	this->m_p0 = currentP;//copy(currentS, m_s0);
}

double KS_IKConstraintEnergy::computeValue(const dVector& p) {

	ikMechanismController->motorAngleValues = p;
	ikMechanismController->updateMotorConnections();
	mechanism->solveAssembly();
	currentMechState = mechanism->s;
	mechanism->AConstraintEnergy->setCurrentBestSolution(mechanism->s);
	double totalIkConstraintEnergy = 0;
	totalIkConstraintEnergy +=
	0.5*((ikMechanismController->xEEDOF)*SQR(ikMechanismController->eeComponent->getWorldCenterPosition()[0] - ikMechanismController->xEEd) +
		(ikMechanismController->yEEDOF)*SQR(ikMechanismController->eeComponent->getWorldCenterPosition()[1] - ikMechanismController->yEEd) +
		(ikMechanismController->zEEDOF)*SQR(ikMechanismController->eeComponent->getWorldCenterPosition()[2] - ikMechanismController->zEEd) +
		(ikMechanismController->alphaEEDOF)*SQR(ikMechanismController->eeComponent->getAlpha() - ikMechanismController->alphaEEd) +
		(ikMechanismController->betaEEDOF)*SQR(ikMechanismController->eeComponent->getBeta() - ikMechanismController->betaEEd) +
		(ikMechanismController->gammaEEDOF)*SQR(ikMechanismController->eeComponent->getGamma() - ikMechanismController->gammaEEd));
	
	//add the regularizer contribution
	dVector vd;
	vd.resize(actCount); vd.setZero();
	vd = p * 1.0 + m_p0 * (-1);
	double nrmvd2 = vd.squaredNorm();
	totalIkConstraintEnergy += 0.5*regularizer*nrmvd2;
	//Logger::print("ASS energy and IKConstraintEnergy  %lf %lf\n", mechanism->AConstraintEnergy->computeValue(mechanism->s), totalIkConstraintEnergy);
		return totalIkConstraintEnergy;
}

void KS_IKConstraintEnergy::addGradientTo(dVector & grad, const dVector & p)
{
	grad.resize(actCount);
	grad.setZero();
	computedO_ds(p);
	computedS_dp(p);
	grad += -1.0*((dO_ds.transpose()*dS_dp).transpose());

	dVector vreg; vreg.resize(actCount); vreg.setZero();
	vreg = p * 1.0 + m_p0 * (-1.0); 
	grad = vreg * regularizer + grad * 1.0;

};

//this method gets called whenever a new best solution to the objective function is found
void KS_IKConstraintEnergy::setCurrentBestSolution(const dVector& p){
	ikMechanismController->motorAngleValues = p;
	ikMechanismController->updateMotorConnections();
	updateRegularizingSolutionTo(p);
}


void KS_IKConstraintEnergy::computedO_ds(const dVector& p)
{   
	mechanism->setAssemblyState(currentMechState);
	int eeCompInd= ikMechanismController->eeComponent->getComponentIndex()*stateSize;
	dVector tmp; tmp.resize(stateCount); tmp.setZero();
	tmp[0+ eeCompInd] = (ikMechanismController->gammaEEDOF)*(ikMechanismController->eeComponent->getGamma() - ikMechanismController->gammaEEd);
	tmp[1+ eeCompInd] = (ikMechanismController->betaEEDOF)*(ikMechanismController->eeComponent->getBeta() - ikMechanismController->betaEEd); 
	tmp[2+ eeCompInd] = (ikMechanismController->alphaEEDOF)*(ikMechanismController->eeComponent->getAlpha() - ikMechanismController->alphaEEd); 
	tmp[3+ eeCompInd] = (ikMechanismController->xEEDOF)*(ikMechanismController->eeComponent->getWorldCenterPosition()[0] - ikMechanismController->xEEd);
	tmp[4+ eeCompInd] = (ikMechanismController->yEEDOF)*(ikMechanismController->eeComponent->getWorldCenterPosition()[1] - ikMechanismController->yEEd);
	tmp[5+ eeCompInd] = (ikMechanismController->zEEDOF)*(ikMechanismController->eeComponent->getWorldCenterPosition()[2] - ikMechanismController->zEEd);
	dO_ds = tmp;
	/*for (int i = 0; i < dO_ds.size(); i++) {
		Logger::print("dO_ds %lf\n", dO_ds[i]);
	}*/
}

void KS_IKConstraintEnergy::computedS_dp(const dVector& p)
{
	Eigen::SimplicialLDLT<SparseMatrix, Eigen::Lower> solver;
	//Eigen::SparseLU<SparseMatrix> solver;

	ikMechanismController->motorAngleValues = p;
	ikMechanismController->updateMotorConnections();

	/*dVector currentMechState = mechanism->s;
	DynamicArray<MTriplet> tmPhessianEntries; tmPhessianEntries.clear();
	SparseMatrix ddE_dsds; ddE_dsds.resize(stateCount, stateCount); ddE_dsds.setZero();
	mechanism->AConstraintEnergy->addHessianEntriesTo(tmPhessianEntries, mechCurrentStateAtCurrentIter);
	ddE_dsds.setFromTriplets(tmPhessianEntries.begin(), tmPhessianEntries.end());
	solver.compute(ddE_dsds);*/
	solver.compute(mechanism->AConstraintEnergy->ddE_dsds);
	//computeddAE_ds_dp(p);
	//dS_dp = solver.solve(ddAE_ds_dp);
	computeddAE_ds_dpAnalytic(p);
	dS_dp = solver.solve(ddAE_ds_dpA);
	//dS_dp = ddE_dsds.triangularView<Eigen::Lower>().solve(ddAE_ds_dp);
	//Logger::print("dS_dp norm %lf\n", dS_dp.norm());
}

void KS_IKConstraintEnergy::computeddAE_ds_dpAnalytic(const dVector & p)
{
	ddAE_ds_dpA.resize(stateCount, actCount);
	ddAE_ds_dpA.setZero();
	ikMechanismController->motorAngleValues = p;
	ikMechanismController->updateMotorConnections();
	mechanism->setAssemblyState(currentMechState);
	for (int i = 0; i < actCount; i++) {
		mechanism->actuated_connections[i]->computeddAE_ds_dp();
		if (mechanism->actuated_connections[i]->m_compIn != NULL) {
			int cIn = mechanism->actuated_connections[i]->m_compIn->getComponentIndex()*KS_MechanicalComponent::getStateSize();
			MatrixNxM tmp1 = mechanism->actuated_connections[i]->getdAE_ds_dp1();
			for (int j = 0; j < tmp1.size(); j++) {
				ddAE_ds_dpA(cIn + j, i) += tmp1(0, j);
			}
		}
		int cOut = mechanism->actuated_connections[i]->m_compOut->getComponentIndex()*KS_MechanicalComponent::getStateSize();
		MatrixNxM tmp2 = mechanism->actuated_connections[i]->getdAE_ds_dp2();
		for (int j = 0; j < tmp2.size(); j++) {
			ddAE_ds_dpA(cOut + j, i) += tmp2(0, j);
		}
		
	}
}

void KS_IKConstraintEnergy::computedAE_ds(const dVector & p)
{
	ikMechanismController->motorAngleValues = p;
	ikMechanismController->updateMotorConnections();
	dAE_ds.setZero();
	mechanism->AConstraintEnergy->addGradientTo(dAE_ds, currentMechState);
}

void KS_IKConstraintEnergy::computeddAE_ds_dp(const dVector& p)
{
	ddAE_ds_dp.resize(stateCount, actCount);
	ddAE_ds_dp.setZero();

	dVector pSet = p;

	double dp = 10e-6;
	int pSize = (int)pSet.size();

	for (int i = 0; i<pSize; i++) {
		double tmpVal = pSet[i];
		pSet[i] = tmpVal + dp;
		computedAE_ds(pSet);
		dVector tempGrad1 = dAE_ds;
		pSet[i] = tmpVal - dp;
		computedAE_ds(pSet);
		dVector tempGrad2 = dAE_ds;

		pSet[i] = tmpVal;
		dVector tempGradofGrad = (tempGrad1 - tempGrad2) / (2 * dp);

		for (int j = i; j < stateCount; j++) {
			if (!IS_ZERO(tempGradofGrad[j])) {
				ddAE_ds_dp(j, i) = tempGradofGrad[j];
				//Logger::print("ddAE_ds_dp j:%d i:%d value:%lf\n", j, i, ddAE_ds_dp(j, i));
			}
		}
	}
	//Logger::print("ddAE_ds_dp norm %lf\n", ddAE_ds_dp.norm());
	computedAE_ds(pSet);
}