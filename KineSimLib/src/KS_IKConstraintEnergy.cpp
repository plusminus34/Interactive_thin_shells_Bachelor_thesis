#include <KineSimLib/KS_IKConstraintEnergy.h>
#include <KineSimLib/KS_MechanicalAssembly.h>
#include <KineSimLib/KS_AssemblyConstraintEnergy.h>
#include <KineSimLib/KS_IKMechanismController.h>
#include <KineSimLib/KS_Constraint.h>



KS_IKConstraintEnergy::KS_IKConstraintEnergy(void){
	//regularizer = 0;
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

	dO_ds.resize(stateCount); dO_ds.setZero(); //computeGradofEEObjectiveWithMechanismState
	dAE_ds.resize(stateCount); dAE_ds.setZero(); // gradient of the assembly constraint energy with respect to the mechanism state
	ddAE_ds_dp.resize(stateCount, actCount); ddAE_ds_dp.setZero();//gradiant of the gradient with respect to the control parameters with fixed mechanism state
	dS_dp.resize(stateCount, actCount); dS_dp.setZero();//compute the gradient of the mech state with respect to the control variables 

}

//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
/*void KS_IKConstraintEnergy::updateRegularizingSolutionTo(const dVector &currentS){
	this->m_s0 = currentS;//copy(currentS, m_s0);
}*/

double KS_IKConstraintEnergy::computeValue(const dVector& p) {

	ikMechanismController->motorAngleValues = p;
	ikMechanismController->updateMotorConnections();
	//Logger::print("i am here with p%lf\n",p[0]);
	mechanism->solveAssembly();
	dVector currentMechState = mechanism->s;
	mechCurrentStateAtCurrentIter = currentMechState;
	mechanism->setAssemblyState(currentMechState);
	/*Logger::print("ikMechanismController->eeComponent->getWorldCenterPosition()[1][2] %lf  %lf\n", ikMechanismController->eeComponent->getWorldCenterPosition()[1],
		ikMechanismController->eeComponent->getWorldCenterPosition()[2]);*/
	double totalIkConstraintEnergy = 0;
	totalIkConstraintEnergy +=
	0.5*10*((ikMechanismController->xEEDOF)*SQR(ikMechanismController->eeComponent->getWorldCenterPosition()[0] - ikMechanismController->xEEd) +
		(ikMechanismController->yEEDOF)*SQR(ikMechanismController->eeComponent->getWorldCenterPosition()[1] - ikMechanismController->yEEd) +
		(ikMechanismController->zEEDOF)*SQR(ikMechanismController->eeComponent->getWorldCenterPosition()[2] - ikMechanismController->zEEd) +
		(ikMechanismController->alphaEEDOF)*SQR(ikMechanismController->eeComponent->getAlpha() - ikMechanismController->alphaEEd) +
		(ikMechanismController->betaEEDOF)*SQR(ikMechanismController->eeComponent->getBeta() - ikMechanismController->betaEEd) +
		(ikMechanismController->gammaEEDOF)*SQR(ikMechanismController->eeComponent->getGamma() - ikMechanismController->gammaEEd));
	
	//add the regularizer contribution
	/*dVector vd;
	vd.resize(s.size()); vd.setZero();
	vd = s * 1.0 + m_s0 * (-1);// replaced add function
	double nrmvd2 = vd.squaredNorm();	//double nrmvd2 = dotprod(vd, vd);
	totalConstraintEnergy += 0.5*regularizer*nrmvd2; */
	//Logger::print("ASS energy and IKConstraintEnergy  %lf %lf\n", mechanism->AConstraintEnergy->computeValue(currentMechState), totalIkConstraintEnergy);
		return totalIkConstraintEnergy;
}

void KS_IKConstraintEnergy::addGradientTo(dVector & grad, const dVector & p)
{
	grad.resize(actCount);
	grad.setZero();
	computedO_ds(p);
	computedS_dp(p);
	//Logger::print("dods cols and dsdp rows %d %d\n", dO_ds.cols(), dS_dp.rows());
	grad += -1.0*((dO_ds.transpose()*dS_dp).transpose());
	//grad += (dO_ds*dS_dp).transpose();
	//Logger::print("dsdp first and second column norm and dods norm %lf %lf %Lf\n", dS_dp.col(0).norm(), dS_dp.col(1).norm(), dO_ds.norm());
	/*for (int i = 0; i < dO_ds.size(); i++) {
	Logger::print("dO_ds %lf\n", dO_ds[i]);
	Logger::print("dS_dp col1 %lf\n", dS_dp.col(0)[i]);
	Logger::print("dS_dp col2 %lf\n", dS_dp.col(1)[i]);
	Logger::print("ddAE_ds_dp col1 %lf\n", ddAE_ds_dp.col(0)[i]);
	Logger::print("ddAE_ds_dp col2 %lf\n", ddAE_ds_dp.col(1)[i]);
	}*/

};

//this method gets called whenever a new best solution to the objective function is found
/*void KS_IKConstraintEnergy::setCurrentBestSolution(const dVector& p){
	//assembly->setAssemblyState(s);
	//updateRegularizingSolutionTo(s);
}*/


void KS_IKConstraintEnergy::computedO_ds(const dVector& p)
{
	dVector currentMechState = mechanism->s;
	mechanism->setAssemblyState(mechCurrentStateAtCurrentIter);
	int eeCompInd= ikMechanismController->eeComponent->getComponentIndex()*stateSize;
	dVector tmp; tmp.resize(stateCount); tmp.setZero();
	tmp[0+ eeCompInd] = (ikMechanismController->gammaEEDOF)*(ikMechanismController->eeComponent->getGamma() - ikMechanismController->gammaEEd);
	tmp[1+ eeCompInd] = (ikMechanismController->betaEEDOF)*(ikMechanismController->eeComponent->getBeta() - ikMechanismController->betaEEd); 
	tmp[2+ eeCompInd] = (ikMechanismController->alphaEEDOF)*(ikMechanismController->eeComponent->getAlpha() - ikMechanismController->alphaEEd); 
	tmp[3+ eeCompInd] = 10*(ikMechanismController->xEEDOF)*(ikMechanismController->eeComponent->getWorldCenterPosition()[0] - ikMechanismController->xEEd);
	tmp[4+ eeCompInd] = 10*(ikMechanismController->yEEDOF)*(ikMechanismController->eeComponent->getWorldCenterPosition()[1] - ikMechanismController->yEEd);
	tmp[5+ eeCompInd] = 10*(ikMechanismController->zEEDOF)*(ikMechanismController->eeComponent->getWorldCenterPosition()[2] - ikMechanismController->zEEd);
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
	computeddAE_ds_dp(p);
	dS_dp = solver.solve(ddAE_ds_dp);
	//dS_dp = ddE_dsds.triangularView<Eigen::Lower>().solve(ddAE_ds_dp);
	//Logger::print("dS_dp norm %lf\n", dS_dp.norm());
}

void KS_IKConstraintEnergy::computedAE_ds(const dVector & p)
{
	dVector currentMechState = mechanism->s;// this doesn't change during
	ikMechanismController->motorAngleValues = p;
	ikMechanismController->updateMotorConnections();
	dAE_ds.setZero();
	mechanism->AConstraintEnergy->addGradientTo(dAE_ds, mechCurrentStateAtCurrentIter);
	//dAE_ds = mechanism->AConstraintEnergy->dE_ds; // this doesnt work need to compute the gradient
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