#include "KineSimLib/KS_AssemblyConstraintEnergy.h"
#include "KineSimLib/KS_Constraint.h"
#include "KineSimLib/KS_P2PConstraint.h"
#include "KineSimLib/KS_LockedComponentConstraint.h"
#include "KineSimLib/KS_MechanicalAssembly.h"

KS_AssemblyConstraintEnergy::KS_AssemblyConstraintEnergy(void){
	regularizer = 0;
}

KS_AssemblyConstraintEnergy::~KS_AssemblyConstraintEnergy(void){
}

void KS_AssemblyConstraintEnergy::initialize(KS_MechanicalAssembly* a){
	assert(a != NULL);
	assert(a->getComponentCount() >= 1);
	assembly = a;
	int stateCount = a->getComponentCount() * KS_MechanicalComponent::getStateSize();
	dE_ds.resize(stateCount); dE_ds.setZero(); //setValues(dE_ds, 0);
	ddE_dsds.resize(stateCount, stateCount);
	ddE_dsds.setZero();
	hessianEntries.clear(); 

	constraints.clear();

	for (int i = 0; i<a->getConnectionCount(); i++) {
		a->getConnection(i)->addConstraintsToList(constraints);
	}

	scalarConstraintCount = 0;
	for (uint i = 0; i<constraints.size(); i++) {
		scalarConstraintCount += constraints[i]->getConstraintCount();
	}
	Logger::print(" number of constraints %d\n", scalarConstraintCount);


	m_s0.resize(stateCount);
	assembly->getAssemblyState(m_s0);
}

//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
void KS_AssemblyConstraintEnergy::updateRegularizingSolutionTo(const dVector &currentS){
	this->m_s0 = currentS;//copy(currentS, m_s0);
}

double KS_AssemblyConstraintEnergy::computeValue(const dVector& s){
	assembly->setAssemblyState(s);
	double totalConstraintEnergy = 0;
	for (uint i = 0; i < constraints.size(); i++) {
		totalConstraintEnergy += constraints[i]->getEnergy();
		//Logger::print("getEnergy %d %lf\n", i,constraints[i]->getEnergy());
	}
	
	//add the regularizer contribution
	//dVector vd((int)s.size(), 0);
	dVector vd;
	vd.resize(s.size()); vd.setZero();

	//add(s,1.0,m_s0,-1.0,vd);// 
	vd = s * 1.0 + m_s0 * (-1);// replaced add function
	double nrmvd2 = vd.squaredNorm();	//double nrmvd2 = dotprod(vd, vd);

	totalConstraintEnergy += 0.5*regularizer*nrmvd2;

	return totalConstraintEnergy;
}

void KS_AssemblyConstraintEnergy::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & s)
{
	hessianEntries.clear();
	int stateCount = assembly->getComponentCount() * KS_MechanicalComponent::getStateSize();
	ddE_dsds.resize(stateCount, stateCount);
	ddE_dsds.setZero();
	assembly->setAssemblyState(s);
	//ddE_dsds.setZero();
	for (int i = 0; i<(int)constraints.size(); i++)
		constraints[i]->addEnergyHessianTo(hessianEntries);

	//add the regularizer now... TODO: maybe we should have block handles or something to add to the diagonal faster...
	for (int i = 0; i<stateCount; i++)
		hessianEntries.push_back(MTriplet(i, i, regularizer));
		//ddE_dsds.addToElementAt(i, i, regularizer);
	this->hessianEntries = hessianEntries;
	ddE_dsds.setFromTriplets(hessianEntries.begin(), hessianEntries.end());

	//return &ddE_dsds;
}

void KS_AssemblyConstraintEnergy::addGradientTo(dVector & grad, const dVector & s)
{
	grad.resize(s.size());
	grad.setZero();
	//Logger::print("compute gradient called\n");
	assembly->setAssemblyState(s);
	grad.setZero();//setValues(dE_ds, 0);

	for (uint i = 0; i<constraints.size(); i++)
		constraints[i]->addEnergyGradientTo(grad);

	//add the regularizer now... 
	//dVector vreg((int)s.size(), 0);
	dVector vreg; vreg.resize(s.size()); vreg.setZero();

	//add(s,1.0,m_s0,-1.0,vreg);
	vreg = s * 1.0 + m_s0 * (-1.0); //replaced add
									//add(vreg,regularizer,dE_ds,1.0,dE_ds);
	grad = vreg * regularizer + grad * 1.0;//replaced add

	dE_ds = grad;
	/*for (int i = 0; i < s.size(); i++) {
		Logger::print("gradient %lf\n", grad[i]);
	}*/

}

//this method gets called whenever a new best solution to the objective function is found
void KS_AssemblyConstraintEnergy::setCurrentBestSolution(const dVector& s){
	assembly->setAssemblyState(s);
	updateRegularizingSolutionTo(s);
}
