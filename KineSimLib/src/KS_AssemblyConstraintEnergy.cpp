#include "KineSimLib/KS_AssemblyConstraintEnergy.h"
#include "KineSimLib/KS_Constraint.h"
#include "KineSimLib/KS_P2PConstraint.h"
#include "KineSimLib/KS_LockedComponentConstraint.h"
#include "KineSimLib/KS_MechanicalAssembly.h"
//#include "KineSimLib/KS_PhaseToPhaseConstraint.h"
//#include "KineSimLib/KS_PhaseConstraint.h"

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
	//ddE_dsds.resize(stateCount, stateCount, true);
	ddE_dsds.setZero();

	constraints.clear();

	for (int i=0; i<a->getConnectionCount(); i++){
		a->getConnection(i)->addConstraintsToList(constraints);
	}

	scalarConstraintCount = 0;
	for (uint i=0;i<constraints.size();i++){
		//constraints[i]->cleanSparseMatrixBlocks();
		constraints[i]->setConstraintStartIndex(scalarConstraintCount);
		scalarConstraintCount += constraints[i]->getConstraintCount();
	}

	C.resize(scalarConstraintCount);
	//dCds.resize(scalarConstraintCount, a->getComponentCount() * KS_MechanicalComponent::getStateSize(), false);
	//dCds.resize(scalarConstraintCount, a->getComponentCount() * KS_MechanicalComponent::getStateSize());
	dCdsEntries.clear();
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
	for (uint i=0;i<constraints.size();i++)
		totalConstraintEnergy += constraints[i]->getEnergy();
	
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

/*SparseMatrix* KS_AssemblyConstraintEnergy::getHessianAt(const dVector& s){
	assembly->setAssemblyState(s);
	ddE_dsds.setZero();
	for (int i=0;i<(int)constraints.size();i++)
		constraints[i]->addEnergyHessianTo(ddE_dsds);

	//add the regularizer now... TODO: maybe we should have block handles or something to add to the diagonal faster...
	for (int i=0; i<ddE_dsds.rows();i++)
		ddE_dsds.addToElementAt(i, i, regularizer);

	return &ddE_dsds;
}*/

/*dVector* KS_AssemblyConstraintEnergy::getGradientAt(const dVector& s){
	assembly->setAssemblyState(s);
	dE_ds.setZero();//setValues(dE_ds, 0);

	for (uint i=0;i<constraints.size();i++)
		constraints[i]->addEnergyGradientTo(dE_ds);

	//add the regularizer now... 
	dVector vreg((int)s.size(), 0);
	//add(s,1.0,m_s0,-1.0,vreg);
	vreg = s * 1.0 + m_s0 * (-1.0); //replaced add
	//add(vreg,regularizer,dE_ds,1.0,dE_ds);
	dE_ds = vreg * regularizer + dE_ds * 1.0;//replaced add

	return &dE_ds;
}*/

void KS_AssemblyConstraintEnergy::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & s)
{
	assembly->setAssemblyState(s);
	//ddE_dsds.setZero();
	for (int i = 0; i<(int)constraints.size(); i++)
		constraints[i]->addEnergyHessianTo(hessianEntries);

	//add the regularizer now... TODO: maybe we should have block handles or something to add to the diagonal faster...
	for (int i = 0; i<ddE_dsds.rows(); i++)
		hessianEntries.push_back(MTriplet(i, i, regularizer));
		//ddE_dsds.addToElementAt(i, i, regularizer);

	//return &ddE_dsds;
}

void KS_AssemblyConstraintEnergy::addGradientTo(dVector & grad, const dVector & s)
{
	assembly->setAssemblyState(s);
	dE_ds.setZero();//setValues(dE_ds, 0);

	for (uint i = 0; i<constraints.size(); i++)
		constraints[i]->addEnergyGradientTo(dE_ds);

	//add the regularizer now... 
	//dVector vreg((int)s.size(), 0);
	dVector vreg; vreg.resize(s.size()); vreg.setZero();

	//add(s,1.0,m_s0,-1.0,vreg);
	vreg = s * 1.0 + m_s0 * (-1.0); //replaced add
									//add(vreg,regularizer,dE_ds,1.0,dE_ds);
	dE_ds = vreg * regularizer + dE_ds * 1.0;//replaced add

	grad = dE_ds;

}

//this method gets called whenever a new best solution to the objective function is found
void KS_AssemblyConstraintEnergy::setCurrentBestSolution(const dVector& s){
	assembly->setAssemblyState(s);
	updateRegularizingSolutionTo(s);
}

dVector* KS_AssemblyConstraintEnergy::getConstraintVectorAt(const dVector& s){
	assembly->setAssemblyState(s);
	for (uint i=0;i<constraints.size();i++)
		constraints[i]->writeConstraintValuesTo(C);
	return &C;
}


void KS_AssemblyConstraintEnergy::getConstraintJacobianAt(const dVector& s){
	assembly->setAssemblyState(s);
	//dCds.setZero();
	for (uint i=0;i<constraints.size();i++)
		constraints[i]->writeConstraintJacobianValuesTo(dCdsEntries);
	//return &dCds;
}

/*void KS_AssemblyConstraintEnergy::getPlanarConstraintJacobianAt(const dVector& s, Matrix &J){
	getConstraintJacobianAt(s);

	J.resize(dCds.getNumCols()/2,dCds.getNumCols()/2);
	J.setToZeros();
	int currentRow=0;
	for(uint i=0;i<constraints.size();i++){
		KS_P2PConstraint* p2pconstraint = dynamic_cast<KS_P2PConstraint*>(constraints[i]);
		if(p2pconstraint){
			for(int j=0;j<assembly->getComponentCount();j++)
				for(int k=0;k<3;k++)
					for(int l=0;l<2;l++)
						J(currentRow+l,3*j+k)=dCds.getElementAt(p2pconstraint->getConstraintStartIndex()+l,6*j+2+k);
			currentRow+=2;	
		}else{
			KS_PhaseToPhaseConstraint* ph2phconstraint = dynamic_cast<KS_PhaseToPhaseConstraint*>(constraints[i]);
			if(ph2phconstraint){
				for(int j=0;j<assembly->getComponentCount();j++)
					for(int k=0;k<3;k++)
						J(currentRow,3*j+k)=dCds.getElementAt(ph2phconstraint->getConstraintStartIndex(),6*j+2+k);
				currentRow+=1;
			}else{
				KS_PhaseConstraint* phconstraint = dynamic_cast<KS_PhaseConstraint*>(constraints[i]);
				if(phconstraint){
					for(int j=0;j<assembly->getComponentCount();j++)
						for(int k=0;k<3;k++)
							J(currentRow,3*j+k)=dCds.getElementAt(phconstraint->getConstraintStartIndex(),6*j+2+k);
					currentRow+=1;
				}else{
					KS_LockedComponentConstraint* lcc = dynamic_cast<KS_LockedComponentConstraint*>(constraints[i]);
					if(lcc){
						int freezePhase=0;
						if(lcc->getConstraintCount()==6){
							for(int j=0;j<assembly->getComponentCount();j++)
								for(int k=0;k<3;k++)	
									J(currentRow,3*j+k)=dCds.getElementAt(lcc->getConstraintStartIndex()+2,6*j+2+k);
							currentRow+=1;
							freezePhase=1;
						}
						for(int j=0;j<assembly->getComponentCount();j++)
							for(int k=0;k<3;k++)
								for(int l=0;l<2;l++)
									J(currentRow+l,3*j+k)=dCds.getElementAt(lcc->getConstraintStartIndex()+2+freezePhase+l,6*j+2+k);
						currentRow+=2;
					}
				}
			}
		}
	}

}*/