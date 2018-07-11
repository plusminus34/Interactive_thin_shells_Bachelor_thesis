#include "KineSimLib/KS_Constraint.h"


KS_Constraint::KS_Constraint(void){
}

KS_Constraint::~KS_Constraint(void){
}

void KS_Constraint::addEnergyGradientTo(dVector& gradient){
	computeEnergyGradient();
	//compute the gradient, and write it out
	for (int j=0;j<getNumberOfAffectedComponents();j++){
		KS_MechanicalComponent* c = getIthAffectedComponent(j);
		for (int i = 0; i < KS_MechanicalComponent::getStateSize(); i++) {
			gradient[KS_MechanicalComponent::getStateSize() * c->getComponentIndex() + i] += (*get_dE_dsi(j))[i];
			//Logger::print("gradient %d %lf\n", i, (*get_dE_dsi(j))[i]);
		}

	}
}

void KS_Constraint::addEnergyHessianTo(DynamicArray<MTriplet>& hessianEntries)
{
	computeEnergyHessian();

	//write out the hessian blocks
	for (int i = 0; i<getNumberOfAffectedComponents(); i++)
		for (int j = 0; j<getNumberOfAffectedComponents(); j++) {
			int startRow = KS_MechanicalComponent::getStateSize() * getIthAffectedComponent(i)->getComponentIndex();
			int startCol = KS_MechanicalComponent::getStateSize() * getIthAffectedComponent(j)->getComponentIndex();
			addSparseMatrixDenseBlockToTriplet(hessianEntries, startRow, startCol, *get_ddE_dsidsj(i, j), true);
		}

}


void KS_Constraint::setAffectedComponentsState(const dVector& state){
	for (int i=0;i<getNumberOfAffectedComponents();i++){
		KS_MechanicalComponent* c = getIthAffectedComponent(i);
		c->setAngles(state[KS_MechanicalComponent::getStateSize() * i + 0], state[KS_MechanicalComponent::getStateSize() * i + 1], state[KS_MechanicalComponent::getStateSize() * i + 2]);
		c->setWorldCenterPosition(P3D(state[KS_MechanicalComponent::getStateSize() * i + 3], state[KS_MechanicalComponent::getStateSize() * i + 4], state[KS_MechanicalComponent::getStateSize() * i + 5]));
	}
}

