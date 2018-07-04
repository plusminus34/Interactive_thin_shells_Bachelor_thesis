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
		for (int i=0;i<KS_MechanicalComponent::getStateSize();i++)
			gradient[KS_MechanicalComponent::getStateSize() * c->getComponentIndex() + i] += (*get_dE_dsi(j))[i];
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

/*void KS_Constraint::addEnergyHessianTo(SparseMatrix& hessian){
	if (hBlocks.size() == 0)
		for (int i=0;i<getNumberOfAffectedComponents()*getNumberOfAffectedComponents();i++)
			hBlocks.push_back(SparseMatrixBlockHandle());

	computeEnergyHessian();

	//write out the hessian blocks
	for (int i=0;i<getNumberOfAffectedComponents();i++)
		for (int j=0;j<getNumberOfAffectedComponents();j++){
			int hBlockIndex = i*getNumberOfAffectedComponents()+j;
			if (hBlocks[hBlockIndex].isInitialized() == false) hBlocks[hBlockIndex].createBlock(&hessian, KS_MechanicalComponent::getStateSize() * getIthAffectedComponent(i)->getComponentIndex(), KS_MechanicalComponent::getStateSize() * getIthAffectedComponent(j)->getComponentIndex(), KS_MechanicalComponent::getStateSize(), KS_MechanicalComponent::getStateSize());
			assert(hBlockIndex<(int)hBlocks.size());
			hBlocks[hBlockIndex].addBlockValues(&hessian, get_ddE_dsidsj(i,j)->getData());
		}
}*/

//writes out the values of the current constraints at the appropriate location in the constraint vector
void KS_Constraint::writeConstraintValuesTo(dVector& C){
	dVector* constraintValues = getConstraintValues();
	for (uint i=0;i<constraintValues->size();i++){
		C[i + constraintStartIndex] = (*constraintValues)[i];
	}
}

void KS_Constraint::writeConstraintJacobianValuesTo(DynamicArray<MTriplet>& dCdsEntries)
{
	computeConstraintJacobian();
	//write out the constraint jacobian values
	for (int i = 0; i<getNumberOfAffectedComponents(); i++) {
		int startRow = constraintStartIndex;
		int startCol = KS_MechanicalComponent::getStateSize() * getIthAffectedComponent(i)->getComponentIndex();
		addSparseMatrixDenseBlockToTriplet(dCdsEntries, startRow, startCol, *getConstraintJacobian(i), true);
	}
}

//writes out the values of the jacobian of the constraint vector at the appropriate location in the given sparse matrix
/*void KS_Constraint::writeConstraintJacobianValuesTo(SparseMatrix& dCds){
	if (dCdsBlocks.size() == 0)
		for (int i=0;i<getNumberOfAffectedComponents();i++)
			dCdsBlocks.push_back(SparseMatrixBlockHandle());

	computeConstraintJacobian();

	//write out the constraint jacobian values
	for (int i=0;i<getNumberOfAffectedComponents();i++){
		if (dCdsBlocks[i].isInitialized() == false) dCdsBlocks[i].createBlock(&dCds, constraintStartIndex, KS_MechanicalComponent::getStateSize() * getIthAffectedComponent(i)->getComponentIndex(), getConstraintCount(), KS_MechanicalComponent::getStateSize());
		assert(i<(int)dCdsBlocks.size());
		dCdsBlocks[i].setBlockValues(&dCds, getConstraintJacobian(i)->getData());
	}
}*/

/*void KS_Constraint::cleanSparseMatrixBlocks(){
	hBlocks.clear();
	dCdsBlocks.clear();
	dCdsiBlocks.clear();
}*/


void KS_Constraint::setAffectedComponentsState(const dVector& state){
	for (int i=0;i<getNumberOfAffectedComponents();i++){
		KS_MechanicalComponent* c = getIthAffectedComponent(i);
		c->setAngles(state[KS_MechanicalComponent::getStateSize() * i + 0], state[KS_MechanicalComponent::getStateSize() * i + 1], state[KS_MechanicalComponent::getStateSize() * i + 2]);
		c->setWorldCenterPosition(P3D(state[KS_MechanicalComponent::getStateSize() * i + 3], state[KS_MechanicalComponent::getStateSize() * i + 4], state[KS_MechanicalComponent::getStateSize() * i + 5]));
	}
}

/*void KS_Constraint::testEnergyGradientAndHessian(){
	computeEnergyGradient();
	computeEnergyHessian();
	computeConstraintJacobian();


	int stateSize = getNumberOfAffectedComponents()*KS_MechanicalComponent::getStateSize();
	//dVector state(stateSize, 0);
	dVector state;
	state.resize(stateSize); state.setZero();
	for (int i=0;i<getNumberOfAffectedComponents();i++){
		KS_MechanicalComponent* c = getIthAffectedComponent(i);
		int i0 = i* KS_MechanicalComponent::getStateSize();
		state[i0+0] = c->getGamma();
		state[i0+1] = c->getBeta();
		state[i0+2] = c->getAlpha();
		//state[i0+3] = c->getWorldCenterPosition().x;
		state[i0 + 3] = c->getWorldCenterPosition()[0];
		//state[i0+4] = c->getWorldCenterPosition().y;
		state[i0 + 4] = c->getWorldCenterPosition()[1];
		//state[i0+5] = c->getWorldCenterPosition().z;
		state[i0 + 5] = c->getWorldCenterPosition()[2];

	}
	
	//now we need to do the finite differences in order to estimate the gradient...
	double ds = 0.00001;
	dVector gradient((int)state.size(), 0);
	for (uint i=0; i<state.size();i++){
		double pValue = state[i];
		state[i] = pValue + ds;
		setAffectedComponentsState(state);
		double nrgP = getEnergy();

		state[i] = pValue - ds;
		setAffectedComponentsState(state);
		double nrgM = getEnergy();

		//now compute the derivative using central finite differences
		gradient[i] = (nrgP - nrgM) / (2*ds);

		//now reset the state
		state[i] = pValue;
		setAffectedComponentsState(state);
	}

	setAffectedComponentsState(state);

	Logger::print("Checking gradient!\n");

	//now we can compute the error in the gradient...
	for (int i=0;i<getNumberOfAffectedComponents();i++){
		dVector* analyticGradient = get_dE_dsi(i);
		for (int j=0;j<KS_MechanicalComponent::getStateSize(); j++){
			if (fabs((*analyticGradient)[j] - gradient[KS_MechanicalComponent::getStateSize()*i+j]) > 0.0001){
				Logger::print("Mismatch in gradient at(%d)! analytic: %lf, fd:%lf, error: %lf\n", KS_MechanicalComponent::getStateSize()*i+j, (*analyticGradient)[j], gradient[KS_MechanicalComponent::getStateSize()*i+j], fabs((*analyticGradient)[j] - gradient[KS_MechanicalComponent::getStateSize()*i+j]));
			}
		}
	}

	MatrixNxM hessian;
	hessian.resize(KS_MechanicalComponent::getStateSize() * getNumberOfAffectedComponents(), KS_MechanicalComponent::getStateSize() * getNumberOfAffectedComponents());
	hessian.setZero();
	dVector gradP, gradM;
	gradP.resize(gradient.size(), 0);
	gradM.resize(gradient.size(), 0);
	//now we need to check the hessian - we'll do finite differences on the gradient of the energy
	for (uint i=0; i<state.size();i++){
		double pValue = state[i];
		state[i] = pValue + ds;
		setAffectedComponentsState(state);
		computeEnergyGradient();
		for (int j=0;j<getNumberOfAffectedComponents();j++){
			dVector* analyticGradient = get_dE_dsi(j);
			for (int k=0;k<KS_MechanicalComponent::getStateSize(); k++)
				gradP[KS_MechanicalComponent::getStateSize()*j+k] = (*analyticGradient)[k];
		}

		state[i] = pValue - ds;
		setAffectedComponentsState(state);
		computeEnergyGradient();
		for (int j=0;j<getNumberOfAffectedComponents();j++){
			dVector* analyticGradient = get_dE_dsi(j);
			for (int k=0;k<KS_MechanicalComponent::getStateSize(); k++)
				gradM[KS_MechanicalComponent::getStateSize()*j+k] = (*analyticGradient)[k];
		}

		//now compute the derivative using central finite differences
		for (uint j=0;j<gradM.size();j++)
			hessian(i, j) = (gradP[j] - gradM[j]) / (2*ds);

		//now reset the state
		state[i] = pValue;
		setAffectedComponentsState(state);
	}

	setAffectedComponentsState(state);

	//now compare the hessians...
	Logger::print("Checking hessian!\n");
	for (int i=0;i<hessian.rows();i++){
		for (int j=0;j<hessian.cols();j++){
			double valAnalytic = get_ddE_dsidsj(i/KS_MechanicalComponent::getStateSize(), j/KS_MechanicalComponent::getStateSize())->operator () (i % KS_MechanicalComponent::getStateSize(), j % KS_MechanicalComponent::getStateSize());
			if (fabs(hessian(i,j) - valAnalytic) > 0.0001){
				Logger::print("Mismatch in hessian at (%d, %d)! analytic: %lf, fd:%lf, error: %lf\n", i,j,valAnalytic, hessian(i,j), fabs(hessian(i,j) - valAnalytic));
			}
		}
	}
}*/

