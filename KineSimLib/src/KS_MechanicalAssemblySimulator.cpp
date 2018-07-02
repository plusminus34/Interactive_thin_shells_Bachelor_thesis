#include "KineSimLib/KS_MechanicalAssemblySimulator.h"

KS_MechanicalAssemblySimulator::KS_MechanicalAssemblySimulator(void){
	energyFunction = NULL;
	minimizer = NULL;
}

KS_MechanicalAssemblySimulator::~KS_MechanicalAssemblySimulator(void){
	delete energyFunction;
	delete minimizer;
}

//assume the driver input (or other constraints are updated elsewhere)
double KS_MechanicalAssemblySimulator::solve(double regularizer){
	energyFunction->setRegularizer(regularizer);
	energyFunction->assembly->getAssemblyState(s);
	double val;
	minimizer->minimize(energyFunction, s, val);
	energyFunction->assembly->setAssemblyState(s);
	return val;
}

int KS_MechanicalAssemblySimulator::getLastSolverIterations() {
	return minimizer->getIterationsUsed();
}
/*
Not yet tested....
//this method computes the jacobian that tells us how the state changes with the ticker. It helps to estimate velocities of specific points on the assembly, for instance...
void KS_MechanicalAssemblySimulator::compute_dsdTicker(dVector* assemblyState, double correspondingTickerValue, dVector* dsdTicker){
	//save the current state
	energyFunction->assembly->getAssemblyState(s);
	double currentTickerValue = energyFunction->assembly->getTickerValue();

	//now set the input state and corresponding ticker...
	int n = (int)assemblyState->size();							//state size
	int m = (int)energyFunction->getScalarConstraintCount();	//number of constraints

	//ds/dp = - dC/ds^-1 * dC/dp, but we will use the pseudo inverse because we may have more constraints than state variables (although they are redundant)
	dCds.resize(m, n, false);
	dCds.zero();
	energyFunction->assembly->setTickerValue(correspondingTickerValue);
	energyFunction->precomputeDerivativeInformationAt(*assemblyState);
	dCds.add(*energyFunction->getCurrentConstraintJacobian());
	dCdst_dCds.setToAtA(dCds);

	double dp = 10e-5;
	dVector C_P(m,0), C_M(m,0), dCdp(m,0);
	dVector b;

	b.resize(n);
	dsdTicker->resize(n);
	energyFunction->assembly->setTickerValue(correspondingTickerValue + dp);
	energyFunction->computeValue(*assemblyState);
	copy(*energyFunction->getCurrentConstraintVector(), C_P);

	energyFunction->assembly->setTickerValue(correspondingTickerValue - dp);
	energyFunction->computeValue(*assemblyState);
	copy(*energyFunction->getCurrentConstraintVector(), C_M);

	add(C_P, -1.0/(2*dp), C_M, 1.0/(2*dp), dCdp);

	//each vector is a column vector of b, and store b in column-major...
	dCds.multVectorTransposed(dCdp, b);

	//dqdqp will be stored in column-major order
	linSolver.solveLinearSystem(dCdst_dCds, &((*dsdTicker)[0]), &b[0], 1);

	//and now reset the state to what it was in the beginning...
	energyFunction->assembly->setAssemblyState(s);
	energyFunction->assembly->setTickerValue(currentTickerValue);
}
*/
void KS_MechanicalAssemblySimulator::initialize(KS_MechanicalAssembly* a, double solverResidual){
	delete energyFunction;
	energyFunction = new KS_AssemblyConstraintEnergy();
	energyFunction->initialize(a);
	delete minimizer;
	minimizer = new NewtonFunctionMinimizer(50, solverResidual,30);
//	minimizer->printOutput = true;
}

void KS_MechanicalAssemblySimulator::testGradientsAndHessians(){
	//now test out the whole gradient/hessian
	dVector currentState;
	energyFunction->assembly->getAssemblyState(currentState);

	for (uint i=0;i<energyFunction->constraints.size();i++)
		energyFunction->constraints[i]->testEnergyGradientAndHessian();	

	logPrint("Checking full energy gradient...\n");

	int size = (int)currentState.size();

	dVector gradient = *energyFunction->getGradientAt(currentState);

	//now we need to do the finite differences in order to estimate the gradient...
	double ds = 0.00001;
	for (int i=0; i<size; i++){
		double tmp = currentState[i];
		currentState[i] = tmp + ds;
		double nrgP = energyFunction->computeValue(currentState);

		currentState[i] = tmp - ds; 
		double nrgM = energyFunction->computeValue(currentState);

		//now compute the derivative using central finite differences
		double error = gradient.at(i) -  (nrgP - nrgM) / (2*ds);
		if (fabs(error) > 0.0001)
			logPrint("gradient error at %d: %lf (%lf vs %lf)\n", i, error, gradient.at(i), (nrgP - nrgM) / (2*ds));

		//now reset the state
		currentState[i] = tmp;
		energyFunction->assembly->setAssemblyState(currentState);
	}

	logPrint("Checking full energy hessian...\n");

	SparseMatrix* hes = energyFunction->getHessianAt(currentState);

	//now we need to do the finite differences in order to estimate the hessian...
	dVector gP(size,0), gM(size,0);
	for (int i=0; i<size; i++){
		double tmp = currentState[i];
		currentState[i] = tmp + ds;
		copy(*energyFunction->getGradientAt(currentState), gP);

		currentState[i] = tmp - ds; 
		copy(*energyFunction->getGradientAt(currentState), gM);

		//now reset the state
		currentState[i] = tmp;

		//now compute the derivative using central finite differences
		for (int j=0;j<size;j++){
			double error = hes->getElementAt(i,j) -  (gP[j] - gM[j]) / (2*ds);
			if (fabs(error) > 0.0001)
				logPrint("hessian error at %d %d: %lf. Value: %lf\n", i, j, error, hes->getElementAt(i,j));
		}
	}

	logPrint("Checking constraint jacobian...\n");

	SparseMatrix* jacobian = energyFunction->getConstraintJacobianAt(currentState);

	//now we need to do the finite differences in order to estimate the hessian...
	dVector C_P(energyFunction->getScalarConstraintCount(),0), C_M(energyFunction->getScalarConstraintCount(),0);
	for (int i=0; i<size; i++){
		double tmp = currentState[i];
		currentState[i] = tmp + ds;
		copy(*energyFunction->getConstraintVectorAt(currentState), C_P);

		currentState[i] = tmp - ds;
		copy(*energyFunction->getConstraintVectorAt(currentState), C_M);

		//now reset the state
		currentState[i] = tmp;
		energyFunction->computeValue(currentState);

		//now compute the derivative using central finite differences
		for (int j=0;j<energyFunction->getScalarConstraintCount();j++){
			double error = jacobian->getElementAt(j, i) -  (C_P[j] - C_M[j]) / (2*ds);
			if (fabs(error) > 0.0001)
				logPrint("jacobian constraint error at %d %d: %lf vs %lf. Error: %lf\n", i, j, jacobian->getElementAt(j,i), (C_P[j] - C_M[j]) / (2*ds), error);
		}
	}
}

int KS_MechanicalAssemblySimulator::getNumberOfScalarConstraints(){
	if (energyFunction!=NULL)
		return energyFunction->getScalarConstraintCount();
	return 0;
}

SparseMatrix* KS_MechanicalAssemblySimulator::getCurrentConstraintJacobian(){
	if(energyFunction !=NULL){
		dVector currentState;
		energyFunction->assembly->getAssemblyState(currentState);
		return energyFunction->getConstraintJacobianAt(currentState);	
	}else 
		return (SparseMatrix*) NULL;
}

void KS_MechanicalAssemblySimulator::getPlanarConstraintJacobianAt(Matrix &J){
	if(energyFunction !=NULL){
		dVector currentState;
		energyFunction->assembly->getAssemblyState(currentState);
		energyFunction->getPlanarConstraintJacobianAt(currentState, J);	
	}
}



