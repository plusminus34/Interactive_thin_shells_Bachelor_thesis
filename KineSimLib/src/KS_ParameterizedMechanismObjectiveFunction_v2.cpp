#include "KineSimLib/KS_ParameterizedMechanismObjectiveFunction_v2.h"


KS_ParameterizedMechanismObjectiveFunction_v2::KS_ParameterizedMechanismObjectiveFunction_v2(KS_ParameterizedMechanicalAssembly* mechanism, int numberOfEvaluationPoints, double solveRegularizer){
	this->parameterizedAssembly = mechanism;
	this->numberOfEvaluationPoints = numberOfEvaluationPoints;
	this->startTickerValue = mechanism->getCurrentMechanicalAssembly()->getTickerValue();
	mechanism->getCurrentMechanicalAssembly()->getAssemblyState(this->startState);
	energyWeight = 10000000.0;
	condNumberWeight = 100.0;
	this->solveRegularizer = solveRegularizer;
}

KS_ParameterizedMechanismObjectiveFunction_v2::~KS_ParameterizedMechanismObjectiveFunction_v2(){
}

//checks to make sure that the solution is valid... in particular, the states should not "flip", and each of the driving assemblies should be moving strictly forward...
bool KS_ParameterizedMechanismObjectiveFunction_v2::solutionIsValid(const dVector& safeAssemblyState, const dVector& newAssemblyState){
	dVector startState;
	parameterizedAssembly->getCurrentMechanicalAssembly()->getAssemblyState(startState);

	int nComponents = parameterizedAssembly->getCurrentMechanicalAssembly()->getComponentCount();
	int nConnections = parameterizedAssembly->getCurrentMechanicalAssembly()->getConnectionCount();
	dVector componentPhases_safe(nComponents, 0), componentPhases_new(nComponents, 0);

	double maxPhaseDifference = 0;
	double averagePhaseDifference = 0;

	parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(safeAssemblyState);
	for (int j=0; j < nComponents; j++)
		componentPhases_safe[j] = parameterizedAssembly->getCurrentMechanicalAssembly()->getComponent(j)->getPhase();

	parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(newAssemblyState);
	for (int j=0; j < nComponents; j++)
		componentPhases_new[j] = parameterizedAssembly->getCurrentMechanicalAssembly()->getComponent(j)->getPhase();

	for (int j = 0; j < nComponents; j++){
		double phaseDifference = fabs(componentPhases_safe[j] - componentPhases_new[j]);
		if (maxPhaseDifference < phaseDifference)
			maxPhaseDifference = phaseDifference;
		averagePhaseDifference += phaseDifference / (nComponents);
	}
	parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(startState);


	bool mightHaveFlipped = maxPhaseDifference > (PI / 2.0);

	if (mightHaveFlipped)
		logPrint("FAIL: States might have flipped... (%lf)\n", maxPhaseDifference);
	
	return mightHaveFlipped == false;
}

bool KS_ParameterizedMechanismObjectiveFunction_v2::computeAssemblyMotionGivenParameters(const dVector &p, DynamicArray<dVector> *stateArray, DynamicArray<double> *tickerValueArray){
	Logger::printStatic("computing a motion!\n");
	
	parameterizedAssembly->setParameters(p);
	stateArray->clear();
	tickerValueArray->clear();

	parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(this->startState);
	parameterizedAssembly->getCurrentMechanicalAssembly()->setTickerValue(this->startTickerValue);

	bool allConverged = true;
	//collect the states that we sample the objective function at
	for (int i=0; i<numberOfEvaluationPoints; i++){

		double energyVal = parameterizedAssembly->getCurrentMechanicalSimulator()->solve(solveRegularizer);
		if (energyVal > 1e-1)
			return false;

		stateArray->push_back(dVector());
		parameterizedAssembly->getCurrentMechanicalAssembly()->getAssemblyState(stateArray->at(i));
		tickerValueArray->push_back(parameterizedAssembly->getCurrentMechanicalAssembly()->getTickerValue());

/*
		Logger::printStatic("Solver energy: %e\t", energyVal);
		Logger::printStatic("read energy: %e\t", parameterizedAssembly->getCurrentMechanicalSimulator()->energyFunction->computeValue(stateArray->at(i)));
		parameterizedAssembly->getCurrentMechanicalAssembly()->setTickerValue(tickerValueArray->at(i));
		double assemblyEnergy = parameterizedAssembly->getCurrentMechanicalSimulator()->energyFunction->computeValue(stateArray->at(i));
		parameterizedAssembly->getCurrentMechanicalSimulator()->energyFunction->setRegularizer(0);
		Logger::printStatic("and then again energy: %e\n", parameterizedAssembly->getCurrentMechanicalSimulator()->energyFunction->computeValue(stateArray->at(i)));
		parameterizedAssembly->getCurrentMechanicalSimulator()->energyFunction->setRegularizer(solveRegularizer);
*/


		if (i > 0 && solutionIsValid(stateArray->at(i-1), stateArray->at(i)) == false)
			return false; 

		parameterizedAssembly->getCurrentMechanicalAssembly()->stepAssembly(true);
	}

	parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(this->startState);
	parameterizedAssembly->getCurrentMechanicalAssembly()->setTickerValue(this->startTickerValue);

	return allConverged;
}


void KS_ParameterizedMechanismObjectiveFunction_v2::setCurrentBestSolution(const dVector& p){
	//sync the assembly with this new set of parameters
	parameterizedAssembly->setParameters(p);

//	should also check the distance between the first and last states... if they are large, it's not very good...

	parameterizedAssembly->getCurrentMechanicalAssembly()->clearTracerParticles();
	parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(this->startState);
	parameterizedAssembly->getCurrentMechanicalAssembly()->setTickerValue(this->startTickerValue);
	if (parameterizedAssembly->getCurrentMechanicalSimulator()->solve(solveRegularizer) < 1e-7){
		parameterizedAssembly->getCurrentMechanicalAssembly()->getAssemblyState(this->startState);
	}

	//make sure we don't do any extra work if we've already computed the motion for these parameters
	if (p.isSameAs(best_parameterValues))
		return;

	bool solutionValid = true;
	//make sure we don't do any extra work if we've already computed the motion for these parameters
	if (p.isSameAs(tmp_currentParameterSet) == false){
		tmp_currentParameterSet = p;
		solutionValid = computeAssemblyMotionGivenParameters(tmp_currentParameterSet, &tmp_assemblyStateArray, &tmp_correspondingTickerValues);
		parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(this->startState);
		parameterizedAssembly->getCurrentMechanicalAssembly()->setTickerValue(this->startTickerValue);
	}
	solutionValid = solutionValid && tmp_assemblyStateArray.size() == numberOfEvaluationPoints;

	if(!solutionValid)
		return;

	best_parameterValues = tmp_currentParameterSet;

	best_assemblyStateArray = tmp_assemblyStateArray;
	best_correspondingTickerValues = tmp_correspondingTickerValues;

	//tidy up a little...
	parameterizedAssembly->getCurrentMechanicalAssembly()->setTickerValue(this->startTickerValue);
	parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(this->startState);
	parameterizedAssembly->saveParameterSet("../data/KS/aTestForLinkages/new/bestCurrentPset.pset");

}

//this should always return the current value of the objective function
double KS_ParameterizedMechanismObjectiveFunction_v2::computeValue(double const *p){
	dVector newParams(parameterizedAssembly->getParameterCount(), 0);
	for (uint i=0;i<newParams.size();i++)
		newParams[i] = p[i];

	if (newParams.isSameAs(tmp_currentParameterSet)){
		if (tmp_assemblyStateArray.size() != numberOfEvaluationPoints)
			return DBL_MAX;
	}else if(newParams.isSameAs(best_parameterValues)){
		tmp_currentParameterSet = best_parameterValues;
		tmp_assemblyStateArray = best_assemblyStateArray;
		tmp_correspondingTickerValues = best_correspondingTickerValues;
	} else{ //compute the motion, given that these are new parameters...
		tmp_currentParameterSet = newParams;
		if (!computeAssemblyMotionGivenParameters(tmp_currentParameterSet, &tmp_assemblyStateArray, &tmp_correspondingTickerValues)){
			parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(startState);
			parameterizedAssembly->getCurrentMechanicalAssembly()->setTickerValue(startTickerValue);
			return DBL_MAX;
		}
	}

//	return 0;

	double val = 0.0;
	//collect the states that we sample the objective function at
	for (int i=0; i<(int)tmp_assemblyStateArray.size(); i++){
		parameterizedAssembly->getCurrentMechanicalAssembly()->setTickerValue(tmp_correspondingTickerValues[i]);
		parameterizedAssembly->getCurrentMechanicalSimulator()->energyFunction->setRegularizer(0);
		double assemblyEnergy = parameterizedAssembly->getCurrentMechanicalSimulator()->energyFunction->computeValue(tmp_assemblyStateArray[i]);
		parameterizedAssembly->getCurrentMechanicalSimulator()->energyFunction->precomputeDerivativeInformationAt(tmp_assemblyStateArray[i]);
		parameterizedAssembly->getCurrentMechanicalSimulator()->energyFunction->setRegularizer(solveRegularizer);

		SVDFactorization svd;
		SparseMatrix U,S,V;
//		svd.computeSVD(*parameterizedAssembly->getCurrentMechanicalSimulator()->getCurrentConstraintJacobian(),U,S,V);

		Matrix J;
		parameterizedAssembly->getCurrentMechanicalSimulator()->getCurrentReducedConstraintJacobian(J);
/*
		for (int i=0;i<J.getNumRows();i++){
			double rowLen = 0;
			for (int j=0;j<J.getNumCols();j++){
				double val = J.getElementAt(i,j);			
				rowLen += val * val;
			}
			rowLen = sqrt(rowLen);
			for (int j=0;j<J.getNumCols();j++){
				double val = J.getElementAt(i,j);			
				if (val != 0)
					J.setElementAt(i, j, val / rowLen);
			}
		}
*/
		svd.computeSVD(J,U,S,V);

		//for(int i=0;i<std::min(S.getNumCols(),S.getNumRows());i++)
		//	Logger::printStatic("%d th singular value of JC : %e \n",i+1,S.getElementAt(i,i));

		//based on visual inspection, when the cond number goes to about 0.008, things get very close to co-linear, so close to bifurcations/flipping. When the
		//condition number (well, the inverse) is about 0.012 or greater, everything seems to be ok... so heavily penalize everything that is below 0.012

		int n = std::min(S.getNumCols(),S.getNumRows());
		double conditionNumber = S.getElementAt(n-1,n-1);// / S.getElementAt(0,0);
		double origCondNumber = conditionNumber;

		double valGood = 7e-2;//0.015;//7e-2
		double valOk = 5e-2;//0.012;//5e-2

		if (conditionNumber >= valOk)
			conditionNumber = 1-mapTo01Range(conditionNumber, valOk, valGood);
		else{
			conditionNumber = linearlyInterpolate(2, 1, valOk * 0.9, valOk, conditionNumber);
			conditionNumber *= conditionNumber;
		}
			
		double condPenalty = pow(conditionNumber,2);
		val += condNumberWeight * condPenalty + energyWeight * assemblyEnergy;

/*

		//compute sum of smallest singular values (smallest quarter)
		int numQuarter = n/4;
		double sumSmallSS = 0;
		double EPS_SMALL = 1e-3;
		int nSmallSS = 0;
		for(int i=0; i<n; i++)
			if(S.getElementAt(i,i)<EPS_SMALL)
				nSmallSS++;

		for(int i=0; i<numQuarter;i++)
			sumSmallSS += S.getElementAt(n-i-1,n-i-1);
		//Logger::printStatic("1/condition number: %lf \n",conditionNumber);
		//Logger::printStatic("Sum 1/4 smalles singular values: %lf \n",sumSmallSS);
		//Logger::printStatic("Num zero singular values (<%lf): %d \n",EPS_SMALL,nSmallSS);
		const double EPS_COND_NUM = 1e-8;
		//double condPenalty = -log(conditionNumber+EPS_COND_NUM);
		double alpha = -1;
		double condPenalty = pow(conditionNumber+EPS_COND_NUM,alpha);
		//double condPenalty = 1.0/(conditionNumber+EPS_COND_NUM);
		//Logger::printStatic("Cond number penalty: %lf \n",condPenalty);
		val += condPenalty + energyWeight*assemblyEnergy;
//		Logger::printStatic("1/conditionNumber: %e\tCond number penalty: %e\tenergy:%e\n",conditionNumber,condPenalty,assemblyEnergy);
*/
	}

	//tidy up a little...
	parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(startState);
	parameterizedAssembly->getCurrentMechanicalAssembly()->setTickerValue(startTickerValue);

	//Logger::printStatic("Cond number penalty: %lf \n",val);
	return val;
}	

