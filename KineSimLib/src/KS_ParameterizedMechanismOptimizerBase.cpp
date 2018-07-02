#include "KineSimLib/KS_ParameterizedMechanismOptimizerBase.h"

#define SUPER_LARGE_VAL 100000000

KS_ParameterizedMechanismOptimizerBase::KS_ParameterizedMechanismOptimizerBase(KS_ParameterizedMechanicalAssembly* mechanism, int numberOfEvaluationPoints){
	this->parameterizedAssembly = mechanism;
	this->startTickerValue = mechanism->getCurrentMechanicalAssembly()->getTickerValue();
	mechanism->getCurrentMechanicalAssembly()->getAssemblyState(this->startState);
	this->numberOfEvaluationPoints = numberOfEvaluationPoints;
	updateStartingState();
}

KS_ParameterizedMechanismOptimizerBase::~KS_ParameterizedMechanismOptimizerBase(){
}

bool KS_ParameterizedMechanismOptimizerBase::computeAssemblyMotionGivenParameters(const dVector &p, DynamicArray<dVector> *stateArray, DynamicArray<double> *tickerValueArray){

	//make sure we don't do any extra work if we've already computed the motion for these parameters
	if (p.isSameAs(cached_parameterSet) && cached_assemblyStateArray.size() == numberOfEvaluationPoints+1){
		*stateArray = cached_assemblyStateArray;
		*tickerValueArray = cached_correspondingTickerValues;
		loadStartingState();
		return true;
	}

	parameterizedAssembly->setParameters(p);
	loadStartingState();
	
	stateArray->clear();
	tickerValueArray->clear();

	//collect the states that we sample the objective function at
	for (int i=0; i<numberOfEvaluationPoints+1; i++){
		if (parameterizedAssembly->getCurrentMechanicalSimulator()->solve(0.0001) > 1e-8){
			loadStartingState();
			return false;
		}
		parameterizedAssembly->getCurrentMechanicalAssembly()->updateTracerParticles();
		stateArray->push_back(dVector());
		parameterizedAssembly->getCurrentMechanicalAssembly()->getAssemblyState(stateArray->at(i));
		tickerValueArray->push_back(parameterizedAssembly->getCurrentMechanicalAssembly()->getTickerValue());
		parameterizedAssembly->getCurrentMechanicalAssembly()->stepAssembly(true);
	}

	loadStartingState();

	cached_assemblyStateArray = *stateArray;
	cached_correspondingTickerValues = *tickerValueArray;
	cached_parameterSet = p;

	return true;
}

bool KS_ParameterizedMechanismOptimizerBase::computeAssemblyMotionGivenParameters(const dVector &p){
	DynamicArray<dVector> stateArray;
	DynamicArray<double> tickerValueArray;

	return computeAssemblyMotionGivenParameters(p, &stateArray, &tickerValueArray);
}

void KS_ParameterizedMechanismOptimizerBase::loadStartingState(){
	parameterizedAssembly->getCurrentMechanicalAssembly()->setAssemblyState(startState);
	parameterizedAssembly->getCurrentMechanicalAssembly()->setTickerValue(startTickerValue);
}


void KS_ParameterizedMechanismOptimizerBase::updateStartingState(){
	loadStartingState();
	if (parameterizedAssembly->getCurrentMechanicalSimulator()->solve(0.0001) < 1e-7)
		parameterizedAssembly->getCurrentMechanicalAssembly()->getAssemblyState(startState);
}




