#include "KineSimLib/KS_ParameterizedMechanicalAssembly.h"
#include "KineSimLib/KS_LoaderUtils.h"
#include <sstream>
#include <fstream>
#include <errno.h>

KS_ParameterizedMechanicalAssembly::KS_ParameterizedMechanicalAssembly(void){
	theAssembly = NULL;
	theSimulator = NULL;
	simulatorSolveAccuracy = 1e-7;
}

KS_ParameterizedMechanicalAssembly::~KS_ParameterizedMechanicalAssembly(void){
	delete theAssembly;
	delete theSimulator;
}

int KS_ParameterizedMechanicalAssembly::getParameterCount(){
	return (int)parameterNames.size();
}

const string& KS_ParameterizedMechanicalAssembly::getParameterName(int index){
	return parameterNames[index];
}

void KS_ParameterizedMechanicalAssembly::saveParameterSet(const char* fName){
	//load the values of the parameters, as well as the initial state of the assembly and the corresponding ticker value...
	FILE* fp = fopen(fName, "w");
	for (uint i=0; i<paramReferences.size();i++)
		fprintf(fp, "%lf ", paramReferences[i].getValue());
	fprintf(fp, "\n");

	fprintf(fp, "%lf\n", getCurrentMechanicalAssembly()->getTickerValue());
	dVector state;
	getCurrentMechanicalAssembly()->getAssemblyState(state);
	for (uint i=0;i<state.size();i++)
		fprintf(fp, "%lf ", state[i]);
	fprintf(fp, "\n");
	fclose(fp);
}

void KS_ParameterizedMechanicalAssembly::loadState(const char* fName){
	//load the values of the parameters, as well as the initial state of the assembly and the corresponding ticker value...
	FILE* fp = fopen(fName, "r");
	double tickerValue;
	fscanf(fp, "%lf", &tickerValue);
	dVector state; getCurrentMechanicalAssembly()->getAssemblyState(state);
	for (uint i=0;i<state.size();i++)
		fscanf(fp, "%lf", &state[i]);
	fclose(fp);
	getCurrentMechanicalAssembly()->setTickerValue(tickerValue);
	getCurrentMechanicalAssembly()->setAssemblyState(state);
}


void KS_ParameterizedMechanicalAssembly::loadParameterSet(const char* fName){
	//load the values of the parameters, as well as the initial state of the assembly and the corresponding ticker value...
	FILE* fp = fopen(fName, "r");
	for (uint i=0; i<paramReferences.size();i++){
		double val = 0;
		fscanf(fp, "%lf", &val);
		paramReferences[i].setValue(val);
	}
	setParameters(parameterValues);
	double tickerValue;
	fscanf(fp, "%lf", &tickerValue);
	dVector state; getCurrentMechanicalAssembly()->getAssemblyState(state);
	for (uint i=0;i<state.size();i++)
		fscanf(fp, "%lf", &state[i]);
	fclose(fp);
	getCurrentMechanicalAssembly()->setAssemblyState(state);
	getCurrentMechanicalAssembly()->setTickerValue(tickerValue);
}


bool KS_ParameterizedMechanicalAssembly::loadParameterizedAssemblyFromFile(const char* fName){
	fileContents.clear();
	parameterNames.clear();
	parameterValueMax.clear();
	parameterValueMin.clear();
	parameterValues.clear();
	parameterRegularizers.clear();
	paramReferences.clear();
	constantNames.clear();
	constantValues.clear();

	delete theAssembly;
	theAssembly = NULL;

	FILE* fp = fopen(fName, "r");

	if (fp == NULL){
		assert(false);
		return false;
	}

	//have a temporary buffer used to read the file line by line...
	char buffer[1000];
	//this is where it happens.
	while (!feof(fp)){
		//get a line from the file...
		readValidLine(buffer, fp, sizeof(buffer));

		char *line = lTrim(buffer);
		int lineType = getKSLineType(line);

		if (lineType == KS_PARAMETER){
			char name[100];
			double val = 0, min = -1000, max = 1000, regularizer = 0;
			sscanf(trim(line), "%s %lf %lf %lf %lf", name, &val, &min, &max, &regularizer);
			parameterNames.push_back(string(name));
			parameterValues.push_back(val);
			parameterValueMin.push_back(min);
			parameterValueMax.push_back(max);
			parameterRegularizers.push_back(regularizer);
			paramReferences.push_back(PARAM_REFERENCE(&parameterValues, (int)parameterValues.size() - 1));
		}else if (lineType == KS_CONSTANT){
			char name[100];
			double val = 0;
			sscanf(trim(line), "%s %lf", name, &val);
			constantNames.push_back(string(name));
			constantValues.push_back(val);
			paramReferences.push_back(PARAM_REFERENCE(&constantValues, (int)constantValues.size() - 1));
		}else{
			fileContents.append(buffer);
			fileContents.append("\n");
		}
	}

	fclose(fp);

	return createAssemblyFromCurrentParameterSet();
}

void KS_ParameterizedMechanicalAssembly::replaceSymbolsInString(const DynamicArray<string>& names, const DynamicArray<double>& values, const string& inputString, string& outputString){
	outputString.clear();
	outputString += inputString;
	DynamicArray<string> namesWithMinus;
	for (uint i = 0; i < names.size(); i++) {
		stringstream ss;
		ss << "-" << names[i];
		namesWithMinus.push_back(ss.str());
	}
	//replace all with minus
	for (uint i=0;i<namesWithMinus.size(); i++){
		int pIndex = -1;
		while ((pIndex = (int)outputString.find(namesWithMinus[i])) != string::npos){
			stringstream ss;
			ss << (-values[i]);
			outputString.replace(pIndex, namesWithMinus[i].size(), ss.str());
		}
	}
	//replace all the instances of the parameter names with their current values
	for (uint i=0;i<names.size(); i++){
		int pIndex = -1;
		while ((pIndex = (int)outputString.find(names[i])) != string::npos){
			stringstream ss;
			ss << values[i];
			outputString.replace(pIndex, names[i].size(), ss.str());
		}
	}
}

bool KS_ParameterizedMechanicalAssembly::createAssemblyFromCurrentParameterSet(){
	dVector currentState;
	double currentTickerValue;
	bool hasAState = false;

	if (theAssembly){
		hasAState = true;
		theAssembly->getAssemblyState(currentState);
		currentTickerValue = theAssembly->getTickerValue();
	}

	delete theAssembly;
	delete theSimulator;

	replaceSymbolsInString(constantNames, constantValues, fileContents, tmpString1);

	//create a tmp file now that has an instance of the current parameter values
	replaceSymbolsInString(parameterNames, parameterValues, tmpString1, tmpString);
	//now write this to a file and load the assembly...
	char filename[2560];
	// int irand = rand()%10000;
	sprintf(filename, "out/tmpParameterizedAssembly.cdma");

	FILE* fp = fopen(filename, "w");
	if(!fp)	{
		int err = 0;
		_get_errno(&err);
		char errMsg[1024];
		sprintf(errMsg,"%s", strerror(err));
		Logger::printStatic("fopen failed with msg %s\n",errMsg);
		return false;
	}

	fprintf(fp, "%s", tmpString.c_str());
	fclose(fp);

	theAssembly = new KS_MechanicalAssembly();
	if (theAssembly->readFromFile(filename) == false)
		return false;

	if (hasAState){
		theAssembly->setAssemblyState(currentState);
		theAssembly->setTickerValue(currentTickerValue);
	}


	theSimulator = new KS_MechanicalAssemblySimulator();
	theSimulator->initialize(theAssembly, simulatorSolveAccuracy);
	//unlink(filename);
	return true;
}

//creates a new mechanical assembly with the current parameter values set
KS_MechanicalAssembly* KS_ParameterizedMechanicalAssembly::getCurrentMechanicalAssembly(){
	if (theAssembly == NULL)
		createAssemblyFromCurrentParameterSet();
	return theAssembly;
}

//returns a pointer to the current mechanical simulator
KS_MechanicalAssemblySimulator* KS_ParameterizedMechanicalAssembly::getCurrentMechanicalSimulator(){
	return theSimulator;
}

//changes the parameter values
void KS_ParameterizedMechanicalAssembly::setParameters(const DynamicArray<double>& p){
	assert(p.size() == parameterValues.size());
	parameterValues = p;

	for (uint i=0;i<parameterValues.size();i++)
		boundToRange(&parameterValues[i], parameterValueMin[i], parameterValueMax[i]);

	//now set the parameters of the assembly, but without touching the state... for now that means we'll just create the assembly from scratch...
	createAssemblyFromCurrentParameterSet();
}

/*
	computes the jacobian that tells us how a change in the parameters affects the state of the assembly. Stored in column-major order. First |s| elements tell us
	how the state changes with p1, second set of |s| elements tells us how the full state changes with p2, and so on and so forth.
*/
void KS_ParameterizedMechanicalAssembly::compute_dsdp(dVector* assemblyState, double correspondingTickerValue, dVector* pValues, dVector* result){
	setParameters(*pValues);
	KS_MechanicalAssembly*			assembly = getCurrentMechanicalAssembly(); 	assembly->setTickerValue(correspondingTickerValue);
	KS_AssemblyConstraintEnergy*		energyFunction = new KS_AssemblyConstraintEnergy(); energyFunction->initialize(assembly);

	int n = (int)assemblyState->size();							//state size
	int m = (int)energyFunction->getScalarConstraintCount();	//number of constraints
	int p = (int)pValues->size();								//number of parameters

	//ds/dp = - dC/ds^-1 * dC/dp, but we will use the pseudo inverse because we may have more constraints than state variables (although they are redundant)
	if (dCds.size() == 0) dCds.resize(1);
	dCds[0].resize(m, n, false);
	dCds[0].zero();
	dCds[0].add(*energyFunction->getConstraintJacobianAt(*assemblyState));
	dCdst_dCds.setToAtA(dCds[0]);

	double dp = 10e-5;
	dVector C_P(m,0), C_M(m,0), dCdp_i_col(m,0);
	dVector tmpVec(n, 0);
	dVector b;

	b.resize(n * p);
	result->resize(n * p);

	for (int i=0;i<p;i++){
		double tmpVal = pValues->at(i);
		pValues->at(i) = tmpVal + dp;
		setParameters(*pValues); assembly = getCurrentMechanicalAssembly(); assembly->setTickerValue(correspondingTickerValue); delete energyFunction; energyFunction = new KS_AssemblyConstraintEnergy(); energyFunction->initialize(assembly);
		copy(*energyFunction->getConstraintVectorAt(*assemblyState), C_P);

		pValues->at(i) = tmpVal - dp; 
		setParameters(*pValues); assembly = getCurrentMechanicalAssembly(); assembly->setTickerValue(correspondingTickerValue); delete energyFunction; energyFunction = new KS_AssemblyConstraintEnergy(); energyFunction->initialize(assembly);
		copy(*energyFunction->getConstraintVectorAt(*assemblyState), C_M);

		//now reset the ith param value
		pValues->at(i) = tmpVal;

		add(C_P, 1.0/(2*dp), C_M, -1.0/(2*dp), dCdp_i_col);

		//each vector is a column vector of b, and store b in column-major...
		dCds[0].multVectorTransposed(dCdp_i_col, tmpVec);
		for (int j=0;j<n;j++)
			b[i*n+j] = -tmpVec[j];
	}

	//now reset the params and the state, and make sure everything is consistent...
	setParameters(*pValues); assembly = getCurrentMechanicalAssembly(); assembly->setTickerValue(correspondingTickerValue); delete energyFunction; assembly->setAssemblyState(*assemblyState);

	//dqdqp will be stored in column-major order
	linSolver.solveLinearSystem(&dCdst_dCds, &((*result)[0]), &b[0], p);
}

/*
	computes the jacobian that tells us how a change in the parameters affects the state of the assembly. Stored in column-major order.
	This operation is performed for each assembly state that is passed in as parameters...
*/
void KS_ParameterizedMechanicalAssembly::compute_dsdp(DynamicArray<dVector>* assemblyStates, DynamicArray<double>* correspondingTickerValues, dVector* pValues, DynamicArray<dVector>* results){
	setParameters(*pValues);
	KS_MechanicalAssembly*			assembly = getCurrentMechanicalAssembly();
	KS_MechanicalAssemblySimulator*	simulator = getCurrentMechanicalSimulator();

	int nStates = (int)assemblyStates->size();
	assert(nStates == (int)correspondingTickerValues->size());
	int n = (int)assemblyStates->at(0).size();					//state size
	int m = (int)simulator->energyFunction->getScalarConstraintCount();	//number of constraints
	int p = (int)pValues->size();								//number of parameters

	//ds/dp = - dC/ds^-1 * dC/dp, but we will use the pseudo inverse because we may have more constraints than state variables (although they are redundant)
	dCds.resize(nStates);
	results->resize(nStates);
	DynamicArray<dVector> b(nStates), C_P(nStates), C_M(nStates);
	for (int i=0; i < nStates; i++){
		results->at(i).resize(n * p);
		dCds[i].resize(m, n, false);
		b[i].resize(n * p);
		C_P[i].resize(m, 0);
		C_M[i].resize(m, 0);
		dCds[i].zero();
		assembly->setTickerValue(correspondingTickerValues->at(i));
		dCds[i].add(*simulator->energyFunction->getConstraintJacobianAt(assemblyStates->at(i)));
	}

	double dp = 10e-5;
	dVector tmpVec(n, 0);
	dVector dCdp_i_col(m, 0);

	for (int i=0;i<p;i++){
		double tmpVal = pValues->at(i);
		pValues->at(i) = tmpVal + dp;
		setParameters(*pValues); assembly = getCurrentMechanicalAssembly(); simulator = getCurrentMechanicalSimulator();
		
		for (int j = 0; j<nStates;j++){
			assembly->setTickerValue(correspondingTickerValues->at(j)); 
			copy(*simulator->energyFunction->getConstraintVectorAt(assemblyStates->at(j)), C_P[j]);
		}

		pValues->at(i) = tmpVal - dp; 
		setParameters(*pValues); assembly = getCurrentMechanicalAssembly(); simulator = getCurrentMechanicalSimulator();
		
		for (int j = 0; j<nStates;j++){
			assembly->setTickerValue(correspondingTickerValues->at(j)); 
			copy(*simulator->energyFunction->getConstraintVectorAt(assemblyStates->at(j)), C_M[j]);
		}

		//now reset the ith param value
		pValues->at(i) = tmpVal;

		for (int j=0; j<nStates; j++){
			add(C_P[j], 1.0/(2*dp), C_M[j], -1.0/(2*dp), dCdp_i_col);

			//each vector is a column vector of b, and store b in column-major...
			dCds[j].multVectorTransposed(dCdp_i_col, tmpVec);
			for (int k=0;k<n;k++)
				b[j][i*n+k] = -tmpVec[k];
		}
	}

	for (int j=0; j<nStates; j++){
		dCdst_dCds.setToAtA(dCds[j]);
		//dqdqp will be stored in column-major order
		linSolver.solveLinearSystem(&dCdst_dCds, &results->at(j)[0], &b[j][0], p);
	}

	//now reset the params and the state, and make sure everything is consistent...
	setParameters(*pValues); assembly = getCurrentMechanicalAssembly(); assembly->setTickerValue(correspondingTickerValues->at(0)); assembly->setAssemblyState(assemblyStates->at(0));
}

void KS_ParameterizedMechanicalAssembly::test_dsdp(dVector* assemblyState, double correspondingTickerValue, dVector* pValues){
	//first compute the analytic jacobian
	dVector dsdp;
	compute_dsdp(assemblyState, correspondingTickerValue, pValues, &dsdp);

	test_dsdp(assemblyState, correspondingTickerValue, pValues, &dsdp);
}

void KS_ParameterizedMechanicalAssembly::test_dsdp(DynamicArray<dVector>* assemblyStates, DynamicArray<double>* correspondingTickerValues, dVector* pValues, DynamicArray<dVector>* dsdp){
	int nStates = (int)assemblyStates->size();
	assert(nStates == (int)correspondingTickerValues->size());

	for (int i=0; i<nStates; i++)
		test_dsdp(&assemblyStates->at(i), correspondingTickerValues->at(i), pValues, &dsdp->at(i));
}

void KS_ParameterizedMechanicalAssembly::test_dsdp(dVector* assemblyState, double correspondingTickerValue, dVector* pValues, dVector* dsdp){
	logPrint("Testing dsdp jacobian...\n");
	simulatorSolveAccuracy = 1e-10;

	int n = (int)assemblyState->size();	//state size
	int p = (int)pValues->size();//number of parameters

	double dp = 0.001;
	dVector s_p(n,0), s_m(n,0), dsdp_i_col(n,0);

	for (int i=0;i<p;i++){
		double tmpVal = pValues->at(i);
		pValues->at(i) = tmpVal + dp;
		setParameters(*pValues);
		getCurrentMechanicalAssembly()->setAssemblyState(*assemblyState);
		getCurrentMechanicalAssembly()->setTickerValue(correspondingTickerValue);
		if (getCurrentMechanicalSimulator()->solve(0.0001) > 0.001)	assert(false);
		getCurrentMechanicalAssembly()->getAssemblyState(s_p);

		pValues->at(i) = tmpVal - dp; 
		setParameters(*pValues);
		getCurrentMechanicalAssembly()->setAssemblyState(*assemblyState);
		getCurrentMechanicalAssembly()->setTickerValue(correspondingTickerValue);
		if (getCurrentMechanicalSimulator()->solve(0.0001) > 0.001)	assert(false);
		getCurrentMechanicalAssembly()->getAssemblyState(s_m);

		//now reset the state
		pValues->at(i) = tmpVal;

		add(s_p, 1.0/(2*dp), s_m, -1.0/(2*dp), dsdp_i_col);
		//now we have a column - the ith column, to compare with the analytic data
		for (int j=0;j<n;j++){
			double error = (dsdp->at(i*n+j) - dsdp_i_col[j]);
			if (fabs(error) > 0.0001)
				logPrint("element at (%d %d): err: %lf, val1: %lf, val2: %lf\n", i, j, error, dsdp->at(i*n+j), dsdp_i_col[j]);
		}
	}

	simulatorSolveAccuracy = 1e-7;
	setParameters(*pValues);
	getCurrentMechanicalAssembly()->setAssemblyState(*assemblyState);
	getCurrentMechanicalAssembly()->setTickerValue(correspondingTickerValue);
}







