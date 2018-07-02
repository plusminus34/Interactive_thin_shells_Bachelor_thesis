#pragma once
#include "KS_MechanicalAssembly.h"
#include "KS_MechanicalAssemblySimulator.h"
#include <iostream>
#include <string>

using namespace std;

class KS_ParameterizedMechanicalAssembly{
	friend class APP_KSRobotDesigner;
	friend class APP_KSEditor;
	friend class APP_KSMotionCurveOptimizer;
private:
	DynamicArray<string> parameterNames;
	DynamicArray<double> parameterValueMin;
	DynamicArray<double> parameterValueMax;
	DynamicArray<double> parameterRegularizers;
	DynamicArray<double> parameterValues;

	DynamicArray<string> constantNames;
	DynamicArray<double> constantValues;

	struct PARAM_REFERENCE{
		//index in the appropriate array...
		int index;
		DynamicArray<double> *paramArray;

		PARAM_REFERENCE(DynamicArray<double>* p, int i){
			index = i;
			paramArray = p;
		}

		double getValue(){
			return paramArray->at(index);
		}

		void setValue(double val){
			paramArray->at(index) = val;
		}

		void addValue(double val){
			paramArray->at(index) += val;
		}
	};
	DynamicArray<PARAM_REFERENCE> paramReferences;


	string fileContents;
	//this is the instantiated assembly. It gets re-generated whenever the parameters get set again.
	KS_MechanicalAssembly* theAssembly;
	//because the mechanism can change completely when the parameters are reset, and the simulator thus needs to be re-initialized, we'll keep a copy of it here...
	KS_MechanicalAssemblySimulator* theSimulator;

	double simulatorSolveAccuracy;

	//to make sure the simulator gets intialized properly whenever the assembly may get updated, keep a copy of it here...
	bool createAssemblyFromCurrentParameterSet();
	void replaceSymbolsInString(const DynamicArray<string>& names, const DynamicArray<double>& values, const string& inputString, string& outputString);
	string tmpString, tmpString1;


	//variables to compute gradients...
	SparseLinearSolverCholMod linSolver;
	SparseMatrix dCdst_dCds;

	DynamicArray<SparseMatrix> dCds;

public:

	KS_ParameterizedMechanicalAssembly(void);
	~KS_ParameterizedMechanicalAssembly(void);

	bool loadParameterizedAssemblyFromFile(const char* fName);
	void loadParameterSet(const char* fName);
	void saveParameterSet(const char* fName);

	void loadState(const char* fName);

	const string& getParameterName(int index);
	int getParameterCount();

	//keeps the state, changes the parameterization
	void setParameters(const DynamicArray<double>& p);

	DynamicArray<double>* getParameterVector(){
		return &parameterValues;
	}

	DynamicArray<double>* getRegularizerWeights(){
		return &parameterRegularizers;
	}

	//returns a pointer to the current mechanical assembly
	KS_MechanicalAssembly* getCurrentMechanicalAssembly();

	//returns a pointer to the current mechanical simulator
	KS_MechanicalAssemblySimulator* getCurrentMechanicalSimulator();

	/*
		computes the jacobian that tells us how a change in the assembly's parameters affects the state of the assembly. Stored in column-major order.
	*/
	void compute_dsdp(dVector* assemblyState, double correspondingTickerValue, dVector* pValues, dVector* result);

	/*
		computes the jacobian that tells us how a change in the parameters affects the state of the assembly. Stored in column-major order.
		This operation is performed for each assembly state that is passed in as parameters...
	*/
	void compute_dsdp(DynamicArray<dVector>* assemblyStates, DynamicArray<double>* correspondingTickerValues, dVector* pValues, DynamicArray<dVector>* results);

	void test_dsdp(dVector* assemblyState, double correspondingTickerValue, dVector* pValues);
	void test_dsdp(dVector* assemblyState, double correspondingTickerValue, dVector* pValues, dVector* dsdp);
	void test_dsdp(DynamicArray<dVector>* assemblyStates, DynamicArray<double>* correspondingTickerValues, dVector* pValues, DynamicArray<dVector>* dsdp);

};

