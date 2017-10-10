#pragma once

#include <MathLib/Trajectory.h>

#define INITIALIZE_PARAMETER(pList, x, val, min, max, name)																			\
	{ x = (int)pList.size(); pList.push_back(ControlParameter(val, min, max, fabs((double)((max) - (min)))/10.0, false, name)); }

#define INITIALIZE_PARAMETER_FULL(pList, x, val, min, max, maxAbsValueChange, frozen, name)											\
	{ x = (int)pList.size(); pList.push_back(ControlParameter(val, min, max, maxAbsValueChange, frozen, name)); }

class ControlParameter{
public:
	//keep track of the min and max values that this parameter can take
	double min, max;
	//keep track of the value
	double value;
	//see if this variable should change value or not, in case we were running some optimization
	bool frozen;
	//this is a parameter that indicates how much this value can be changing each iteration
	double maxAbsValueChange;
	//and a name - used for debugging mostly
	char* pName;

	ControlParameter(double value, double min, double max, double maxAbsValueChange, bool frozen, char* name){
		this->min = min;
		this->max = max;
		this->value = value;
		this->maxAbsValueChange = maxAbsValueChange;
		this->frozen = frozen;
		this->pName = name;
	}

	ControlParameter(const ControlParameter& other){
		this->min = other.min;
		this->max = other.max;
		this->value = other.value;
		this->maxAbsValueChange = other.maxAbsValueChange;
		this->frozen = other.frozen;
		this->pName = other.pName;
	}

	ControlParameter& operator = (const ControlParameter& other){
		this->min = other.min;
		this->max = other.max;
		this->value = other.value;
		this->maxAbsValueChange = other.maxAbsValueChange;
		this->frozen = other.frozen;
		this->pName = other.pName;

		return *this;
	}

	inline double getScaledValue(int scaleFactor){
		return scaleFactor * (this->value - this->min)/(this->max - this->min);
	}

};

inline void setParameterValues(DynamicArray<double>* values, DynamicArray<ControlParameter>* parameters){
	if (values->size() != parameters->size())
		throwError("Number of parameters do not match.");
	for (uint i=0;i<values->size();i++)
		parameters->at(i).value = values->at(i);
}

inline void filterParameterValues(double* values, DynamicArray<ControlParameter>* parameters){
	for (uint i=0;i<parameters->size();i++){
		boundToRange(&values[i], parameters->at(i).min, parameters->at(i).max);
	}
}

inline void setParameterValues(double const* values, DynamicArray<ControlParameter>* parameters){
	double* copiedValues = new double[parameters->size()];
	for (uint i=0;i<parameters->size();i++){
		copiedValues[i] = values[i];
	}
	filterParameterValues(copiedValues, parameters);

	for (uint i=0;i<parameters->size();i++){
		parameters->at(i).value = copiedValues[i];
		//boundToRange(&parameters->at(i).value, parameters->at(i).min, parameters->at(i).max);
	}
	delete [] copiedValues;
}

inline void scaleAndSetParameterValues(double const* values, DynamicArray<ControlParameter>* parameters, int scaleFactor){
	double* copiedValues = new double[parameters->size()];
	for (uint i=0;i<parameters->size();i++){
		copiedValues[i] = values[i]*(parameters->at(i).max-parameters->at(i).min)/scaleFactor + parameters->at(i).min;
	}
	setParameterValues(copiedValues, parameters);
	delete [] copiedValues;
}

inline void writeParameterValuesToFile(char* fName, const DynamicArray<ControlParameter>& parameters){
	FILE* fp = fopen(fName, "w");
	if (fp == NULL)
		throwError("Cannot open file %s", fName);

	for (uint i=0; i< parameters.size();i++){
		if (i > 0 && strcmp(parameters[i].pName, parameters[i-1].pName) == 0)
			fprintf(fp, "%lf\n", parameters[i].value);
		else
			fprintf(fp, "\n# %s\n%lf\n", parameters[i].pName, parameters[i].value);
	}

	fclose(fp);
}

inline void readParameterValuesFromFile(char* fName, DynamicArray<ControlParameter>* parameters){
	DynamicArray<double> values;
	FILE* fp = fopen(fName, "r");
	if (fp == NULL)
		throwError("Cannot open file %s", fName);
	readDoublesFromFile(fp, &values);
	fclose(fp);
	setParameterValues(&values, parameters);
}

inline void getScaledParameterValues(DynamicArray<double>* values, DynamicArray<ControlParameter>* parameters, int scaleFactor){
	values->clear();
	for (uint i=0; i< parameters->size();i++)
		values->push_back(parameters->at(i).getScaledValue(scaleFactor));
}

inline void getParameterValues(DynamicArray<double>* values, DynamicArray<ControlParameter>* parameters){
	values->clear();
	for (uint i=0; i< parameters->size();i++)
		values->push_back(parameters->at(i).value);
}

inline void setDiscontinuousTrajectory(Trajectory1D* traj, const DynamicArray<ControlParameter>& parameterList, int *indices, int count, double firstKnotPosition = 0.0, double lastKnotPosition = 1.0){
	traj->clear();
	for (int i=0;i<count;i++)
		traj->addKnot(firstKnotPosition + ((double)i / (count-1)) * (lastKnotPosition - firstKnotPosition), parameterList[indices[i]].value);
}

inline void setContinuousTrajectory(Trajectory1D* traj, const DynamicArray<ControlParameter>& parameterList, int *indices, int count){
	traj->clear();
	for (int i=0;i<count;i++)
		traj->addKnot((double)i / count, parameterList[indices[i]].value);
	//and now make sure the last knot value is the same as the first
	traj->addKnot(1, parameterList[indices[0]].value);
}
