#pragma once
#include <RobotDesignerLib/ParameterizedRobotDesign.h>

class MorphologicalRobotDesign : public ParameterizedRobotDesign
{
public:
	MorphologicalRobotDesign(Robot* _robot);
	~MorphologicalRobotDesign();

	void getCurrentSetOfParameters(DynamicArray<double>& params);
	void setParameters(const DynamicArray<double>& params);

	void loadParamsFromFile(const char* fName);
};

