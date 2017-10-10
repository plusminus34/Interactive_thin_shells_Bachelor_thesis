#pragma once

#include <RobotDesignerLib/LocomotionEngineManager.h>

class LocomotionEngineManagerIP : public LocomotionEngineManager
{

public:
	LocomotionEngineManagerIP();
	LocomotionEngineManagerIP(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints);

	~LocomotionEngineManagerIP();

	void warmStartMOpt();
};

class LocomotionEngineManagerIPv2 : public LocomotionEngineManager{

public:
	LocomotionEngineManagerIPv2();
	LocomotionEngineManagerIPv2(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints);

	~LocomotionEngineManagerIPv2();

	void warmStartMOpt();
};