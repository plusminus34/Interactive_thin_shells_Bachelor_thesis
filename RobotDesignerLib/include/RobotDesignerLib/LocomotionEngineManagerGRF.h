#pragma once

#include <RobotDesignerLib/LocomotionEngineManager.h>

class LocomotionEngineManagerGRF : public LocomotionEngineManager{
public:
	LocomotionEngineManagerGRF();
	LocomotionEngineManagerGRF(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints);

	~LocomotionEngineManagerGRF();

	void warmStartMOpt();
};

class LocomotionEngineManagerGRFv2 : public LocomotionEngineManager {
public:
	LocomotionEngineManagerGRFv2();
	LocomotionEngineManagerGRFv2(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints);

	~LocomotionEngineManagerGRFv2();

	void warmStartMOpt();
};

class LocomotionEngineManagerGRFv3 : public LocomotionEngineManager {
public:
	LocomotionEngineManagerGRFv3();
	LocomotionEngineManagerGRFv3(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints);

	~LocomotionEngineManagerGRFv3();

	void warmStartMOpt();
};