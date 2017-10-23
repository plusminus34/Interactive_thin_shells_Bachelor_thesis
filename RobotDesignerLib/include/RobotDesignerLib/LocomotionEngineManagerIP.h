#pragma once

#include <RobotDesignerLib/LocomotionEngineManager.h>

class LocomotionEngineManagerIP : public LocomotionEngineManager{

public:
	LocomotionEngineManagerIP();
	LocomotionEngineManagerIP(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints);

	~LocomotionEngineManagerIP();
};

/*****************************************************************************************/
/* IPv1 is the original mopt described in robot designer.                                */
/******************************************************************************************/
class LocomotionEngineManagerIPv1 : public LocomotionEngineManagerIP {

public:
	LocomotionEngineManagerIPv1();
	LocomotionEngineManagerIPv1(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints);
	~LocomotionEngineManagerIPv1();

	void setupObjectives();
	void warmStartMOpt();
};

/***************************************************************************************/
/* IPv2 uses separate DOFs for rotations.                                              */
/***************************************************************************************/
class LocomotionEngineManagerIPv2 : public LocomotionEngineManagerIP {

public:
	LocomotionEngineManagerIPv2();
	LocomotionEngineManagerIPv2(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints);

	~LocomotionEngineManagerIPv2();

	void setupObjectives();
	void warmStartMOpt();
};

