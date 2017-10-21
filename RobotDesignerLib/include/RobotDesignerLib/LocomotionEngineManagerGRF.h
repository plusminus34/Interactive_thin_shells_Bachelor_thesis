#pragma once

#include <RobotDesignerLib/LocomotionEngineManager.h>

class LocomotionEngineManagerGRF : public LocomotionEngineManager {
public:
	LocomotionEngineManagerGRF();
	LocomotionEngineManagerGRF(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints);

	~LocomotionEngineManagerGRF();

	void warmStartMOpt();
};

/*************************************************************************************************************/
/* GRFv1 employs an SQP solver operating on a mix of objectives and constraints to generate the motion plan*/
/*************************************************************************************************************/
class LocomotionEngineManagerGRFv1 : public LocomotionEngineManagerGRF {
public:
	LocomotionEngineManagerGRFv1();
	LocomotionEngineManagerGRFv1(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints);

	~LocomotionEngineManagerGRFv1();
	void setupObjectives();
};

/*************************************************************************************************************/
/* For GRFv2, constraints are transformed into objectives with high weights, so this one employs a Newton-style solver   */
/*************************************************************************************************************/
class LocomotionEngineManagerGRFv2 : public LocomotionEngineManagerGRF {
public:
	LocomotionEngineManagerGRFv2();
	LocomotionEngineManagerGRFv2(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints);

	~LocomotionEngineManagerGRFv2();
	void setupObjectives();
};
