#pragma once

#include <RobotDesignerLib/LocomotionEngineManager.h>

class LocomotionEngineManagerGRF : public LocomotionEngineManager {
protected:
	bool periodicMotion = true;

public:
	LocomotionEngineManagerGRF();
	LocomotionEngineManagerGRF(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints, bool periodicMotion = true);

	~LocomotionEngineManagerGRF();

	void warmStartMOpt();

	void setDefaultOptimizationFlags();

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

public:
	LocomotionEngineManagerGRFv2();
	LocomotionEngineManagerGRFv2(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints, bool periodicMotion = true);

	~LocomotionEngineManagerGRFv2();
	void setupObjectives();
};


/*************************************************************************************************************************/
/* For GRFv3, constraints are transformed into objectives with high weights, so this one employs a Newton-style solver   */
/* Furthermore, robot states are not explicitly optimized, at least not at the same time as everything else              */
/*************************************************************************************************************************/
class LocomotionEngineManagerGRFv3 : public LocomotionEngineManagerGRF {
public:

public:
	LocomotionEngineManagerGRFv3();
	LocomotionEngineManagerGRFv3(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints, bool periodicMotion = true);

	~LocomotionEngineManagerGRFv3();
	void setupObjectives();
	void warmStartMOpt();
	void setDefaultOptimizationFlags();
};