#ifndef MOTION_PLAN_ANALYSIS_H
#define MOTION_PLAN_ANALYSIS_H

#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

#include <nanogui/common.h>

class MotionPlanAnalysis
{
public:
	MotionPlanAnalysis(nanogui::Screen *screen);

	~MotionPlanAnalysis();

	void updateMotionPlan(const LocomotionEngineMotionPlan *motionPlan);

	const LocomotionEngineMotionPlan *motionPlan;
	nanogui::Window *window;
};

#endif // MOTION_PLAN_ANALYSIS_H
