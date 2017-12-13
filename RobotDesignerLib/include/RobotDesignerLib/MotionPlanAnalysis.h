#ifndef MOTION_PLAN_ANALYSIS_H
#define MOTION_PLAN_ANALYSIS_H

#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <GUILib/Plot.h>

class MotionPlanAnalysis
{
public:
	MotionPlanAnalysis(nanogui::Screen *screen);

	~MotionPlanAnalysis();

	void updateFromMotionPlan(const LocomotionEngineMotionPlan *motionPlan);

	void setTimeAt(float t);

//private:
	nanogui::Window *window;
	Plot *plot;
};

#endif // MOTION_PLAN_ANALYSIS_H
