#ifndef MOTION_PLAN_ANALYSIS_H
#define MOTION_PLAN_ANALYSIS_H

#include <GUILib/GLIncludes.h>
#include <GUILib/Plot.h>

#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

struct PlotWidget {
	PlotWidget(){}
	PlotWidget(Plot *plot, nanogui::Widget *widget){
		this->plot = plot;
		this->widget = widget;
	}

	Plot *plot;
	nanogui::Widget *widget;
};

class MotionPlanAnalysis
{
public:
	MotionPlanAnalysis(nanogui::Screen *screen);

	~MotionPlanAnalysis();

	void updateFromMotionPlan(const LocomotionEngineMotionPlan *motionPlan);

	void setTimeAt(float t);

private:
	PlotWidget makePlotWidget(nanogui::Widget *window, const std::string &name);

	void updatePlotScaling();

public: //private:
	nanogui::Window *window;

	enum PlotNames {
		WHEEL_SPEED, WHEEL_TILT_ANGLE, WHEEL_YAW_ANGLE, EE_POS_Y,
		JOINT_ANGLES, COM_POSITION, COM_ORIENTATION,
		GRF_FORCES
	};
	std::map<PlotNames, PlotWidget> plots;
};

#endif // MOTION_PLAN_ANALYSIS_H
