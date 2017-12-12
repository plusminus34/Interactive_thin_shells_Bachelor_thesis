#include <RobotDesignerLib/MotionPlanAnalysis.h>

#include <nanogui/nanogui.h>

MotionPlanAnalysis::MotionPlanAnalysis(nanogui::Screen *screen)
	: motionPlan(motionPlan){

	using namespace nanogui;
	using namespace Eigen;

	FormHelper *gui = new FormHelper(screen);
	window = gui->addWindow(Eigen::Vector2i(10, 10), "Motion Plan Analysis");

	gui->addButton("Test!", [](){});
}

MotionPlanAnalysis::~MotionPlanAnalysis()
{
	delete window;
}

void MotionPlanAnalysis::updateMotionPlan(const LocomotionEngineMotionPlan *motionPlan)
{

}
