#include <RobotDesignerLib/MotionPlanAnalysis.h>

#include <nanogui/nanogui.h>

#include <GUILib/ColorMaps.h>

MotionPlanAnalysis::MotionPlanAnalysis(nanogui::Screen *screen){

	using namespace nanogui;
	using namespace Eigen;

	window = new Window(screen, "Motion Plan Analysis");
	window->setLayout(new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 15, 5));
	window->setVisible(false);
	window->setPosition({1200, 0});

	// create a plot
	plot = new Plot(window, "Wheel Speed");
	plot->setSize(Vector2i(400, 200));
	plot->setBackgroundColor(nanogui::Color(Eigen::Vector3f(0.5, 0.5, 0.5), 0.1f));
	plot->setNumTicks(Vector2i(10, 5));

	// create some buttons
	Button *b = new Button(window, "Show Legend");
	b->setFlags(Button::ToggleButton);
	b->setChangeCallback([this](bool state){
		plot->setShowLegend(state);
	});
}

MotionPlanAnalysis::~MotionPlanAnalysis()
{
	delete window;
}

void MotionPlanAnalysis::updateFromMotionPlan(const LocomotionEngineMotionPlan *motionPlan)
{
	using namespace nanogui;
	using namespace Eigen;

	Eigen::Vector2f dataMin = Vector2f(HUGE_VAL, HUGE_VAL);
	Eigen::Vector2f dataMax = Vector2f(-HUGE_VAL, -HUGE_VAL);

	int nEEs = motionPlan->endEffectorTrajectories.size();
	int index = 0;
	for (const LocomotionEngine_EndEffectorTrajectory &eeTraj : motionPlan->endEffectorTrajectories) {

		if(eeTraj.isWheel)
		{
			int nTimeSteps = eeTraj.wheelSpeed.size();
			VectorXf x(nTimeSteps);
			VectorXf y(nTimeSteps);
			for (int i = 0; i < nTimeSteps; ++i){
				x[i] = (float)i/((float)nTimeSteps-1);
				y[i] = eeTraj.wheelSpeed[i];
			}

			PlotData data(x, y, nanogui::Color(ColorMaps::getColorAt(ColorMaps::viridis, (float)index/(float)nEEs), 1.f));

			// set data
			plot->setPlotData("ee " + std::to_string(index), data);

			// get min/max
			for (int i = 0; i < 2; ++i) {
				dataMin[i] = std::min(dataMin[i], data.mMinVal[i]);
				dataMax[i] = std::max(dataMax[i], data.mMaxVal[i]);
			}

			index++;
		}
	}

	// set min/max of plot
	// Note: we don't use `plot->updateMinMax()`, because "time" will mess it up
	float range = (dataMax[1] - dataMin[1]);
	dataMin[1] -= 0.1*range;
	dataMax[1] += 0.1*range;
	plot->setDataMin(dataMin);
	plot->setDataMax(dataMax);
}

void MotionPlanAnalysis::setTimeAt(float t)
{
	using namespace Eigen;

	VectorXf x(2);
	x << t,t;
	VectorXf y(2);
	y << plot->dataMin()[1], plot->dataMax()[1];
	PlotData data(x, y, nanogui::Color({1, 0, 0}, 1.f));
	plot->setPlotData("time", data);
}
