#include <RobotDesignerLib/MotionPlanAnalysis.h>

#include <nanogui/nanogui.h>

#include <GUILib/ColorMaps.h>

using namespace nanogui;
using namespace Eigen;

MotionPlanAnalysis::MotionPlanAnalysis(nanogui::Screen *screen){

	window = new Window(screen, "Motion Plan Analysis");
	window->setLayout(new GridLayout(Orientation::Vertical, 4, Alignment::Middle, 15, 5));
	window->setVisible(false);
	window->setPosition({1200, 0});

	// create a plot
	plots[WHEEL_SPEED] = makePlotWidget(window, "Wheel Speed");
	plots[WHEEL_TILT_ANGLE] = makePlotWidget(window, "Wheel Tilt Angle");
	plots[WHEEL_YAW_ANGLE] = makePlotWidget(window, "Wheel Yaw Angle");

}

MotionPlanAnalysis::~MotionPlanAnalysis()
{
	delete window;
}

void MotionPlanAnalysis::updateFromMotionPlan(const LocomotionEngineMotionPlan *motionPlan)
{
	int nEEs = motionPlan->endEffectorTrajectories.size();
	int index = 0;
	for (const LocomotionEngine_EndEffectorTrajectory &eeTraj : motionPlan->endEffectorTrajectories) {

		// wheel speed
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
			plots[WHEEL_SPEED].plot->setPlotData("ee " + std::to_string(index), data);
		}

		// wheel tilt angle
		{
			int nTimeSteps = eeTraj.wheelTiltAngle.size();
			VectorXf x(nTimeSteps);
			VectorXf y(nTimeSteps);
			for (int i = 0; i < nTimeSteps; ++i){
				x[i] = (float)i/((float)nTimeSteps-1);
				y[i] = eeTraj.wheelTiltAngle[i];
			}

			PlotData data(x, y, nanogui::Color(ColorMaps::getColorAt(ColorMaps::viridis, (float)index/(float)nEEs), 1.f));
			plots[WHEEL_TILT_ANGLE].plot->setPlotData("ee " + std::to_string(index), data);
		}

		// wheel yaw angle
		{
			int nTimeSteps = eeTraj.wheelYawAngle.size();
			VectorXf x(nTimeSteps);
			VectorXf y(nTimeSteps);
			for (int i = 0; i < nTimeSteps; ++i){
				x[i] = (float)i/((float)nTimeSteps-1);
				y[i] = eeTraj.wheelYawAngle[i];
			}

			PlotData data(x, y, nanogui::Color(ColorMaps::getColorAt(ColorMaps::magma, (float)index/(float)nEEs), 1.f));
			plots[WHEEL_YAW_ANGLE].plot->setPlotData("ee " + std::to_string(index), data);
		}

		index++;
	}

	updatePlotScaling();
}

void MotionPlanAnalysis::setTimeAt(float t)
{
	VectorXf x(2);
	x << t,t;
	VectorXf y(2);

	for (auto &pp : plots) {
		PlotWidget &p = pp.second;
		y << p.plot->dataMin()[1], p.plot->dataMax()[1];
		PlotData data(x, y, nanogui::Color({1, 0, 0}, 1.f));
		p.plot->setPlotData("time", data);
	}
}

PlotWidget MotionPlanAnalysis::makePlotWidget(nanogui::Window *window, const std::string &name)
{
	Widget *widget = new Widget(window);
	widget->setLayout(new GridLayout(Orientation::Horizontal, 2, Alignment::Fill, 5, 5));

	Plot *plot = new Plot(widget, name);
	plot->setSize(Vector2i(400, 200));
	plot->setBackgroundColor(nanogui::Color(Eigen::Vector3f(0.5, 0.5, 0.5), 0.1f));
	plot->setNumTicks(Vector2i(10, 5));

	Widget *widgetButtons = new Widget(widget);
	widgetButtons->setLayout(new GridLayout(Orientation::Vertical, 2, Alignment::Fill, 15, 5));

	// create some buttons
	Button *b = new Button(widgetButtons, "Show Legend");
	b->setFlags(Button::ToggleButton);
	b->setChangeCallback([plot](bool state){
		plot->setShowLegend(state);
	});
	b = new Button(widgetButtons, "Show Ticks");
	b->setFlags(Button::ToggleButton);
	b->setChangeCallback([plot](bool state){
		plot->setShowTicks(state);
	});

	return PlotWidget(plot, widget);
}

void MotionPlanAnalysis::updatePlotScaling()
{
	for (auto &pp : plots) {
		Plot *plot= pp.second.plot;

		Vector2f dataMin = Vector2f(HUGE_VAL, HUGE_VAL);
		Vector2f dataMax = Vector2f(-HUGE_VAL, -HUGE_VAL);

		for (const auto &d: plot->dataColl()) {
			if(d.first == "time")
				continue;

			const PlotData &data = d.second;

			// get min/max
			for (int i = 0; i < 2; ++i) {
				dataMin[i] = std::min(dataMin[i], data.mMinVal[i]);
				dataMax[i] = std::max(dataMax[i], data.mMaxVal[i]);
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
}
