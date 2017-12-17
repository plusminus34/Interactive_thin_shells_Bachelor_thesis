#include <RobotDesignerLib/MotionPlanAnalysis.h>

#include <nanogui/nanogui.h>

#include <GUILib/ColorMaps.h>

using namespace nanogui;
using namespace Eigen;

MotionPlanAnalysis::MotionPlanAnalysis(nanogui::Screen *screen){

	window = new Window(screen, "Motion Plan Analysis");
	window->setLayout(new GridLayout(Orientation::Vertical, 2, Alignment::Middle, 15, 5));
	window->setVisible(false);
	window->setPosition({1200, 0});

	Widget *buttons = new Widget(window);
	buttons->setLayout(new GridLayout(Orientation::Horizontal, 10, Alignment::Minimum, 0, 5));

	Widget *widgetPlots = new Widget(window);
	widgetPlots->setLayout(new GridLayout(Orientation::Vertical, 4, Alignment::Minimum, 0, 5));

	// create a EE plots
	plots[WHEEL_SPEED] = makePlotWidget(widgetPlots, "Wheel Speed");
	plots[WHEEL_TILT_ANGLE] = makePlotWidget(widgetPlots, "Wheel Tilt Angle");
	plots[WHEEL_YAW_ANGLE] = makePlotWidget(widgetPlots, "Wheel Yaw Angle");
	plots[EE_POS_Y] = makePlotWidget(widgetPlots, "EE pos y");

	// create robot state plots
	plots[JOINT_ANGLES] = makePlotWidget(widgetPlots, "Joint Angles");
	plots[COM_POSITION] = makePlotWidget(widgetPlots, "COM Position");
	plots[COM_ORIENTATION] = makePlotWidget(widgetPlots, "COM Orientation");

	// create show/hide toggle buttons
	for (auto &plotWidget : plots) {
		Button *b = new Button(buttons, plotWidget.second.plot->caption());
		b->setPushed(true);
		b->setFlags(Button::ToggleButton);
		Widget *widget = plotWidget.second.widget;
		b->setChangeCallback([screen,widget](bool state){
			widget->setVisible(state);
			screen->performLayout();
		});

	}

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

			PlotData data(x, y, nanogui::Color(ColorMaps::getColorAt(ColorMaps::viridis, (float)index/(float)nEEs, 0.3f), 1.f));
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

			PlotData data(x, y, nanogui::Color(ColorMaps::getColorAt(ColorMaps::viridis, (float)index/(float)nEEs, 0.3f), 1.f));
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

			PlotData data(x, y, nanogui::Color(ColorMaps::getColorAt(ColorMaps::viridis, (float)index/(float)nEEs, 0.3f), 1.f));
			plots[WHEEL_YAW_ANGLE].plot->setPlotData("ee " + std::to_string(index), data);
		}

		// wheel yaw angle
		{
			int nTimeSteps = eeTraj.EEPos.size();
			VectorXf x(nTimeSteps);
			VectorXf y(nTimeSteps);
			for (int i = 0; i < nTimeSteps; ++i){
				x[i] = (float)i/((float)nTimeSteps-1);
				y[i] = eeTraj.EEPos[i][1];
			}

			PlotData data(x, y, nanogui::Color(ColorMaps::getColorAt(ColorMaps::plasma, (float)index/(float)nEEs), 1.f));
			plots[EE_POS_Y].plot->setPlotData("ee " + std::to_string(index), data);
		}

		index++;
	}

	const LocomotionEngine_RobotStateTrajectory &rsTraj = motionPlan->robotStateTrajectory;
	if(rsTraj.qArray.size() > 6)
	{
		int nTimeSteps = rsTraj.qArray.size();

		VectorXf x(nTimeSteps);
		for (int i = 0; i < nTimeSteps; ++i)
			x[i] = (float)i/((float)nTimeSteps-1);

		int nJoints = rsTraj.qArray[6].size()-6;
		std::vector<VectorXf> y(nJoints);
		for (int i = 0; i < nJoints; ++i) {
			y[i].resize(nTimeSteps);
		}

		for (int i = 0; i < nTimeSteps; ++i) {
			for (int j = 0; j < nJoints; ++j)
				y[j][i] = rsTraj.qArray[i][j+6];
		}

		for (int i = 0; i < nJoints; ++i) {
			PlotData data(x, y[i], nanogui::Color(ColorMaps::getColorAt(ColorMaps::plasma, (float)i/(float)(nJoints), 0.2f), 1.f));
			plots[JOINT_ANGLES].plot->setPlotData("q " + std::to_string(i+6), data);
		}
	}

	if(rsTraj.qArray.size() >= 3)
	{
		int nTimeSteps = rsTraj.qArray.size();

		VectorXf x(nTimeSteps);
		for (int i = 0; i < nTimeSteps; ++i)
			x[i] = (float)i/((float)nTimeSteps-1);

		std::vector<VectorXf> y(3);
		for (int i = 0; i < 3; ++i) {
			y[i].resize(nTimeSteps);
		}

		for (int i = 0; i < nTimeSteps; ++i) {
			for (int j = 0; j < 3; ++j)
				y[j][i] = rsTraj.qArray[i][j];
		}

		for (int i = 0; i < 3; ++i) {
			PlotData data(x, y[i], nanogui::Color(ColorMaps::getColorAt(ColorMaps::plasma, (float)i/(float)(3), 0.2f), 1.f));
			plots[COM_POSITION].plot->setPlotData("q " + std::to_string(i), data);
		}
	}

	if(rsTraj.qArray.size() >= 6)
	{
		int nTimeSteps = rsTraj.qArray.size();

		VectorXf x(nTimeSteps);
		for (int i = 0; i < nTimeSteps; ++i)
			x[i] = (float)i/((float)nTimeSteps-1);

		std::vector<VectorXf> y(3);
		for (int i = 0; i < 3; ++i) {
			y[i].resize(nTimeSteps);
		}

		for (int i = 0; i < nTimeSteps; ++i) {
			for (int j = 0; j < 3; ++j)
				y[j][i] = rsTraj.qArray[i][j+3];
		}

		for (int i = 0; i < 3; ++i) {
			PlotData data(x, y[i], nanogui::Color(ColorMaps::getColorAt(ColorMaps::plasma, (float)i/(float)(3), 0.2f), 1.f));
			plots[COM_ORIENTATION].plot->setPlotData("q " + std::to_string(i+3), data);
		}
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

PlotWidget MotionPlanAnalysis::makePlotWidget(nanogui::Widget *window, const std::string &name)
{
	Widget *widget = new Widget(window);
	widget->setLayout(new GridLayout(Orientation::Horizontal, 2, Alignment::Fill, 5, 5));

	Plot *plot = new Plot(widget, name);
	plot->setSize(Vector2i(400, 200));
//	plot->setBackgroundColor(nanogui::Color(Eigen::Vector3f(0.5, 0.5, 0.5), 0.1f));
	plot->setNumTicks(Vector2i(10, 5));

	Widget *widgetButtons = new Widget(widget);
	widgetButtons->setLayout(new GridLayout(Orientation::Vertical, 2, Alignment::Minimum, 0, 5));

	// create some buttons
	Button *b = new Button(widgetButtons, "Legend");
	b->setFlags(Button::ToggleButton);
	b->setChangeCallback([plot](bool state){
		plot->setShowLegend(state);
	});
	b = new Button(widgetButtons, "Ticks");
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
