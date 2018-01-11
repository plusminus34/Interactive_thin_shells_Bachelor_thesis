/*
 * Simple example that shows how to use the `Plot` class to plot data.
 */

#ifndef NANOGUI_GLAD
#define NANOGUI_GLAD
#endif // NANOUI_GLAD
#include <nanogui/opengl.h>

#include <GL/glu.h>

#include <nanogui/nanogui.h>
#include <GUILib/Plot.h>
#include <MathLib/MathLib.h>

using namespace nanogui;

int main(void){
	using namespace nanogui;

	// init nanogui, make screen and window
	nanogui::init();
	Screen *screen = new Screen(Vector2i(1000, 1000), "Plot Demo Screen");
	Window *window = new Window(screen, "Plot Demo Window");
	window->setLayout(new GroupLayout());

	// create a plot
	Plot *plot = new Plot(window, "my first plot");

	// add some data ...
	VectorXf x1(3); x1 << -0.68, 1, 2;
	VectorXf y1(3); y1 << 0, 2, 10;
	plot->setPlotData("data 1", PlotData(x1, y1, Color(Vector3f(1, 0, 0), 0.5), 4.f));

	// ... some more ...
	VectorXf x2(3); x2 << 0, 0.2, 2;
	VectorXf y2(3); y2 << -3.3f, 2, 4;
	plot->setPlotData("data 2", PlotData(x2, y2, Color(Vector3f(0, 1, 0), 1.f)));

	// ... a sine wave ...
	{
		int nPoints = 100;
		VectorXf xValues(nPoints);
		VectorXf yValues(nPoints);
		for (int i = 0; i < xValues.size(); ++i) {
			xValues[i] = 2.f*PI*(float)i/(float)xValues.size();
			yValues[i] = std::sin(xValues[i]);
		}

		PlotData data(xValues, yValues, Color(Vector3f(0, 0, 1), 1.f));
		plot->setPlotData("sine", data);
	}

	// ... and a cosine wave.
	{
		int nPoints = 100;
		VectorXf xValues(nPoints);
		VectorXf yValues(nPoints);
		for (int i = 0; i < xValues.size(); ++i) {
			xValues[i] = 2.f*PI*(float)i/(float)xValues.size();
			yValues[i] = std::cos(xValues[i]);
		}

		PlotData data(xValues, yValues, Color(Vector3f(1, 1, 0), 1.f));
		plot->setPlotData("cosine", data);
	}

	plot->updateMinMax();

	// add a line at y=0
	plot->setPlotData("data 3", PlotData(Vector2f(plot->dataMin()[0], plot->dataMax()[0]) , Vector2f(0,0), Color(Vector3f(0, 0, 0), 0.2), 2));

	plot->setFooter("Lots of plots");
//	plot->setBackgroundColor(Color(1.f,1.f,1.f,0.5f));
	plot->setSize(Vector2i(800, 800));

	Button *b = new Button(window, "Show Legend");
	b->setFlags(Button::ToggleButton);
	b->setChangeCallback([plot](bool state) { plot->setShowLegend(state); });

	b = new Button(window, "Show Ticks");
	b->setFlags(Button::ToggleButton);
	b->setChangeCallback([plot](bool state) { plot->setShowTicks(state); });


	Vector2f zoomMin(0, -1);
	Vector2f zoomMax(1, 1);
	std::stringstream bName; bName << "Zoom, to (" << zoomMin << ") x (" << zoomMax << ")";
	b = new Button(window, bName.str());
	b->setFlags(Button::ToggleButton);
	b->setChangeCallback([plot, zoomMin, zoomMax](bool state) {
		if(state){
			plot->setDataMin(zoomMin);
			plot->setDataMax(zoomMax);
		}
		else
			plot->updateMinMax();
	});

	screen->setVisible(true);
	screen->performLayout();
	window->center();

	nanogui::mainloop();

	nanogui::shutdown();
	return 0;
}


