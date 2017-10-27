#pragma once

#include <GUILib/GLApplication.h>
#include <string>
#include <map>
#include <GUILib/TranslateWidget.h>

#include <RBSimLib/AbstractRBEngine.h>
#include <RBSimLib/WorldOracle.h>

/**
 * Test App for RB Simulations
 */
class WheeledRobotSimApp : public GLApplication {
private:
	TranslateWidget tWidget;
	AbstractRBEngine* rbEngine = NULL;
    bool haveTested = false;

	bool drawMeshes = true, drawMOIs = false, drawCDPs = false, drawSkeletonView = false, drawJoints = false, drawContactForces = true;
	double simTimeStep;
	WorldOracle* worldOracle;

	std::string lastLoadedFile;

	double allWheelsAngle;

public:
	// constructor
    WheeledRobotSimApp(bool maximizeWindows = true);
	// destructor
    virtual ~WheeledRobotSimApp(void);
	// Run the App tasks
	virtual void process();
	// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
	virtual void drawScene();
	// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
	virtual void drawAuxiliarySceneInfo();
	// Restart the application.
	virtual void restart();

	//input callbacks...

	//all these methods should returns true if the event is processed, false otherwise...
	//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
	virtual bool onKeyEvent(int key, int action, int mods);
	//this one gets triggered on UNICODE characters only...
	virtual bool onCharacterPressedEvent(int key, int mods);
	//triggered when mouse buttons are pressed
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	//triggered when mouse moves
	virtual bool onMouseMoveEvent(double xPos, double yPos);
	//triggered when using the mouse wheel
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);

	virtual bool processCommandLine(const std::string& cmdLine);
	
	virtual void saveFile(const char* fName);
	virtual void loadFile(const char* fName);

private:
	inline void addSliderTextVariable(const std::string &name, double *var, const std::pair<double,double> &range, nanogui::Widget *widget, std::string units = "", int precision = 2);

	template<class T>
	static std::string toString(T value, int precision = 2);
};
