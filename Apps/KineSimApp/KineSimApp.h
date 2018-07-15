#pragma once

#include <GUILib/GLApplication.h>
#include <string>
#include <map>
#include <GUILib/TranslateWidget.h>

#include <KineSimLib/KS_MechanicalAssembly.h>
#include <KineSimLib/KS_UIMechanismController.h>


/**
 * Test App for Kinematic 3D simulations
 */
class KineSimApp : public GLApplication {
private:
	KS_MechanicalAssembly * mech1 = NULL;
	KS_UIMechanismController* uiController = NULL;
	KS_IKMechanismController* ikController = NULL;
	bool checkDerivatives = false;
	bool uiControl = false, ikControl = false;
	double simTimeStep = 1;
	bool logState = false;
	//dVector motorAngleValues;
	//dVector startingMechState;



public:
	// constructor
	KineSimApp();
	// destructor
	virtual ~KineSimApp(void);
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
};



