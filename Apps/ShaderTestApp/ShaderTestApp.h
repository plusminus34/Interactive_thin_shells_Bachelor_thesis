#pragma once

#include <GUILib/GLApplication.h>
#include <string>
#include <map>

#include <GUILib/TranslateWidget.h>

/**
 * Test App for shaders
 */
class ShaderTestApp : public GLApplication {
private:
	//this is a list of all objects that was loaded
	DynamicArray<std::string> loadedObjs;

	int selectedObjFile = 0;

	DynamicArray<std::string> materialTextures;
	int selectedMaterialTexture = 0;

	float modelColor[4] = { 1.0f, 1.0f, 1.0f, 1.0f};

public:
	// constructor
	ShaderTestApp();
	// destructor
	virtual ~ShaderTestApp(void);
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



