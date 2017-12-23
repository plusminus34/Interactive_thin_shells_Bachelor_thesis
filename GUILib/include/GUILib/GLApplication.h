#pragma once
#include "GLIncludes.h"
#include <nanogui/nanogui.h>
#include "GLWindow.h"
#include <Utils/Logger.h>
#include <Utils/Timer.h>
#include "GLWindow2D.h"
#include "GLConsole.h"
#include "GLCamera.h"
#include "GLContentManager.h"
#include "GlobalMouseState.h"

#include "Canvas3D.h"

#pragma warning( disable : 4005)

/**
 * All Apps will instantiate this base class...
 */
class GLApplication : public InteractiveWidget, public Canvas3D {
public:
	static GLApplication *getGLAppInstance();
	static void setGLAppInstance(GLApplication* instance);

public:
	// Pointer to glfw window
	GLFWwindow* glfwWindow;
//    GLFWwindow* glfwWindowNano;

	bool appIsRunning = false;
	bool waitForFrameRate = true;

    nanogui::Screen *menuScreen = nullptr;
	nanogui::FormHelper *mainMenu = nullptr;
	nanogui::Button* playButton = nullptr;

	void setupMainMenu();

protected:
	//keep a timer here to see how long it's been since the last redraw
	Timer fpsTimer;
	//keep a timer for measuring app processing time
	Timer processTimer;
	// Console window
	GLConsole* consoleWindow;

	//this is the desired frame rate, specified in FPS
	double desiredFrameRate = 30;
	double animationSpeedupFactor = 1.0;

	// Sets up various settings for OpenGL
	void setupOpenGL();

	// Main draw function - draws app content, shadows, reflections, etc...
	virtual void draw();
	
	// Draw information regarding the frame rate and performance
	void drawFPS(double timeSinceLastUpdate, double percentageOfTimeSpentProcessing);
    void init(int x, int y, int w, int h, bool maximizeWindows);

protected:
	bool slowMo = false;
	bool showMenus = true;
	bool showFPS = true;
	bool showConsole = true;

public:
	// constructors
    GLApplication(int x, int y, int w, int h, bool maximizeWindows = false);
    GLApplication(bool maximizeWindows = false);

	// destructor
	virtual ~GLApplication(void);

	// Run the App tasks
	virtual void process();
	// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
	virtual void drawScene();

	// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
	virtual void drawAuxiliarySceneInfo();
	// Restart the application.
	virtual void restart();

	// Run the main loop
	virtual void runMainLoop();

	// adjusts the window size
	void setWindowTitle(char* windowTitle);

	//adjusts the size of the window
	virtual void adjustWindowSize(int w, int h);

	int getMainWindowWidth() {
		int width, height;
		glfwGetFramebufferSize(glfwWindow, &width, &height);
		return width;
	}

	int getMainWindowHeight() {
		int width, height;
		glfwGetFramebufferSize(glfwWindow, &width, &height);
		return height;
	}

	void getMainWindowWidthAndHeight(int &width, int &height) {
		glfwGetFramebufferSize(glfwWindow, &width, &height);
	}
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
	virtual void loadFiles(int n, const char** fNames);

	virtual P3D getCameraTarget() { return P3D(0, 1, 0); }
};

inline void GLVertex3d(const P3D& p) {
	glVertex3d(p.x(), p.y(), p.z());
}

#include <unordered_map>

const static std::unordered_map<int, int> glfw3to2_keymapping =
{
	// Keyboard key definitions [GLFW3 -> GLFW2]
	{ 255,	256 },
	{ 256,	257 },
	{ 290,	258 },
	{ 291,	259 },
	{ 292,	260 },
	{ 293,	261 },
	{ 294,	262 },
	{ 295,	263 },
	{ 296,	264 },
	{ 297,	265 },
	{ 298,	266 },
	{ 299,	267 },
	{ 300,	268 },
	{ 301,	269 },
	{ 302,	270 },
	{ 303,	271 },
	{ 304,	272 },
	{ 305,	273 },
	{ 306,	274 },
	{ 307,	275 },
	{ 308,	276 },
	{ 309,	277 },
	{ 310,	278 },
	{ 311,	279 },
	{ 312,	280 },
	{ 313,	281 },
	{ 314,	282 },
	{ 265,	283 },
	{ 264,	284 },
	{ 263,	285 },
	{ 262,	286 },
	{ 340,	287 },
	{ 344,	288 },
	{ 341,	289 },
	{ 345,	290 },
	{ 342,	291 },
	{ 346,	292 },
	{ 258,	293 },
	{ 257,	294 },
	{ 259,	295 },
	{ 260,	296 },
	{ 261,	297 },
	{ 266,	298 },
	{ 267,	299 },
	{ 268,	300 },
	{ 269,	301 },
	{ 320,	302 },
	{ 321,	303 },
	{ 322,	304 },
	{ 323,	305 },
	{ 324,	306 },
	{ 325,	307 },
	{ 326,	308 },
	{ 327,	309 },
	{ 328,	310 },
	{ 329,	311 },
	{ 331,	312 },
	{ 332,	313 },
	{ 333,	314 },
	{ 334,	315 },
	{ 330,	316 },
	{ 336,	317 },
	{ 335,	318 },
};

inline int TwConvertKeyGLFW3to2(int key){
	auto itr = glfw3to2_keymapping.find(key);
	if (itr != glfw3to2_keymapping.end())
		return itr->second;

	return key;
}



