#include <GUILib/GLApplication.h>
#include <GUILib/GLUtils.h>
#include <Utils/Utils.h>
#include <GUILib/GLTrackingCamera.h>
#include <GUILib/GLContentManager.h>
#include <GUILib/GLTexture.h>
#include <GUILib/GLShaderMaterial.h>
#include "Utils/BMPIO.h"

#include <nanogui/screen.h>

GLApplication* glAppInstance = NULL;

GLApplication::GLApplication(int x, int y, int w, int h, bool maximizeWindows){
	GLApplication::setGLAppInstance(this);

	if (!glfwInit()) {
		// An error occured
		Logger::print("GLFW initialization failed\n");
		exit(0);
	}

    init(x, y, w, h, maximizeWindows);
}

GLApplication::GLApplication(bool maximizeWindows) {
	GLApplication::setGLAppInstance(this);

#ifdef WIN32
	SetCurrentDirectory((LPCTSTR)SCP_WORKING_DIR);
	TCHAR pwd[MAX_PATH];
	GetCurrentDirectory(MAX_PATH, pwd);
	std::cout << "Working directory: " << pwd << std::endl;
#endif // WIN32

	if (!glfwInit()) {
		// An error occured
		Logger::print("GLFW initialization failed\n");
		exit(0);
	}

	const GLFWvidmode * mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

    int borderLeft = 4;
    int borderTop = 42;
    int borderRight = 4;
    int borderBottom = 60;

    init(borderLeft, borderTop, (mode->width - borderLeft - borderRight), (mode->height - borderTop - borderBottom), maximizeWindows);
}


void GLFW_error(int error, const char* description)
{
    fputs(description, stderr);
}

void GLApplication::init(int x, int y, int w, int h, bool maximizeWindows) {

#ifdef GUI_TWO_WINDOWS
	// Ignore window position and size if maximizeWindow is requested
	if(maximizeWindows)
	{
		x = 600; y = 0;
		w = 800; h = 800;
	}
#endif

	glfwSetErrorCallback(GLFW_error);

	/* Create a windowed mode window and its OpenGL context */
	glfwWindow = glfwCreateWindow(w, h, "", NULL, NULL);
	if (!glfwWindow) {
		Logger::print("Could not initialize GLFW window\n");
		glfwTerminate();
		exit(0);
	}
	setWindowTitle("Simulation And Control Playground");
	glfwSetWindowPos(glfwWindow, x, y);

#if defined(_WIN32) || defined(__linux__)
	float mPixelRatio = nanogui::get_pixel_ratio(glfwWindow);
	glfwSetWindowSize(glfwWindow, (int)(w / mPixelRatio), (int)(h / mPixelRatio));
#endif

	/* Make the window's context current */
	glfwMakeContextCurrent(glfwWindow);

#if defined(NANOGUI_GLAD)
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		throw std::runtime_error("Could not initialize GLAD!");
	glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM
#endif

	//setup callback functions...

	//mouse move
	glfwSetCursorPosCallback(glfwWindow, [](GLFWwindow *, double x, double y) {
		if (GLApplication::getGLAppInstance()->menuScreen->cursorPosCallbackEvent(x, y))
			return;
		GlobalMouseState::updateMouseMove(x, y);
		//	Logger::print("mouse move event: xPos: %lf, yPos: %lf\n", xPos, yPos);
		GLApplication::getGLAppInstance()->onMouseMoveEvent(x, y);
	});

	//mouse click...
	glfwSetMouseButtonCallback(glfwWindow, [](GLFWwindow * glfwWindow, int button, int action, int modifiers) {
		if (GLApplication::getGLAppInstance()->menuScreen->mouseButtonCallbackEvent(button, action, modifiers))
			return;
		double xPos, yPos;
		glfwGetCursorPos(glfwWindow, &xPos, &yPos);
		GlobalMouseState::updateMouseState(xPos, yPos, button, action, modifiers);
		//Logger::print("mouse button event: button %d, action: %d, mods: %d, xPos: %lf, yPos: %lf\n", button, action, mods, xPos, yPos);
		GLApplication::getGLAppInstance()->onMouseButtonEvent(button, action, modifiers, xPos, yPos);
	});

	//mouse scroll...
	glfwSetScrollCallback(glfwWindow, [](GLFWwindow *, double x, double y) {
		if (GLApplication::getGLAppInstance()->menuScreen->scrollCallbackEvent(x, y))
			return;
		GLApplication::getGLAppInstance()->onMouseWheelScrollEvent(x, y);
	});

	//keyboard events...
	glfwSetKeyCallback(glfwWindow, [](GLFWwindow *, int key, int scancode, int action, int mods) {
		if (GLApplication::getGLAppInstance()->menuScreen->keyCallbackEvent(key, scancode, action, mods))
			return;
		//	Logger::print("key press: %d (%c), scancode: %d (%c), key event: %d, mods: %d\n", key, key, scancode, scancode, action, mods);
		GLApplication::getGLAppInstance()->onKeyEvent(key, action, mods);
	});

	glfwSetCharCallback(glfwWindow, [](GLFWwindow *, unsigned int codepoint) {
		GLApplication::getGLAppInstance()->menuScreen->charCallbackEvent(codepoint);
	});

	glfwSetCharModsCallback(glfwWindow, [](GLFWwindow *, unsigned int key, int mods) {
		//	Logger::print("char event: %d (%c), mods: %d\n", key, key, mods);
		GLApplication::getGLAppInstance()->onCharacterPressedEvent(key, mods);
	});

	//dropping files into the window...
	glfwSetDropCallback(glfwWindow, [](GLFWwindow *, int count, const char **filenames) {
		if (GLApplication::getGLAppInstance()->menuScreen->dropCallbackEvent(count, filenames))
			return;
		GLApplication::getGLAppInstance()->loadFiles(count, filenames);
	});

	//resize window...
	glfwSetFramebufferSizeCallback(glfwWindow, [](GLFWwindow *, int width, int height) {
		if (GLApplication::getGLAppInstance()->menuScreen->resizeCallbackEvent(width, height))
			return;
		GLApplication::getGLAppInstance()->adjustWindowSize(width, height);
	});

	GLContentManager::addShaderProgram("diffuse", "../data/shaders/diffuse/diffuse.vert", "../data/shaders/diffuse/diffuse.frag");
	GLContentManager::addShaderProgram("diffuseTextured", "../data/shaders/diffuse/diffuse.vert", "../data/shaders/diffuse/diffuse_textured.frag");
	GLContentManager::addShaderProgram("diffuseTexturedNormalMapped", "../data/shaders/diffuse/diffuse_textured_normalmap.vert", "../data/shaders/diffuse/diffuse_textured_normalmap.frag");
	GLContentManager::addShaderProgram("rim", "../data/shaders/diffuse/diffuse.vert", "../data/shaders/diffuse/diffuse_rim.frag");
	GLContentManager::addShaderProgram("matcap", "../data/shaders/matcap/matcap.vert", "../data/shaders/matcap/matcap.frag");
	GLContentManager::addShaderProgram("matcapBumped", "../data/shaders/matcap/matcap_bump.vert", "../data/shaders/matcap/matcap_bump.frag");
	GLContentManager::addShaderProgram("specularBumped", "../data/shaders/specular/diffuse_spec_bump.vert", "../data/shaders/specular/diffuse_spec_bump.frag");
	GLContentManager::addShaderProgram("specularBumpedMatcap", "../data/shaders/specular/diffuse_spec_bump.vert", "../data/shaders/specular/diffuse_spec_bump_matcap.frag");
	GLContentManager::addShaderProgram("toonShader", "../data/shaders/toon/toon.vert", "../data/shaders/toon/toon.frag");
	GLContentManager::addShaderProgram("silhouette", "../data/shaders/toon/silhouette.vert", "../data/shaders/toon/silhouette.frag");
	GLContentManager::addShaderProgram("radialGradientShader", "../data/shaders/radialGradient/radialGradient.vert", "../data/shaders/radialGradient/radialGradient.frag");

	GLShaderMaterial* material = new GLShaderMaterial();
	material->readFromFile("../data/shaders/radialGradient/radialGradient.mat");
	GLContentManager::addShaderMaterial(material->getMaterialName().c_str(), material);

	setupMainMenu();

	int width, height;
	glfwGetFramebufferSize(glfwWindow, &width, &height);
	glViewport(0, 0, width, height);
	glfwSwapInterval(0);
	glfwSwapBuffers(glfwWindow);
	adjustWindowSize(width, height);

	setupOpenGL();

	consoleWindow = new GLConsole(0, 0, width, 280);

	camera = new GLTrackingCamera();

#ifdef GUI_TWO_WINDOWS
	// Set nano gui window width and let glfwWindow fill out rest of monitor space
	if(maximizeWindows)
	{
		int menuScreenWidth = 400; // width of the nano gui menu
		GLFWmonitor *primaryMonitor = glfwGetPrimaryMonitor();
		int primaryMonitorX, primaryMonitorY;
		glfwGetMonitorPos(primaryMonitor, &primaryMonitorX, &primaryMonitorY);
		const GLFWvidmode * mode = glfwGetVideoMode(primaryMonitor);
		int monitor_width = mode->width;
		int monitor_height = mode->height;
		glfwSetWindowSize(glfwWindow, monitor_width-menuScreenWidth, monitor_height);
		glfwSetWindowSize(menuScreen->glfwWindow(), menuScreenWidth, monitor_height);
		glfwSetWindowPos(glfwWindow, primaryMonitorX + menuScreenWidth, 0);
		glfwSetWindowPos(menuScreen->glfwWindow(), primaryMonitorX, 0);
	}
#endif
}

void GLApplication::setupMainMenu() {

#ifdef GUI_TWO_WINDOWS
	// Let's make a new nano gui window
	nanogui::init();
	menuScreen = new nanogui::Screen({400, 800}, "NanoGUI Test", true, false, 8, 8, 24, 8, 0, 3, 3);
#else
	// Create a nanogui screen and pass the glfw pointer to initialize
	menuScreen = new nanogui::Screen();
	menuScreen->initialize(glfwWindow, true);
#endif // GUI_TWO_WINDOWS

	mainMenu = new nanogui::FormHelper(menuScreen);

	mainMenu->addWindow(Eigen::Vector2i(0, 0), "Main Menu");
	mainMenu->addGroup("Visualization");

	nanogui::Widget *visTools = new nanogui::Widget(mainMenu->window());
	mainMenu->addWidget("", visTools);
	visTools->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 4));
	nanogui::Button* button;

//	button = new nanogui::Button(tools, "");
//	button->setFlags(nanogui::Button::ToggleButton);
//	button->setChangeCallback([this, button](bool val) {  waitForFrameRate = val; });
//	button->setIcon(ENTYPO_ICON_SAVE);
//	button->setTooltip("Wait For Framerate");

	button = new nanogui::Button(visTools, ">>_");
	button->setFlags(nanogui::Button::ToggleButton);
	button->setChangeCallback([this, button](bool val) {  showConsole = val; });
	button->setPushed(showConsole);
	button->setTooltip("Toggle console");

	button = new nanogui::Button(visTools, "FPS");
	button->setFlags(nanogui::Button::ToggleButton);
	button->setChangeCallback([this, button](bool val) {  showFPS = val; });
	button->setPushed(showFPS);
	button->setTooltip("Show FPS");

	button = new nanogui::Button(visTools, "");
	button->setFlags(nanogui::Button::ToggleButton);
	button->setChangeCallback([this, button](bool val) {  showReflections = val; });
	button->setPushed(showReflections);
	button->setIcon(ENTYPO_ICON_SLIDESHARE);
	button->setTooltip("Show Reflections");

	button = new nanogui::Button(visTools, "");
	button->setFlags(nanogui::Button::ToggleButton);
	button->setChangeCallback([this, button](bool val) {  showGroundPlane = val; });
	button->setIcon(ENTYPO_ICON_LOCATION);
	button->setPushed(showGroundPlane);
	button->setTooltip("Show Ground Plane");

	mainMenu->addGroup("Playback Controls                          ");

	nanogui::Widget *tools = new nanogui::Widget(mainMenu->window());
	mainMenu->addWidget("", tools);
	tools->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 4));

	button = new nanogui::Button(tools, "");
	button->setCallback([this]() { restart(); });
	button->setIcon(ENTYPO_ICON_CCW);
	button->setTooltip("Restart");

	button = new nanogui::Button(tools, "");
	button->setFlags(nanogui::Button::ToggleButton);
	button->setIcon(ENTYPO_ICON_CONTROLLER_PLAY);
	button->setChangeCallback([this, button](bool val) {  appIsRunning = val; });
	button->setTooltip("Play/Pause");
	playButton = button;

	button = new nanogui::Button(tools, "");
	button->setCallback([this]() { process(); });
	button->setIcon(ENTYPO_ICON_CONTROLLER_NEXT);
	button->setTooltip("step");

	button = new nanogui::Button(tools, "");
	button->setFlags(nanogui::Button::ToggleButton);
	button->setIcon(ENTYPO_ICON_HOUR_GLASS);
	button->setChangeCallback([this, button](bool val) {  slowMo = val; });
	button->setTooltip("Slow Motion");

	button = new nanogui::Button(tools, "");
	button->setFlags(nanogui::Button::ToggleButton);
	button->setIcon(ENTYPO_ICON_VIDEO_CAMERA);
	button->setChangeCallback([this, button](bool val) {  followCameraTarget = val; });
	button->setTooltip("Follow camera target");

	menuScreen->setVisible(true);
	menuScreen->performLayout();

	menuScreen->setPosition(Eigen::Vector2i(0, 0));
}


//triggered when mouse moves
bool GLApplication::onMouseMoveEvent(double xPos, double yPos) {
//	if (showMenus)

	if (showConsole)
		consoleWindow->onMouseMoveEvent(xPos, yPos);
	if (camera)
		camera->onMouseMoveEvent(xPos, yPos);
	return false;
}

//triggered when mouse buttons are pressed
bool GLApplication::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
//	if (showMenus)

	if (showConsole)
		consoleWindow->onMouseButtonEvent(button, action, mods, xPos, yPos);
	if (camera)
		if (camera->onMouseButtonEvent(button, action, mods, xPos, yPos))
		return true;

	return false;
}

//triggered when using the mouse wheel
bool GLApplication::onMouseWheelScrollEvent(double xOffset, double yOffset) {
//	if (showMenus)
//			return true;
	if (camera)
		if (camera->onMouseWheelScrollEvent(xOffset, yOffset))
			return true;
	return false;
}

bool GLApplication::onKeyEvent(int key, int action, int mods) {
//	Logger::consolePrint("It's a key event...action: %d\n", action);

	//listen to key presses even if the menu is not visible...
	if (key == GLFW_KEY_ESCAPE) {
		glfwSetWindowShouldClose(glfwWindow, GL_TRUE);
		return true;
	}

	//if the console is active and listening, it means we're typing in it, so let it handle it first
	if (consoleWindow->onKeyEvent(key, action, mods))
		return true;

//	if (TwEventKeyGLFW(TwConvertKeyGLFW3to2(key), action))
//		return true;

	if (camera)
		if (camera->onKeyEvent(key, action, mods))
			return true;

	return false;
}

bool GLApplication::onCharacterPressedEvent(int key, int mods) {
//	Logger::consolePrint("It's a char pressed event...\n");


	//if the console is active and listening, it means we're typing in it, so let it handle it first
	if (consoleWindow->onCharacterPressedEvent(key, mods))
		return true;

	//we listen to the keys even if the menu is not visible...
//	if (TwEventCharGLFW(TwConvertKeyGLFW3to2(key), GLFW_PRESS))
//		return true;

	if (camera)
		if (camera->onCharacterPressedEvent(key, mods))
			return true;

	if (key == ' ') {
		appIsRunning = !appIsRunning;
		playButton->setPushed(appIsRunning);
		if (appIsRunning) {
			processTimer.restart();
			Logger::consolePrint("process timer started\n");
		}
		else
			Logger::consolePrint("process time: %lfs\n", processTimer.timeEllapsed());
	}

	return false;
}

void GLApplication::loadFiles(int nFiles, const char** fNames) {
	for (int i = 0;i < nFiles;i++)
		GLApplication::getGLAppInstance()->loadFile(fNames[i]);
}

void GLApplication::loadFile(const char* fName) {
	Logger::consolePrint("LOAD FILE: Do not know what to do with file \'%s\'\n", fName);
}

void GLApplication::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}

void GLApplication::setWindowTitle(char* windowTitle) {
	glfwSetWindowTitle(glfwWindow, windowTitle);
}

GLApplication::~GLApplication(void){
	// terminate GLFW
	glfwTerminate();
}

void GLApplication::runMainLoop() {
	// Main loop (repeated while window is not closed and [ESC] is not pressed)
#ifdef GUI_TWO_WINDOWS
	while (!glfwWindowShouldClose(glfwWindow) && !glfwWindowShouldClose(menuScreen->glfwWindow())){
#else
	while (!glfwWindowShouldClose(glfwWindow)){
#endif
		double timeSpentProcessing = 0;
		fpsTimer.restart();
		if (appIsRunning) process();
		timeSpentProcessing = fpsTimer.timeEllapsed();

#ifdef GUI_TWO_WINDOWS
		glfwMakeContextCurrent(glfwWindow);
#endif
		draw();
#ifndef GUI_TWO_WINDOWS
		mainMenu->refresh();
		menuScreen->drawWidgets();
#endif

		//wait until the required ammount of time has passed (respect the desired FPS requirement)
		if (waitForFrameRate)
			while (fpsTimer.timeEllapsed()< 1.0 / desiredFrameRate);

		if (showFPS)
			drawFPS(fpsTimer.timeEllapsed(), timeSpentProcessing / fpsTimer.timeEllapsed());

		/* Swap front and back buffers */
		glfwSwapBuffers(glfwWindow);

#ifdef GUI_TWO_WINDOWS
		// draw the nanogui
		if(menuScreen)
		{
			glfwMakeContextCurrent(menuScreen->glfwWindow());

			glClearColor(0.3, 0.3, 0.3, 1);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

			mainMenu->refresh();
			menuScreen->drawWidgets();

			glfwSwapBuffers(menuScreen->glfwWindow());
		}
#endif

		if(screenRecorder != NULL) {screenRecorder->call(glfwWindow);}

		/* Poll for and process events */
		glfwPollEvents();
	}
}

/**
* draw information regarding the frame rate and performance
*/
void GLApplication::drawFPS(double timeSinceLastUpdate, double percentageOfTimeSpentProcessing) {
	//in order to avoid writing the a new fps rate at every redraw, we will instead compute averages and display those. We need: the total ellapsedTime since the last update
	static double ellapsedTime = 0;
	//the frame rate that is currently being displayed
	static double oldFrameRate = 0;
	//the number of frames that we've counted so far
	static int nrFramesSinceUpdate = 0;
	//the total time spent processing
	static double processingTime = 0;
	//and the processing time that we are displaying
	static double oldPerformanceRate = 0;

	ellapsedTime += timeSinceLastUpdate;
	processingTime += percentageOfTimeSpentProcessing;
	nrFramesSinceUpdate++;

	//only change the numbers that we display about 3 times per second
	if (ellapsedTime >= 1 / 3.0) {
		oldFrameRate = nrFramesSinceUpdate / ellapsedTime;
		oldPerformanceRate = processingTime / nrFramesSinceUpdate;
		nrFramesSinceUpdate = 0;
		ellapsedTime = 0;
		processingTime = 0;
	}

	glPushMatrix();
	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -1.0f);

	glColor4d(0.75, 0.0, 0.0, 1.0);
	int viewportWidth, viewportHeight;
	getMainWindowWidthAndHeight(viewportWidth, viewportHeight);
	glPushAttrib(GL_COLOR); {
		glColor4f(1., 1., 1., 1.);
		// --
		glprint(viewportWidth - 400, viewportHeight - 20, "FPS: %7.2lf (processing: %7.2lf %%)", oldFrameRate, 100 * oldPerformanceRate);
	} glPopAttrib();
	glPopMatrix();
}

// Run the App tasks
void GLApplication::process() {
	//do the work here...
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void GLApplication::drawScene() {
	int pass, numPass;
	bool wireframe = false;
	unsigned char cubeColor[] = { 255, 0, 0, 128 }; // Model color (32bits RGBA)

	glMatrixMode(GL_MODELVIEW);

	glBegin(GL_LINES);
	glColor3d(1, 0, 0);
	glVertex3d(0, 0, 0);
	glVertex3d(1, 0, 0);

	glColor3d(0, 1, 0);
	glVertex3d(0, 0, 0);
	glVertex3d(0, 1, 0);

	glColor3d(0, 0, 1);
	glVertex3d(0, 0, 0);
	glVertex3d(0, 0, 1);
	glEnd();

	glTranslated(-0.5, -0.5, -0.5);

	// Set color and draw model
	glColor4ubv(cubeColor);

	glLineWidth(3.0);

	if (wireframe){
		glDisable(GL_CULL_FACE);
		glDisable(GL_LIGHTING);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		numPass = 1;
	}
	else{
		glEnable(GL_CULL_FACE);
		glEnable(GL_LIGHTING);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		numPass = 2;
	}

	for (pass = 0; pass<numPass; ++pass){
		// Since the material could be transparent, we draw the convex model in 2 passes:
		// first its back faces, and second its front faces.
		glCullFace((pass == 0) ? GL_FRONT : GL_BACK);

		// Draw the model (a cube)
		glBegin(GL_QUADS);
		glNormal3f(0, 0, -1); glVertex3f(0, 0, 0); glVertex3f(0, 1, 0); glVertex3f(1, 1, 0); glVertex3f(1, 0, 0); // front face
		glNormal3f(0, 0, +1); glVertex3f(0, 0, 1); glVertex3f(1, 0, 1); glVertex3f(1, 1, 1); glVertex3f(0, 1, 1); // back face
		glNormal3f(-1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(0, 0, 1); glVertex3f(0, 1, 1); glVertex3f(0, 1, 0); // left face
		glNormal3f(+1, 0, 0); glVertex3f(1, 0, 0); glVertex3f(1, 1, 0); glVertex3f(1, 1, 1); glVertex3f(1, 0, 1); // right face
		glNormal3f(0, -1, 0); glVertex3f(0, 0, 0); glVertex3f(1, 0, 0); glVertex3f(1, 0, 1); glVertex3f(0, 0, 1); // bottom face
		glNormal3f(0, +1, 0); glVertex3f(0, 1, 0); glVertex3f(0, 1, 1); glVertex3f(1, 1, 1); glVertex3f(1, 1, 0); // top face
		glEnd();
	}

	glLineWidth(1.0);
	glDisable(GL_LIGHTING);
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void GLApplication::drawAuxiliarySceneInfo() {

}

// Restart the application.
void GLApplication::restart() {

}

/**
 *  Initializes the openGL settings
 */
void GLApplication::setupOpenGL(){
//	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable(GL_LINE_SMOOTH);

	glShadeModel(GL_SMOOTH);
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA ,GL_ONE_MINUS_SRC_ALPHA);
}

/**
 *	This method is used to draw the scene.
 */
void GLApplication::draw(){
	//clear the screen
	glClearColor((GLfloat)bgColorR, (GLfloat)bgColorG, (GLfloat)bgColorB, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	//setup the viewport and the perspective transformation matrix
	int viewportWidth, viewportHeight;
	getMainWindowWidthAndHeight(viewportWidth, viewportHeight);
	glViewport(0, 0, viewportWidth, viewportHeight);
	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();									// Reset The Projection Matrix
	gluPerspective(45.0, (double)viewportWidth / viewportHeight, 0.05, 500.0);
	glMatrixMode(GL_MODELVIEW);							// Select The Modelview Matrix
	glLoadIdentity();									// Reset The Modelview Matrix

	glEnable(GL_DEPTH_TEST); //Enable depth testing

	if (camera){
		if (followCameraTarget)
			camera->followTarget(getCameraTarget());
		camera->applyCameraTransformations();
	}else
		gluLookAt(-1, 0, 3, 0, 0, 0, 0, 1, 0);

	setupLights();

	if (showDesignEnvironmentBox)
		drawDesignEnvironment();

	if (showGroundPlane)
		drawGroundAndReflections();

	glEnable(GL_COLOR_MATERIAL);
	drawScene();

	drawAuxiliarySceneInfo();

	if (showConsole) consoleWindow->draw();

}

//adjusts the window size
void GLApplication::adjustWindowSize(int w, int h) {
	//could reset the console window and menus...
}

GLApplication* GLApplication::getGLAppInstance() {
	return glAppInstance;
}

void GLApplication::setGLAppInstance(GLApplication* instance) {
	if (glAppInstance == NULL)
		glAppInstance = instance;
}

bool GLApplication::processCommandLine(const std::string& cmdLine) {
	std::vector<std::string> lines;
	getCharSeparatedStringList(cmdLine.c_str(), lines, ' ');

	if (strcmp(lines[0].c_str(), "clear") == 0) {
		Logger::consoleOutput.clear();
		return true;
	}

	if (strcmp(lines[0].c_str(), "save") == 0 && cmdLine.length() > strlen("save")) {
		saveFile(cmdLine.c_str() + strlen("save") + 1);
		return true;
	}

	if (strcmp(lines[0].c_str(), "load") == 0 && cmdLine.length() > strlen("load")) {
		loadFile(cmdLine.c_str() + strlen("load") + 1);
		return true;
	}

	return false;
}
