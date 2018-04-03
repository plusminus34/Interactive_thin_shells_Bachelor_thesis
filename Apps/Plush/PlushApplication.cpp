#include "PlushApplication.h" 
#include "P2DDragger.h"
 
PlushApplication::PlushApplication() {

    setWindowTitle("PlushApplication");
	appIsRunning = true;
	// --
	resetCamera();
	showConsole = false;
	showGroundPlane = false;
	bgColorR = 0.; bgColorG = 0.; bgColorB = 0.; bgColorA = 1.;
	// --
	push_back_handler(new P2DDragger({ &test_P3D }));
	// --
	// menuScreen->removeChild(0);
	// mainMenu->addWindow(Eigen::Vector2i(0, 0), "Main Menu");
	// mainMenu->addGroup("basic");
	// mainMenu->addVariable("test_int", test_int);
	// mainMenu->addVariable("test_double", test_double);
	// mainMenu->addVariable("RESET_CAMERA", RESET_CAMERA);
	// mainMenu->addVariable("PRINT_CAMERA", PRINT_CAMERA);
	// mainMenu->addVariable("STEP", STEP);
	// mainMenu->addVariable("RECORD", RECORD_VIDEO);

	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	// glEnable(GL_POLYGON_SMOOTH);
	// --
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	// glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);


}

void PlushApplication::drawScene() {
	// TODO: These should all have accompanying bool's that default to true in PlushApplication.
	incrementTime(); // FORNOW: NOTE: This should morally be in process.
	processToggles();
	flashError(); 
	if (DRAW_HANDLERS) { drawHandlers(); }

	if (DRAW_TEST_POINT) {
		glMasterPush(); {
			glPointSize(15.); set_color(WHITE);
			glBegin(GL_POINTS); { glP3D(test_P3D); } glEnd();
			// --
			glPointSize(10.); set_color(BLACK);
			glBegin(GL_POINTS); { glP3D(test_P3D); } glEnd();
		} glMasterPop();
	}

	if (!appIsRunning) {
		if (STEP) {
			STEP = false;
			appIsRunning = true;
			// --
			STEPPED = true;
		}
	} else {
		if (STEPPED) {
			STEPPED = false;
			// --
			appIsRunning = false; 
		} 
	}

}

void PlushApplication::process() { }

void PlushApplication::incrementTime() {
	if (appIsRunning) { t += .01; }
}
 
void PlushApplication::processToggles() {
	if (RESET_CAMERA) { RESET_CAMERA = false; resetCamera(); } 
	if (PRINT_CAMERA) { PRINT_CAMERA = false; printCamera(); } 
}


void PlushApplication::flashError() {
	if (BERN_ERROR) {
		BERN_ERROR = false;
		BERN_ERROR_REPORTING = true;
	}

	if (BERN_ERROR_REPORTING) {
		BERN_ERROR_COOLDOWN--;

		set_color(color_swirl(1. - BERN_ERROR_COOLDOWN / double(N_COOLDOWN), RED, BLACK));
		big_bad_panic_rectangle();

		if (BERN_ERROR_COOLDOWN == 0) {
			BERN_ERROR_REPORTING = false;
		}
	}
}

void PlushApplication::drawHandlers() { 
	for (auto &handler : handlers) {
		handler->draw();
	}
}

void PlushApplication::recordVideo() { 
	if (RECORDING) {
		char fName[1024]; sprintf(fName, "C:\\Users\\Jim\\Desktop\\recordings\\ss%05d.bmp", ++FRAME_i);
		int viewportSettings[4]; //x, y, w, h
		glGetIntegerv(GL_VIEWPORT, viewportSettings);
		saveScreenShot(fName, 0, 0, getMainWindowWidth(), getMainWindowHeight());
		// double W = mainWindowWidth;
		// double H = mainWindowHeight;
		// saveScreenShot(fName, .33*W, .33*H, .66*W, .66*H);
	} 

	// (*) Leave me second.
	if (RECORD_VIDEO) {
		RECORD_VIDEO = false;
		RECORDING = !RECORDING;
		FRAME_i = 0;
	}
}

void PlushApplication::resetCamera() {
	((GLTrackingCamera *)camera)->rotAboutRightAxis = DEFAULT_CAM_ROT_ABOUT_RIGHT_AXIS;
	((GLTrackingCamera *)camera)->rotAboutUpAxis    = DEFAULT_CAM_ROT_ABOUT_UP_AXIS___;
	((GLTrackingCamera *)camera)->camDistance       = DEFAULT_CAM_DISTANCE____________;
	((GLTrackingCamera *)camera)->camTarget         = DEFAULT_CAM_TARGET______________;
}

void PlushApplication::printCamera() {
	cout << "rotAboutRightAxis: " << ((GLTrackingCamera *)camera)->rotAboutRightAxis     << endl;
	cout << "rotAboutUpAxis:    " << ((GLTrackingCamera *)camera)->rotAboutUpAxis        << endl;
	cout << "camDistance:       " << ((GLTrackingCamera *)camera)->camDistance           << endl;
	cout << "camTarget:         " << ((GLTrackingCamera *)camera)->camTarget.transpose() << endl;
	cout << endl; 
}

bool PlushApplication::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	for (auto handler : handlers) { if (handler->mouse_button(button, action, mods, xPos, yPos)) { return true; } }
	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) { return true; }
	return false;
}

bool PlushApplication::onMouseMoveEvent(double xPos, double yPos) {
	for (auto handler : handlers) { if (handler->mouse_move(xPos, yPos)) { return true; } } 
	if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) { return true; }
	if (SPOOF_2D_CAMERA) { 
		((GLTrackingCamera *)camera)->rotAboutRightAxis = 0.;
		((GLTrackingCamera *)camera)->rotAboutUpAxis    = 0.; 
	}
	return false;
}

bool PlushApplication::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	for (auto handler : handlers) { if (handler->mouse_wheel(xOffset, yOffset)) { return true; }; } 
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) { return true; };
	return false;
}

bool PlushApplication::onKeyEvent(int key, int action, int mods) { 
	for (auto handler : handlers) { handler->key_event(key, action, mods); } // FORNOW
	if (GLApplication::onKeyEvent(key, action, mods)) { return true; } 
	return false;
}
 
bool PlushApplication::onCharacterPressedEvent(int key, int mods) {
	if (key == 'c') {
		resetCamera();
		return true;
	} else if (key == 'C') {
		printCamera();
	}
	// -- //
	if (GLApplication::onCharacterPressedEvent(key, mods)) { return true; } 
	return false;
}
