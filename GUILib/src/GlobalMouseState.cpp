#include <GUILib/GlobalMouseState.h>
#include <GUILib/GLIncludes.h>

double GlobalMouseState::lastMouseX = 0, GlobalMouseState::lastMouseY = 0;
double GlobalMouseState::mouseMoveX = 0, GlobalMouseState::mouseMoveY = 0;
bool GlobalMouseState::rButtonPressed = false, GlobalMouseState::lButtonPressed = false, GlobalMouseState::mButtonPressed = false;
bool GlobalMouseState::dragging = false;


void GlobalMouseState::updateMouseState(double xPos, double yPos, int button, int action, int mods) {
	lastMouseX = xPos;
	lastMouseY = yPos;

	if (action == GLFW_PRESS)
		dragging = true;
	else 
		dragging = false;

	if (button == GLFW_MOUSE_BUTTON_LEFT)
		lButtonPressed = action != GLFW_RELEASE;
	if (button == GLFW_MOUSE_BUTTON_MIDDLE)
		mButtonPressed = action != GLFW_RELEASE;
	if (button == GLFW_MOUSE_BUTTON_RIGHT)
		rButtonPressed = action != GLFW_RELEASE;
}

void GlobalMouseState::updateMouseMove(double xPos, double yPos) {
	mouseMoveX = lastMouseX - xPos;
	mouseMoveY = -lastMouseY + yPos;
	lastMouseX = xPos; lastMouseY = yPos;
}

