#include <GUILib/GLUtils.h>
#include "FireworksApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>
#include <GUILib/GLTrackingCamera.h>

FireworksApp::FireworksApp() {
	setWindowTitle("Fireworks...");
	delete camera;
	camera = new GLTrackingCamera(-75);
	camera->setCameraTarget(P3D(0, 0, 0));
	showGroundPlane = false;
	showConsole = false;
    //bgColor[0] = bgColor[1] = bgColor[2] = 0;
}

FireworksApp::~FireworksApp(void){

}

//triggered when mouse moves
bool FireworksApp::onMouseMoveEvent(double xPos, double yPos) {
	if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;

	return false;
}

//triggered when mouse buttons are pressed
bool FireworksApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	return false;
}

//triggered when using the mouse wheel
bool FireworksApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool FireworksApp::onKeyEvent(int key, int action, int mods) {
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool FireworksApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

	return false;
}


void FireworksApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);
}

void FireworksApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}

bool firstTime = true;


// Run the App tasks
void FireworksApp::process() {
	//do the work here...

	double initialSpeed = (10 + getRandomNumberInRange(0, 20));

	if (firstTime) {
		int nNewParticles = 100;
		for (int i = 0; i < nNewParticles; i++) {
			Particle newParticle;
			newParticle.pos = P3D(0, 1, 0);
			newParticle.vel = V3D(getRandomNumberInRange(-1, 1), getRandomNumberInRange(-1, 1), getRandomNumberInRange(-1, 1)).unit() * initialSpeed;
			newParticle.isSeedParticle = true;
			particles.push_back(newParticle);
		}
	}

	for (uint i = 0; i < particles.size(); i++){
		if (particles[i].isSeedParticle) {
			int nNewParticles = rand() % 2;
			for (int j = 0; j < nNewParticles; j++) {
				Particle newParticle;
				newParticle.pos = particles[i].pos;
				newParticle.vel = particles[i].vel + V3D(getRandomNumberInRange(-1, 1), 0.5 + getRandomNumberInRange(-0.5, 0.5), getRandomNumberInRange(-1, 1)).unit() * particles[i].vel.length() * 0.1;
				newParticle.isSeedParticle = false;
				newParticle.dragCoefficient = 10;
				newParticle.intensity = MIN(particles[i].intensity * getRandomNumberInRange(1.0, 2.0), 1.0);
				newParticle.size = 4.0;
				particles.push_back(newParticle);
			}
		}
	}

	//spawn off little particles from each of the seed particles
	firstTime = false;

//advance the particle system forward in time
	double simulationTime = 0;
	double maxRunningTime = 1.0 / desiredFrameRate;
	double simTimeStep = 0.01;

	//if we still have time during this frame, or if we need to finish the physics step, do this until the simulation time reaches the desired value
	while (simulationTime / maxRunningTime < animationSpeedupFactor) {
		simulationTime += simTimeStep;

		for (uint i = 0; i < particles.size(); i++)
			particles[i].advanceInTime(simTimeStep);
	}

	//now remove the particles that have cooled off enough...
	for (int i = (int)particles.size() - 1; i >= 0; i--)
		if (particles[i].intensity < 0.1)
			particles.erase(particles.begin() + i);

	if (particles.size() == 0)
		firstTime = true;
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void FireworksApp::drawScene() {
	//draw here...

	glDisable(GL_TEXTURE_2D);


	for (uint i = 0; i < particles.size(); i++)
		particles[i].draw();
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void FireworksApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void FireworksApp::restart() {

}

bool FireworksApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

