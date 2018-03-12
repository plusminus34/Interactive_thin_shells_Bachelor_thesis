#include "AppArt.h"
#include <PlushHelpers\helpers_star.h>
 
AppArt::AppArt() {
	setWindowTitle("AppArt");

	pixels = (float *) malloc(sizeof(float)*W()*H()); 

	mesh = new CSTSimulationMesh2D();
	mesh->spawnSavedMesh("../Apps/Plush/data/tri/T");
	// mesh->nudge_mesh_up();
	mesh->addGravityForces(V3D(0., -10., 0.));
	mesh->applyYoungsModulusAndPoissonsRatio(1e4, .25);
	// mesh->add_contacts_to_boundary_nodes(); 

	mainMenu->addGroup("app");
	menuScreen->performLayout(); 

}





void AppArt::drawScene() {
	PlushApplication::drawScene();



	glMasterPush(); {
		glEnable(GL_STENCIL_TEST);
		glStencilFunc(GL_ALWAYS, 1, 1);
		glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
		glColorMask(0, 0, 0, 0);
		mesh->draw();
		glReadPixels(0, 0, W(), H(), GL_STENCIL_INDEX, GL_FLOAT, pixels);
	} glMasterPop();

	Eigen::MatrixXf M = Eigen::Map<Eigen::MatrixXf>(pixels, W(), H()); 

	Eigen::MatrixXf K; K.setOnes(7, 7);
	M = conv2(M, K);
	M = conv2(M, K);

	glWindowPos2i(0, 0); 
	glDrawPixels(W(), H(), GL_RED, GL_FLOAT, M.data());

	glMasterPush(); {
		glDisable(GL_DEPTH_TEST);
		mesh->draw();
	} glMasterPop();

}

void AppArt::process() {
}

bool AppArt::onMouseMoveEvent(double xPos, double yPos) {
    return(PlushApplication::onMouseMoveEvent(xPos, yPos));
}

bool AppArt::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
    return(PlushApplication::onMouseButtonEvent(button, action, mods, xPos, yPos));
}

bool AppArt::onMouseWheelScrollEvent(double xOffset, double yOffset) {
    return(PlushApplication::onMouseWheelScrollEvent(xOffset, yOffset));
}

bool AppArt::onKeyEvent(int key, int action, int mods) {	
	return PlushApplication::onKeyEvent(key, action, mods);
}

bool AppArt::onCharacterPressedEvent(int key, int mods) {
	return PlushApplication::onCharacterPressedEvent(key, mods);
}

bool AppArt::processCommandLine(const std::string& cmdLine) {
	return PlushApplication::processCommandLine(cmdLine);
}

void AppArt::loadFile(const char* fName) { } 
void AppArt::saveFile(const char* fName) { }

