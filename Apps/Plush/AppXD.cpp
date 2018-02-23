#include "AppXD.h"
#include <PlushHelpers\helpers_star.h>

AppXD::AppXD() {
	setWindowTitle("AppXD");

	V3D tx_ = V3D(.75, 0., 0.);

	tri_mesh = new CSTSimulationMesh2D();
	tri_mesh->spawnSavedMesh("../Apps/Plush/data/tri/T");
	tri_mesh->nudge_mesh(-tx_);
	tri_mesh->addGravityForces(V3D(0., -10., 0.));
	tri_mesh->applyYoungsModulusAndPoissonsRatio(1e4, .25);
	tri_mesh->pinToCeiling(); 
	// --
	tet_mesh = new CSTSimulationMesh3D();
	tet_mesh->spawnSavedMesh("../Apps/Plush/data/tet/brick");
	tet_mesh->nudge_mesh(tx_);
	tet_mesh->addGravityForces(V3D(0., -10., 0.));
	tet_mesh->applyYoungsModulusAndPoissonsRatio(1e4, .25);
	tet_mesh->pinToCeiling(); 

	mainMenu->addGroup("app");
	mainMenu->addVariable("Draw F2D", tri_mesh->DRAW_NODAL_FORCES);
	mainMenu->addVariable("Draw F3D", tet_mesh->DRAW_NODAL_FORCES);
	mainMenu->addVariable("Transparent 3D", tet_mesh->DRAW_BOUNDARY_TRANSPARENT);
	menuScreen->performLayout(); 
}

void AppXD::drawScene() {
	PlushApplication::drawScene();
	tri_mesh->draw();
	tet_mesh->draw();
	// draw_floor2d();
}

void AppXD::process() {
	tri_mesh->xvPair_INTO_Mesh(tri_mesh->solve_dynamics(timeStep));
	tet_mesh->xvPair_INTO_Mesh(tet_mesh->solve_dynamics(timeStep));
}

bool AppXD::onMouseMoveEvent(double xPos, double yPos) {
    return(PlushApplication::onMouseMoveEvent(xPos, yPos));
}

bool AppXD::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
    return(PlushApplication::onMouseButtonEvent(button, action, mods, xPos, yPos));
}

bool AppXD::onMouseWheelScrollEvent(double xOffset, double yOffset) {
    return(PlushApplication::onMouseWheelScrollEvent(xOffset, yOffset));
}

bool AppXD::onKeyEvent(int key, int action, int mods) {	
	return PlushApplication::onKeyEvent(key, action, mods);
}

bool AppXD::onCharacterPressedEvent(int key, int mods) {
	return PlushApplication::onCharacterPressedEvent(key, mods);
}

bool AppXD::processCommandLine(const std::string& cmdLine) {
	return PlushApplication::processCommandLine(cmdLine);
}

void AppXD::loadFile(const char* fName) { } 
void AppXD::saveFile(const char* fName) { }

