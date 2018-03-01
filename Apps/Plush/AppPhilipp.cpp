#include "AppPhilipp.h"
#include <PlushHelpers\helpers_star.h>

AppPhilipp::AppPhilipp() {
	setWindowTitle("AppPhilipp");

	V3D tx_ = V3D(.75, 0., 0.);

	plot = new QueuePlot(50, 1);

	mesh = new CSTSimulationMesh2D();
	mesh->spawnSavedMesh("../Apps/Plush/data/tri/philipp");
	mesh->nudge_mesh_up();
	mesh->addGravityForces(V3D(0., -10., 0.));
	mesh->applyYoungsModulusAndPoissonsRatio(1e4, .25);
	mesh->add_contacts_to_boundary_nodes(); 

	mainMenu->addGroup("app");
	mainMenu->addVariable("Draw F2D", mesh->DRAW_NODAL_FORCES);
	menuScreen->performLayout(); 

}

void AppPhilipp::drawScene() {
	PlushApplication::drawScene();
	mesh->draw();
	plot->draw();
	draw_floor2d();
}

void AppPhilipp::process() {
	double sin01 = .5*(1. + sin(10.*t));
	double frac = .5 * sin01; // nornmalized contraction
	plot->add_new_data_point(frac);

	mesh->tendons[0]->set_alphac_over_alphaz(frac);

	mesh->xvPair_INTO_Mesh(mesh->solve_dynamics());
}

bool AppPhilipp::onMouseMoveEvent(double xPos, double yPos) {
    return(PlushApplication::onMouseMoveEvent(xPos, yPos));
}

bool AppPhilipp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
    return(PlushApplication::onMouseButtonEvent(button, action, mods, xPos, yPos));
}

bool AppPhilipp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
    return(PlushApplication::onMouseWheelScrollEvent(xOffset, yOffset));
}

bool AppPhilipp::onKeyEvent(int key, int action, int mods) {	
	return PlushApplication::onKeyEvent(key, action, mods);
}

bool AppPhilipp::onCharacterPressedEvent(int key, int mods) {
	return PlushApplication::onCharacterPressedEvent(key, mods);
}

bool AppPhilipp::processCommandLine(const std::string& cmdLine) {
	return PlushApplication::processCommandLine(cmdLine);
}

void AppPhilipp::loadFile(const char* fName) { } 
void AppPhilipp::saveFile(const char* fName) { }

