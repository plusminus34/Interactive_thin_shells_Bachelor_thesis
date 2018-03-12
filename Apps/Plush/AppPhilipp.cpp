#include "AppPhilipp.h"
#include <PlushHelpers\helpers_star.h>

AppPhilipp::AppPhilipp() {
	setWindowTitle("AppPhilipp");

	V3D tx_ = V3D(.75, 0., 0.);

	for (int i = 0; i < NUM_KEYFRAMES; ++i) {
		u_vec.push_back(sin(dfrac(i, NUM_KEYFRAMES - 1) * 2. * PI));
	}
	t_vec = linspace(NUM_KEYFRAMES, 0., 1.);

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
		// TODO: Learn about Lab space (NOTE: Not relavent)

	PlushApplication::drawScene();
	// mesh->draw();
	// plot->draw();
	// draw_floor2d();

	glMasterPush(); {
		//glTranslated(-1., 1., 0.);
		glLineWidth(8.);
		glPointSize(8.);
		// --
		set_color(CLAY);
		glBegin(GL_LINES); {
			glP3D(P3D(0., -1.));
			glP3D(P3D(0., 1.));
			glP3D(P3D(0., 0.));
			glP3D(P3D(1., 0.)); 
		} glEnd();
		// --

		glLineWidth(4.);
		set_color(PUMPKIN);
		for (auto &GL_PRIMITIVE : { GL_LINE_STRIP, GL_POINTS }) {
			glBegin(GL_PRIMITIVE); {
				for (int i = 0; i < NUM_KEYFRAMES; ++i) {
					glP3D(P3D(t_vec[i], u_vec[i]));
				}
			} glEnd();
			set_color(LIGHT_PUMPKIN);
		}
	} glMasterPop();

	glMasterPush(); {
		glPointSize(10.);
		set_color(HENN1NK);
		glBegin(GL_POINTS); {
			glP3D(xy0);
		} glEnd();
	} glMasterPop();

	for (int i = 0; i < NUM_KEYFRAMES; ++i) {
		P3D test = (P3D(t_vec[i], u_vec[i]));
		if (V3D(xy0, test).squaredNorm() < .05) {
			glMasterPush(); {
				glPointSize(10.);
				set_color(ORCHID);
				glBegin(GL_POINTS); {
					glP3D(test);
				} glEnd();
			} glMasterPop(); 
		}
	}
}

void AppPhilipp::process() {
	double sin01 = .5*(1. + sin(10.*t));
	double frac = .5 * sin01; // nornmalized contraction

	plot->add_new_data_point(frac);

	mesh->tendons[0]->set_alphac_over_alphaz(frac);

	mesh->xvPair_INTO_Mesh(mesh->solve_dynamics());
}

bool AppPhilipp::onMouseMoveEvent(double xPos, double yPos) {
	if (PlushApplication::onMouseMoveEvent(xPos, yPos)) { return true; }
	// -- TODO: 
	Ray ray = InteractiveWidget::getRayFromScreenCoords(xPos, yPos);
	xy0 = xy0_from_ray(ray); 
	return false;
}

bool AppPhilipp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (PlushApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) { return true; }
	// -- TODO:
	return false;
}

bool AppPhilipp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (PlushApplication::onMouseWheelScrollEvent(xOffset, yOffset)) { return true; }
	return false;
}

bool AppPhilipp::onKeyEvent(int key, int action, int mods) {	
	if (PlushApplication::onKeyEvent(key, action, mods)) { return true; }
	return false;
}

bool AppPhilipp::onCharacterPressedEvent(int key, int mods) {
	if (PlushApplication::onCharacterPressedEvent(key, mods)) { return true; }
	return false;
}

bool AppPhilipp::processCommandLine(const std::string& cmdLine) {
	if (PlushApplication::processCommandLine(cmdLine)) { return true; }
	return false;
}

void AppPhilipp::loadFile(const char* fName) { } 
void AppPhilipp::saveFile(const char* fName) { }

