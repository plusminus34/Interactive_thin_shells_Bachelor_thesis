#include "AppSoftLoco.h"
// --
#include "P2DDragger.h"
#include "Poser.h"

AppSoftLoco::AppSoftLoco() {
    setWindowTitle("AppSoftLoco");
	this->showReflections = false;
	this->showGroundPlane = false;

	const vector<string> TEST_CASES = {
		"swingup", // 0
	};
	string TEST_CASE = TEST_CASES[0];

	// -- // mesh
	mesh = new CSTSimulationMesh2D();
	char fName[128]; strcpy(fName, "../Apps/Plush/data/tri/"); strcat(fName, TEST_CASE.data()); 
	mesh->spawnSavedMesh(fName);
	mesh->nudge_mesh_up();
	mesh->applyYoungsModulusAndPoissonsRatio(3e4, .25);
	mesh->addGravityForces(V3D(0., -10.));
	// mesh->add_contacts_to_boundary_nodes();

	if (TEST_CASE == "swingup") {
		mesh->pinToLeftWall();
		mesh->relax_tendons();
		// mesh->xvPair_INTO_Mesh(mesh->solve_statics());
		timeStep = .1;
	}

	// -- // ik
	ik = new SoftLocoSolver(mesh);
	// push_back_handler(new P2DDragger(&ik->COMp));
	// push_back_handler(new Poser(mesh, ik));
	// -- 
	INTEGRATE_FORWARD_IN_TIME = false;
	ik->PROJECT           = true;
	ik->LINEAR_APPROX     = true;
	ik->REGULARIZE_alphac = true;

	if (TEST_CASE == "swingup") { 
		ik->c_alphac_ = 1.;
	}
 
	mainMenu->addGroup("app");
	mainMenu->addVariable("SOLVE_IK", SOLVE_IK);
	mainMenu->addVariable("SOLVE_DYNAMICS", SOLVE_DYNAMICS);
	mainMenu->addVariable("UNILATERAL_TENDONS", mesh->UNILATERAL_TENDONS);
	mainMenu->addVariable("timeStep", timeStep);
	mainMenu->addGroup("ik");
	// mainMenu->addVariable("SPEC_COM", ik->SPEC_COM);
	// mainMenu->addVariable("SPEC_FREESTYLE", ik->SPEC_FREESTYLE);
	// mainMenu->addVariable("NUM_ITERS_PER_STEP", ik->NUM_ITERS_PER_STEP);
	mainMenu->addVariable("PROJECT", ik->PROJECT);
	mainMenu->addVariable("c_alphac_", ik->c_alphac_);
	mainMenu->addGroup("testing"); 
	mainMenu->addVariable("INTEGRATE_FORWARD_IN_TIME", INTEGRATE_FORWARD_IN_TIME);
	mainMenu->addVariable("HIGH_PRECISION_NEWTON", mesh->HIGH_PRECISION_NEWTON);
	mainMenu->addVariable("LINEAR_APPROX", ik->LINEAR_APPROX);
	menuScreen->performLayout(); 
}

void AppSoftLoco::processToggles() {
	PlushApplication::processToggles();
}

void AppSoftLoco::drawScene() {
	DRAW_HANDLERS = false;
	PlushApplication::drawScene(); 
	draw_floor2d();
	// mesh->draw(); // FORNOW
	ik->draw(); 
	PlushApplication::recordVideo();
}

void AppSoftLoco::process() { 
		ik->SOLVE_DYNAMICS = SOLVE_DYNAMICS;
		ik->timeStep = timeStep;
		// --
		ik->x_0 = mesh->x; ik->v_0 = mesh->v;
		ik->step();
		if (INTEGRATE_FORWARD_IN_TIME) { mesh->xvPair_INTO_Mesh((SOLVE_DYNAMICS) ? mesh->solve_dynamics(ik->timeStep, ik->x_0, ik->v_0, ik->alphacJ_curr[0]) : mesh->solve_statics(ik->x_0, ik->alphacJ_curr[0])); }
}

////////////////////////////////////////////////////////////////////////////////
// -- // handlers //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool AppSoftLoco::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (PlushApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) { return true; }
    return false;
}

bool AppSoftLoco::onMouseMoveEvent(double xPos, double yPos) {
	if (PlushApplication::onMouseMoveEvent(xPos, yPos) == true) { return true; }
    return false;
}
 
bool AppSoftLoco::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (PlushApplication::onMouseWheelScrollEvent(xOffset, yOffset)) { return true; };
    return false;
}

bool AppSoftLoco::onKeyEvent(int key, int action, int mods) {
	if (PlushApplication::onKeyEvent(key, action, mods)) { return true; } 
    return false;
}

bool AppSoftLoco::onCharacterPressedEvent(int key, int mods) {
	if (PlushApplication::onCharacterPressedEvent(key, mods)) { return true; } 
    return false;
} 
