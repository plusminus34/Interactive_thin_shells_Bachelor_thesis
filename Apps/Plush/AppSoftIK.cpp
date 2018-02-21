#include "AppSoftIK.h"
// --
#include "P2DDragger.h"
#include "Poser.h"

AppSoftIK::AppSoftIK() {
    setWindowTitle("AppSoftIK");
	this->showReflections = false;
	this->showGroundPlane = false;

	// string TEST_CASE = "ik";
	string TEST_CASE = "3ball";

	// -- // mesh
	mesh = new CSTSimulationMesh2D();
	char fName[128]; strcpy(fName, "../Apps/Plush/data/tri/"); strcat(fName, TEST_CASE.data()); 
	mesh->spawnSavedMesh(fName);
	mesh->nudge_mesh_up();
	mesh->applyYoungsModulusAndPoissonsRatio(3e4, .25);
	mesh->addGravityForces(V3D(0., -10.));
	// mesh->add_contacts_to_boundary_nodes();

	if (TEST_CASE == "ik") {
		mesh->pinToFloor();
		mesh->rig_boundary_simplices();
		// --
		mesh->relax_tendons();
		mesh->xvPair_INTO_Mesh(mesh->solve_statics());
	} else if (TEST_CASE == "3ball") {
		mesh->add_contacts_to_boundary_nodes();
	}

	// -- // ik
	ik = new SoftIKSolver(mesh);
	push_back_handler(new P2DDragger(&ik->COMp));
	push_back_handler(new Poser(mesh, ik));
	// -- 
	ik->PROJECT = true;
	ik->REGULARIZE_alphac = true; ik->c_alphac_ = .01;
	ik->REGULARIZE_honey = false;
 
	// -- // inspector
	// inspector = new Inspector(mesh);
	// push_back_handler(inspector);
 
	mainMenu->addGroup("app");
	mainMenu->addVariable("SOLVE_IK", SOLVE_IK);
	mainMenu->addVariable("SOLVE_DYNAMICS", SOLVE_DYNAMICS);
	mainMenu->addVariable("UNILATERAL_TENDONS", mesh->UNILATERAL_TENDONS);
	mainMenu->addGroup("ik");
	mainMenu->addVariable("SPEC_COM", ik->SPEC_COM);
	mainMenu->addVariable("SPEC_FREESTYLE", ik->SPEC_FREESTYLE);
	mainMenu->addVariable("NUM_ITERS_PER_STEP", ik->NUM_ITERS_PER_STEP);
	mainMenu->addVariable("LINEAR_APPROX", ik->LINEAR_APPROX);
	mainMenu->addVariable("PROJECT", ik->PROJECT);
	mainMenu->addVariable("HIGH_PRECISION_NEWTON", mesh->HIGH_PRECISION_NEWTON);
	mainMenu->addVariable("c_alphac_", ik->c_alphac_);
	menuScreen->performLayout(); 
}

void AppSoftIK::processToggles() {
	PlushApplication::processToggles();
}

void AppSoftIK::drawScene() {
	DRAW_HANDLERS = false;
	PlushApplication::drawScene(); 
	draw_floor2d();
	mesh->draw();
	ik->draw(); 
	PlushApplication::recordVideo();
}

void AppSoftIK::process() { 
		ik->SOLVE_DYNAMICS = SOLVE_DYNAMICS;
		ik->timeStep = timeStep;
		// --
		ik->x_0 = mesh->x; ik->v_0 = mesh->v;
		ik->step();
		mesh->xvPair_INTO_Mesh((SOLVE_DYNAMICS) ? mesh->solve_dynamics(ik->timeStep, ik->x_0, ik->v_0, ik->alphac_curr) : mesh->solve_statics(ik->x_0, ik->alphac_curr));
}

////////////////////////////////////////////////////////////////////////////////
// -- // handlers //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool AppSoftIK::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (PlushApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) { return true; }
    return false;
}

bool AppSoftIK::onMouseMoveEvent(double xPos, double yPos) {
	if (PlushApplication::onMouseMoveEvent(xPos, yPos) == true) { return true; }
    return false;
}
 
bool AppSoftIK::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (PlushApplication::onMouseWheelScrollEvent(xOffset, yOffset)) { return true; };
    return false;
}

bool AppSoftIK::onKeyEvent(int key, int action, int mods) {
	if (PlushApplication::onKeyEvent(key, action, mods)) { return true; } 
    return false;
}

bool AppSoftIK::onCharacterPressedEvent(int key, int mods) {
	if (PlushApplication::onCharacterPressedEvent(key, mods)) { return true; } 
    return false;
} 
