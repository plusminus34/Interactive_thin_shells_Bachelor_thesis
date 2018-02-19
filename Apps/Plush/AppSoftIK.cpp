#include "AppSoftIK.h"
// --
#include "P2DDragger.h"
#include "Poser.h"

AppSoftIK::AppSoftIK() {
    setWindowTitle("AppSoftIK");
	this->showReflections = false;
	this->showGroundPlane = false;

	string TEST_CASE = "ik"; 

	// -- // mesh
	mesh = new CSTSimulationMesh2D();
	char fName[128]; strcpy(fName, "../Apps/Plush/data/tri/"); strcat(fName, TEST_CASE.data()); 
	mesh->spawnSavedMesh(fName);
	mesh->nudge_mesh_up();
	mesh->applyYoungsModulusAndPoissonsRatio(3e4, .25);
	mesh->addGravityForces(V3D(0., -10.));
	// mesh->add_contacts_to_boundary_nodes();
	mesh->pinToFloor(); 
	mesh->rig_boundary_simplices();
	// --
	mesh->relax_tendons(); mesh->xvPair_INTO_Mesh(mesh->solve_statics());

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
 
	/*
	TwAddVarRW(mainMenuBar, "SOLVE_IK", TW_TYPE_BOOLCPP, &SOLVE_IK, "key=I");
	TwAddVarRW(mainMenuBar, "SOLVE_DYNAMICS", TW_TYPE_BOOLCPP, &SOLVE_DYNAMICS, "key=S");
	TwAddSeparator(mainMenuBar, "ik:general", "");
	TwAddVarRW(mainMenuBar, "mode", TW_TYPE_INT32, &ik->mode, "key=TAB min=0 max=1");
	TwAddVarRW(mainMenuBar, "NUM_ITERS_PER_STEP"FREESTYLE TW_TYPE_INT32, &ik->NUM_ITERS_PER_STEP, "");
	TwAddVarRW(mainMenuBar, "LINEAR_APPROX", TW_TYPE_BOOLCPP, &ik->LINEAR_APPROX, "");
	TwAddVarRW(mainMenuBar, "PROJECT", TW_TYPE_BOOLCPP, &ik->PROJECT, "");
	TwAddVarRW(mainMenuBar, "UNILATERAL_TENDONS", TW_TYPE_BOOLCPP, &mesh->UNILATERAL_TENDONS, "");
	TwAddVarRW(mainMenuBar, "HIGH_PRECISION_NEWTON", TW_TYPE_BOOLCPP, &mesh->HIGH_PRECISION_NEWTON, "");
	TwAddSeparator(mainMenuBar, "ik:reg", "");
	TwAddVarRW(mainMenuBar, "?_alphac_   ", TW_TYPE_BOOLCPP, &ik->REGULARIZE_alphac, "");
	TwAddVarRW(mainMenuBar, "c_alphac_   ", TW_TYPE_DOUBLE,  &ik->c_alphac_, "");
	*/
	mainMenu->addGroup("AppSoftIK");
	mainMenu->addVariable("SOLVE_DYNAMICS", SOLVE_DYNAMICS);
	mainMenu->addVariable("SPEC_COM", ik->SPEC_COM);
	mainMenu->addVariable("SPEC_FREESTYLE", ik->SPEC_FREESTYLE);
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
