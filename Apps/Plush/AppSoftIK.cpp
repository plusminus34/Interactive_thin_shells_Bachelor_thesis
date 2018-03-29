#include "AppSoftIK.h"
// --
#include "P2DDragger.h"
#include "Poser.h"

AppSoftIK::AppSoftIK() {
    setWindowTitle("AppSoftIK");
	this->showReflections = false;
	this->showGroundPlane = false;

	const vector<string> TEST_CASES = {
		"tentacle", // 0
		"3ball",    // 1
		"tri",      // 2
		"swingup",  // 3
		"luca",     // 4
		"sugar"     // 5
	};
	string TEST_CASE = TEST_CASES[4];

	// BEG: Convenience block
	SPOOF_2D_CAMERA = true;
	// --
	if (TEST_CASE == "luca") {
		DEFAULT_CAM_DISTANCE____________ = -13.;
		DEFAULT_CAM_TARGET______________ = P3D(-.15, .85);
		resetCamera();
	}
	// END

	// -- // mesh
	mesh = new CSTSimulationMesh2D();
	char fName[128]; strcpy(fName, "../Apps/Plush/data/tri/"); strcat(fName, TEST_CASE.data()); 
	mesh->spawnSavedMesh(fName);
	mesh->nudge_mesh_up();
	mesh->applyYoungsModulusAndPoissonsRatio(3e4, .25);
	mesh->addGravityForces(V3D(0., -10.));
	// mesh->add_contacts_to_boundary_nodes();

	if (TEST_CASE == "tentacle") {
		mesh->pinToFloor();
		mesh->rig_boundary_simplices();
		// --
		mesh->relax_tendons();
		mesh->xvPair_INTO_Mesh(mesh->solve_statics());
	} else if (TEST_CASE == "3ball") {
		mesh->add_contacts_to_boundary_nodes();
	} else if (TEST_CASE == "tri") {
		mesh->pinToFloor();
		mesh->rig_boundary_simplices();
		INTEGRATE_FORWARD_IN_TIME = false;
	} else if (TEST_CASE == "swingup") {
		mesh->pinToLeftWall();
		mesh->relax_tendons();
		mesh->xvPair_INTO_Mesh(mesh->solve_statics());
		mesh->timeStep = .1;
	} else if (TEST_CASE == "luca") {
		// mesh->add_contacts_to_boundary_nodes();
		// mesh->nudge_mesh_up();
		mesh->pinToFloor();
		mesh->timeStep = .01; 
	} else if (TEST_CASE == "sugar") {
		mesh->pinToFloor();
		mesh->timeStep = .01; 
	}


	// -- // ik
	ik = new SoftIKSolver(mesh);
	push_back_handler(new P2DDragger(&ik->COMp));
	push_back_handler(new Poser(mesh, ik));
	// -- 
	ik->PROJECT           = true;
	ik->LINEAR_APPROX     = true;
	ik->REGULARIZE_alphac = true;
	ik->HONEY_alphac      = true;
	ik->NUM_ITERS_PER_STEP = 1;

	if (TEST_CASE == "tentacle") { 
		ik->c_alphac_ = 1;
		ik->h_alphac_ = 1000;
	} else if (TEST_CASE == "3ball") {
		ik->c_alphac_ = .5;
		ik->h_alphac_ = 10.; 
	} else if (TEST_CASE == "tri") {
		ik->REGULARIZE_alphac = false;
		ik->HONEY_alphac = false;
	} else if (TEST_CASE == "swingup") {
		ik->c_alphac_ = 1.;
		ik->h_alphac_ = 0.; 
	} else if (TEST_CASE == "luca") {
		ik->c_alphac_ = 1.;
		ik->h_alphac_ = 100.; 
	} else if (TEST_CASE == "sugar") {
		ik->c_alphac_ = 1.;
		ik->h_alphac_ = 0.; 
	}
 
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
	mainMenu->addVariable("PROJECT", ik->PROJECT);
	mainMenu->addVariable("c_alphac_", ik->c_alphac_);
	mainMenu->addVariable("h_alphac_", ik->h_alphac_);
	mainMenu->addGroup("testing"); 
	mainMenu->addVariable("INTEGRATE_FORWARD_IN_TIME", INTEGRATE_FORWARD_IN_TIME);
	mainMenu->addVariable("CHECK_IK_GRADIENT", ik->CHECK_IK_GRADIENT);
	mainMenu->addVariable("HIGH_PRECISION_NEWTON", mesh->HIGH_PRECISION_NEWTON);
	mainMenu->addVariable("LINEAR_APPROX", ik->LINEAR_APPROX);
	menuScreen->performLayout(); 
}

void AppSoftIK::processToggles() {
	PlushApplication::processToggles();
}

void AppSoftIK::drawScene() {
	DRAW_HANDLERS = false;
	PlushApplication::drawScene(); 
	// draw_floor2d();
	mesh->draw(mesh->x, ik->alphac_curr);

	// glMasterPush(); {
	// 	glTranslated(2.5, 0., 0.);
	// 	dVector ZERO_; ZERO_.setZero(ik->T());
	// 	mesh->draw(mesh->X, ZERO_); 
	// } glMasterPop();

	ik->draw(); 

	PlushApplication::recordVideo();
}

void AppSoftIK::process() { 
	ik->SOLVE_DYNAMICS = SOLVE_DYNAMICS;
	ik->timeStep = timeStep;
	// --
	if (INTEGRATE_FORWARD_IN_TIME) { ik->x_0 = mesh->x; ik->v_0 = mesh->v; }
	if (SOLVE_IK) { ik->step(); }
	if (INTEGRATE_FORWARD_IN_TIME) { mesh->xvPair_INTO_Mesh((SOLVE_DYNAMICS) ? mesh->solve_dynamics(ik->x_0, ik->v_0, ik->alphac_curr) : mesh->solve_statics(ik->x_0, ik->alphac_curr)); }

	// ik->x_0 = mesh->x;
	// ik->step();
	// auto xv = mesh->solve_statics(mesh->x, ik->alphac_curr);
	// mesh->xvPair_INTO_Mesh(xv);
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
