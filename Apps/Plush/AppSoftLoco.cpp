#include "AppSoftLoco.h"

using namespace nanogui;

AppSoftLoco::AppSoftLoco() {
    setWindowTitle("AppSoftLoco");
	this->showReflections = false;
	this->showGroundPlane = false;

	const vector<string> TEST_CASES = {
		"swingup", // 0
		"tri",     // 1
		"tentacle" // 2
	};
	string TEST_CASE = TEST_CASES[1];

	// -- // mesh
	char fName[128]; strcpy(fName, "../Apps/Plush/data/tri/"); strcat(fName, TEST_CASE.data());
	for (auto ptr : { &mesh, &Zmesh }) {
		*ptr = new CSTSimulationMesh2D();
		(*ptr)->spawnSavedMesh(fName);
		// (*ptr)->nudge_mesh_up();
		(*ptr)->applyYoungsModulusAndPoissonsRatio(3e4, .25);
		(*ptr)->addGravityForces(V3D(0., -10.)); 
		// (*ptr)->pinToFloor(); 
		// (*ptr)->pinToLeftWall(); 
		// (*ptr)->xvPair_INTO_Mesh((*ptr)->solve_statics());
		// (*ptr)->rig_boundary_simplices();
	}

	// -- // ik
	ik = new SoftLocoSolver(mesh);
	{
		for (int i = 0; i < ik->K; ++i) {
			// if (i != 0) { continue; } // !!!
			if (i != ik->K - 1) { continue; }
			auto &COMp = ik->COMpJ[i];
			auto COM_handler = new P2DDragger(&COMp);
			COM_handlers.push_back(COM_handler);
			push_back_handler(COM_handler);
		}
	}
	INTEGRATE_FORWARD_IN_TIME = false;

	ik->PROJECT = true;
	ik->REGULARIZE_u = false;
	ik->LINEAR_APPROX = false;
	ik->c_u_ = .1;
	ik->NUM_ITERS_PER_STEP = 1;
	{
		// Zik = new SoftIKSolver(Zmesh);
		// Zik->PROJECT = true;
		// Zik->REGULARIZE_alphac = false;
		// Zik->LINEAR_APPROX = false;
		// Zik->c_alphac_ = .1;
		// Zik->HONEY_alphac = false;
		// Zik->NUM_ITERS_PER_STEP = 1;
	}
 
	mainMenu->addGroup("app");
	mainMenu->addVariable("SOLVE_IK", SOLVE_IK);
	mainMenu->addVariable("SOLVE_DYNAMICS", ik->SOLVE_DYNAMICS);
	mainMenu->addVariable("UNILATERAL_TENDONS", mesh->UNILATERAL_TENDONS);
	// mainMenu->addVariable("timeStep", timeStep); // TODO
	mainMenu->addGroup("ik");
	// mainMenu->addVariable("SPEC_COM", ik->SPEC_COM);
	// mainMenu->addVariable("SPEC_FREESTYLE", ik->SPEC_FREESTYLE);
	// mainMenu->addVariable("NUM_ITERS_PER_STEP", ik->NUM_ITERS_PER_STEP);
	mainMenu->addVariable("PROJECT", ik->PROJECT);
	mainMenu->addVariable("c_alphac_", ik->c_u_);
	mainMenu->addGroup("testing"); 
	mainMenu->addVariable("INTEGRATE_FORWARD_IN_TIME", INTEGRATE_FORWARD_IN_TIME);
	mainMenu->addVariable("HIGH_PRECISION_NEWTON", mesh->HIGH_PRECISION_NEWTON);
	mainMenu->addVariable("LINEAR_APPROX", ik->LINEAR_APPROX);
	mainMenu->addGroup("loco");
	mainMenu->addVariable("SELECTED_FRAME_i", ik->SELECTED_FRAME_i);
	mainMenu->addGroup("z");
	mainMenu->addVariable("appIsRunnig", appIsRunning);
	mainMenu->addVariable("STEP", STEP);
	menuScreen->performLayout(); 
	appIsRunning = false;
}

void AppSoftLoco::processToggles() {
	PlushApplication::processToggles();
}

void AppSoftLoco::drawScene() {
	{
		// for (size_t i = 0; i < COM_handlers.size(); ++i) {
		// 	COM_handlers[i]->ACTIVE = (i == ik->SELECTED_FRAME_i);
		// }
	}

	DRAW_HANDLERS = false;
	PlushApplication::drawScene(); 
	draw_floor2d();

	mesh->draw(); // FORNOW

	// glMasterPush(); {
	// 	glTranslated(1., 0., 0.);
	// 	glLineWidth(9);
	// 	set_color(GOLDCLOVER);
	// 	glBegin(GL_LINE_STRIP); {
	// 		for (auto &bs : Zmesh->boundary_simplices) {
	// 			for (auto &node : bs->nodes) {
	// 				glP3D(node->getCoordinates(Zik->x_curr));
	// 			}
	// 		}
	// 	} glEnd();
	// } glMasterPop();

	ik->draw(); 

	PlushApplication::recordVideo();
}

void AppSoftLoco::process() { 
	// ik->COMp_FORNOW = ik->COMpJ[0]; // !!!
	// Zik->COMp       = ik->COMpJ[0]; // !!!
	ik->COMp_FORNOW = ik->COMpJ[ik->K - 1]; // !!!
	// Zik->COMp       = ik->COMpJ[ik->K - 1]; // !!!
	// --
	// Zik->SOLVE_DYNAMICS = ik->SOLVE_DYNAMICS;
	// -- 
	if (INTEGRATE_FORWARD_IN_TIME) { ik->xm1_curr = mesh->x; ik->vm1_curr = mesh->v; } 
	// if (INTEGRATE_FORWARD_IN_TIME) { Zik->x_0 = mesh->x; Zik->v_0 = mesh->v; } // FORNOW
	if (SOLVE_IK) {
		cout << endl << "--> loco" << endl;
		ik->step();
		// cout << endl << "--> Z_ik" << endl;
		// Zik->step();
		// getchar();
	}
	if (INTEGRATE_FORWARD_IN_TIME) { mesh->xvPair_INTO_Mesh((ik->SOLVE_DYNAMICS) ? mesh->solve_dynamics(ik->xm1_curr, ik->vm1_curr, ik->uJ_curr[0]) : mesh->solve_statics(ik->xm1_curr, ik->uJ_curr[0])); }
	// if (INTEGRATE_FORWARD_IN_TIME) { mesh->xvPair_INTO_Mesh((Zik->SOLVE_DYNAMICS) ? mesh->solve_dynamics(Zik->x_0, Zik->v_0, Zik->alphac_curr) : mesh->solve_statics(Zik->x_0, Zik->alphac_curr)); } // FORNOW
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
	if (key == 'l') {
		if (ik->SELECTED_FRAME_i == ik->K - 1) {
			ik->SELECTED_FRAME_i = 0;
		} else {
			ik->SELECTED_FRAME_i += 1;
		}
	} else if (key == 'h') {
		if (ik->SELECTED_FRAME_i == 0) {
			ik->SELECTED_FRAME_i = ik->K - 1;
		} else {
			ik->SELECTED_FRAME_i -= 1;
		}
	}
	// --
	if (PlushApplication::onCharacterPressedEvent(key, mods)) { return true; } 
    return false;
} 
/*
		if (TEST_CASE == "swingup") {
			(*ptr)->pinToLeftWall();
			(*ptr)->relax_tendons();
			(*ptr)->timeStep = .01;
		} else if (TEST_CASE == "tri") {
*/
