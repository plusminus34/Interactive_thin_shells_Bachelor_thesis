#include "AppSoftLoco.h"

using namespace nanogui;

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
		mesh->timeStep = .1;
	}

	// -- // ik
	ik = new SoftLocoSolver(mesh);
	{
		for (auto &COMp : ik->COMpJ) {
			auto COM_handler = new P2DDragger(&COMp);
			COM_handlers.push_back(COM_handler);
			push_back_handler(COM_handler);
		}
		// push_back_handler(new Poser(mesh, ik));
	}
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
	mainMenu->addVariable("SOLVE_DYNAMICS", ik->SOLVE_DYNAMICS);
	mainMenu->addVariable("UNILATERAL_TENDONS", mesh->UNILATERAL_TENDONS);
	// mainMenu->addVariable("timeStep", timeStep); // TODO
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
	mainMenu->addGroup("loco");
	auto tmp = mainMenu->addVariable("SELECTED_FRAME_i", ik->SELECTED_FRAME_i);
	menuScreen->performLayout(); 

	{
		using namespace nanogui;
		ref<Window> window = mainMenu->addWindow(Eigen::Vector2i(275, 0), "ikMenu");
		mainMenu->addGroup("Group 1");
		mainMenu->addGroup("Group 2");
		menuScreen->performLayout(); 
		// menuScreen->removeChild(window);
	}

}

void AppSoftLoco::processToggles() {
	PlushApplication::processToggles();
}

void AppSoftLoco::drawScene() {
	{
		for (size_t i = 0; i < COM_handlers.size(); ++i) {
			COM_handlers[i]->ACTIVE = (i == ik->SELECTED_FRAME_i);
		}
	}

	DRAW_HANDLERS = false;
	PlushApplication::drawScene(); 
	draw_floor2d();
	// mesh->draw(); // FORNOW
	ik->draw(); 
	PlushApplication::recordVideo();
}

void AppSoftLoco::process() { 
	if (INTEGRATE_FORWARD_IN_TIME) { ik->x_0 = mesh->x; ik->v_0 = mesh->v; } // FORNOW
	if (SOLVE_IK) { ik->step(); }
	if (INTEGRATE_FORWARD_IN_TIME) { mesh->xvPair_INTO_Mesh((ik->SOLVE_DYNAMICS) ? mesh->solve_dynamics(ik->x_0, ik->v_0, ik->alphacJ_curr[0]) : mesh->solve_statics(ik->x_0, ik->alphacJ_curr[0])); }
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
