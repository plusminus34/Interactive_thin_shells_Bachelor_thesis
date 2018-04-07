#include "AppSoftLoco.h"
 
using namespace nanogui;

// TODO: Check gradient (you can clear out the rest of the FD code, add in one big check on dOdyJ --- feel free to leave other checks in for now)
// TODO: You should be able to drag activations yourself
// TODO: Save user-designed control-signals, and use as intial guess.
//       -- // Perturb these control signals to get a sense of the envelope of convergence.
// TODO: Add tangents
// TODO: Scalar to reduce dynamics effects
// TODO: SplinePlot
// TODO: Moritz's idea of optimizing for u_{-1}, essentially "picking the starting pose", right now we have a special case of u_{-1} = 0
// -- Keep in mind that this is guarded by 1000 dynamics solves.
// TODO: Projection
// Bilateral tendons until then

// TODO: Material damping? / Regularizer?
// TODO: Squishier material model?
// TODO: Throttle contraction _rate_?
// TODO: Throttle _reaction forces_?
// TODO: Different smoothing kernel?
// TODO: Aiiiiiir resistance?

AppSoftLoco::AppSoftLoco() {
    setWindowTitle("AppSoftLoco");
	this->showReflections = false;
	this->showGroundPlane = false;
	this->DEFAULT_CAM_TARGET______________ = P3D(-1., 0.);
	this->DEFAULT_CAM_DISTANCE____________ = -4.5;
	this->SPOOF_2D_CAMERA = true;
	this->resetCamera();

	const vector<string> TEST_CASES = {
		"swingup",      // 0
		"tri",          // 1
		"tentacle",     // 2
		"mini_swingup", // 3
		"3ball",        // 4
		"walker",       // 5
		"jumping_cube", // 6
		"6ball",        // 7
		"sugar",        // 8
		"T"             // 9
	};
	string TEST_CASE = TEST_CASES[0];

	// -- // mesh
	char fName[128]; strcpy(fName, "../Apps/Plush/data/tri/"); strcat(fName, TEST_CASE.data());
	mesh = new CSTSimulationMesh2D();
	mesh->timeStep = .01;
	mesh->spawnSavedMesh(fName, true);
	// mesh->rotate_mesh(PI);
	mesh->nudge_mesh_up();
	mesh->applyYoungsModulusAndPoissonsRatio(3e5, .25); // FORNOW
	mesh->addGravityForces(V3D(0., -10.)); 
    // mesh->pinToFloor(); 
	mesh->pinToLeftWall(); 
	// mesh->add_contacts_to_boundary_nodes();
	// mesh->xvPair_INTO_Mesh((*ptr)->solve_statics());
	if (TEST_CASE == "tri") { mesh->rig_boundary_simplices(); }
	// mesh->rig_boundary_simplices();
	// mesh->rig_all_lower_simplices();

	for (size_t i = 0; i < mesh->tendons.size(); ++i) {
		mesh->tendons[i]->SPEC_COLOR = kelly_color(i);
	}

	// FORNOW
	// for (int _ = 0; _ < 1000; ++_) { mesh->xvPair_INTO_Mesh(mesh->solve_dynamics()); }

	// -- // ik
	ik = new SoftLocoSolver(mesh);
	{
		for (int i = 0; i < ik->K; ++i) {
			// if (i != 0) { continue; } // !!!
			if (i != ik->K - 1 && i != ik->K/2) { continue; }
			auto &COMp = ik->COMpJ[i];
			auto COM_handler = new P2DDragger(&COMp);
			COM_handlers.push_back(COM_handler);
			push_back_handler(COM_handler);
		}
	}
	INTEGRATE_FORWARD_IN_TIME = false;
	mesh->UNILATERAL_TENDONS = false;

	ik->PROJECT = false;
	ik->LINEAR_APPROX = false;
	// ik->r_u_ = .1;
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

	{
		ik->REGULARIZE_u = false;
		ik->SUBSEQUENT_u = false;
		mesh->HIGH_PRECISION_NEWTON = false;
		ik->COMpJ.back() += V3D(-.2, .4); 
		appIsRunning = false;
	}


	// FORNOW
	{
		vector<P3D *> all_points;
		for (int i = 0; i < ik->T(); ++i) {
			vector<P3D *> test_spline;
			for (int z = 0; z < ik->Z; ++z) {
				P3D *point = new P3D(dfrac(z, ik->Z - 1), 0.);
				test_spline.push_back(point);
				all_points.push_back(point);
			}
			test_splines.push_back(test_spline);
		}
		push_back_handler2(new P2DDragger_v2(all_points, &test_frame, true, -1., .5));
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
	mainMenu->addGroup("testing"); 
	mainMenu->addVariable("INTEGRATE_FORWARD_IN_TIME", INTEGRATE_FORWARD_IN_TIME);
	mainMenu->addVariable("CHECK_DERIVATIVES", mesh->checkDerivatives);
	mainMenu->addVariable("TEST_Q_FD", ik->TEST_Q_FD);
	mainMenu->addVariable("TEST_R_FD", ik->TEST_R_FD);
	mainMenu->addVariable("HIGH_PRECISION_NEWTON", mesh->HIGH_PRECISION_NEWTON);
	mainMenu->addVariable("LINEAR_APPROX", ik->LINEAR_APPROX);
	mainMenu->addVariable("REGULARIZE_U", ik->REGULARIZE_u);
	mainMenu->addVariable("r_u_", ik->r_u_);
	mainMenu->addVariable("SUBSEQUENT_U", ik->SUBSEQUENT_u);
	mainMenu->addVariable("s_u_", ik->s_u_);
	mainMenu->addGroup("loco");
	mainMenu->addVariable("SELECTED_FRAME_i", ik->SELECTED_FRAME_i);
	mainMenu->addGroup("z");
	mainMenu->addVariable("appIsRunning", appIsRunning);
	mainMenu->addVariable("STEP", STEP);
	mainMenu->addVariable("PLAY_PREVIEW", PLAY_PREVIEW);
	mainMenu->addVariable("ENABLE_SPLINE_INTERACTION", ENABLE_SPLINE_INTERACTION);
	mainMenu->addButton("PROJECT SPLINE", [&]() {for (int t = 0; t < ik->T(); ++t) { test_splines[t][0]->y() = 0.; } });
	mainMenu->addButton("GRADIENT MAGNITUDE", [&]() { cout << "|G| = " << stack_vec_dRowVector(ik->calculate_dOdyJ(ik->yJ_curr, ik->xJ_curr)).squaredNorm() << endl; });
	mainMenu->addButton("OBJECTIVE_VALUE", [&]() { cout << " O  = " << ik->calculate_OJ(ik->yJ_curr)   << endl; }); 
	mainMenu->addButton("Q_VALUE",         [&]() { cout << " Q  = " << ik->calculate_QJ(ik->uJ_curr()) << endl; }); 
	mainMenu->addButton("R_VALUE",         [&]() { cout << " R  = " << ik->calculate_RJ(ik->uJ_curr()) << endl; }); 
	mainMenu->addButton("FD_TEST_dOJdyJ", [&]() { ik->FD_TEST_dOJdyJ(ik->yJ_curr, ik->xJ_curr); });
	mainMenu->addVariable("FD_TEST_STEPSIZE", ik->FD_TEST_STEPSIZE);
	mainMenu->addVariable("_DYNAMICS_MAX_ITERATIONS", mesh->_DYNAMICS_MAX_ITERATIONS);
	mainMenu->addVariable("_DYNAMICS_SOLVE_RESIDUAL", mesh->_DYNAMICS_SOLVE_RESIDUAL);
	menuScreen->performLayout(); 
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

	glMasterPush(); {
		test_frame.glAffineTransform();
		// --
		glPointSize(5.);
		glLineWidth(2.);
		// --
		glBegin(GL_POINTS); {
			for (int i = 0; i < ik->T(); ++i) { 
				set_color(kelly_color(i));
				auto &test_spline = test_splines[i];
				for (auto &test_point : test_spline) {
					glP3D(*test_point);
				}
			}
		} glEnd();
		// --
		auto uJ_curr_T = ik->zipunzip(ik->uJ_of_yJ(ik->yJ_curr));
		auto K_range = linspace(ik->K, 0., 1.);
		for (int t = 0; t < ik->T(); ++t) {
			glBegin(GL_LINE_STRIP); {
				set_color(kelly_color(t));
				// (*)
				glvecP3D(zip_vec_dVector2vecP3D({ vecDouble2dVector(K_range), uJ_curr_T[t]/mesh->tendons[t]->get_alphaz() }));
			} glEnd();
		} 
	} glMasterPop();

	DRAW_HANDLERS = false;
	PlushApplication::drawScene(); 
	draw_floor2d();

	if (ENABLE_SPLINE_INTERACTION) {
		for (int t = 0; t < ik->T(); ++t) {
			auto &test_spline = test_splines[t];
			for (int z = 0; z < ik->Z; ++z) {
				ik->yJ_curr[z][t] = test_spline[z]->y() * mesh->tendons[t]->get_alphaz();
			}
		}
		ik->xJ_curr = ik->xJ_of_yJ(ik->yJ_curr);
	}

	if (!PLAY_PREVIEW) {
		desiredFrameRate = 30;
		// --
		PREVIEW_i = -LEADIN_FRAMES;
		// --
		ik->draw();
		mesh->draw(); // FORNOW 

	} else if (PLAY_PREVIEW) { 
		desiredFrameRate = 100;
		// -- 
		if (!POPULATED_PREVIEW_TRAJEC) {
			POPULATED_PREVIEW_TRAJEC = true;
			uJ_preview = ik->uJ_curr();
			for (int _ = 0; _ < LEADOUT_FRAMES; ++_) { uJ_preview.push_back(uJ_preview.back()); }
			xJ_preview = ik->solve_trajectory(mesh->timeStep, ik->xm1_curr, ik->vm1_curr, uJ_preview); 
		} 

		PREVIEW_i++;
		if (PREVIEW_i < 0) {
			mesh->draw(ik->xm1_curr);
		} else if (PREVIEW_i < (int) uJ_preview.size()) { // TODO: min() logic
			mesh->draw(xJ_preview[PREVIEW_i], uJ_preview[PREVIEW_i]); 
		} else {
			mesh->draw(xJ_preview.back(), uJ_preview.back()); 
		}

	}

	PlushApplication::recordVideo();
}

void AppSoftLoco::process() {
	if (!PLAY_PREVIEW) {
		POPULATED_PREVIEW_TRAJEC = false;
		// ik->COMp_FORNOW = ik->COMpJ[0]; // !!!
		// Zik->COMp       = ik->COMpJ[0]; // !!!
		// ik->COMp_FORNOW = ik->COMpJ[ik->K - 1]; // !!!
		// Zik->COMp       = ik->COMpJ[ik->K - 1]; // !!!
		// --
		// Zik->SOLVE_DYNAMICS = ik->SOLVE_DYNAMICS;
		// -- 
		if (INTEGRATE_FORWARD_IN_TIME) { ik->xm1_curr = mesh->x; ik->vm1_curr = mesh->v; }
		// if (INTEGRATE_FORWARD_IN_TIME) { Zik->x_0 = mesh->x; Zik->v_0 = mesh->v; } // FORNOW
		if (SOLVE_IK) {
			ik->step();
			// cout << endl << "--> Z_ik" << endl;
			// Zik->step();
			// getchar();

			// BEG***
			for (int t = 0; t < ik->T(); ++t) {
				auto &test_spline = test_splines[t];
				for (int z = 0; z < ik->Z; ++z) {
					test_spline[z]->y() = ik->yJ_curr[z][t]/mesh->tendons[t]->get_alphaz();
				}
			}
			// ***END
		}
		if (INTEGRATE_FORWARD_IN_TIME) { mesh->xvPair_INTO_Mesh((ik->SOLVE_DYNAMICS) ? mesh->solve_dynamics(ik->xm1_curr, ik->vm1_curr, ik->uJ_curr()[0]) : mesh->solve_statics(ik->xm1_curr, ik->uJ_curr()[0])); }
		// if (INTEGRATE_FORWARD_IN_TIME) { mesh->xvPair_INTO_Mesh((Zik->SOLVE_DYNAMICS) ? mesh->solve_dynamics(Zik->x_0, Zik->v_0, Zik->alphac_curr) : mesh->solve_statics(Zik->x_0, Zik->alphac_curr)); } // FORNOW
	}
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

