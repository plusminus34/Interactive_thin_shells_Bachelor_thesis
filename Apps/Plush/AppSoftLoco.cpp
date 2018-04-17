#include "AppSoftLoco.h"
#include "XYPlot.h"
 
using namespace nanogui;

// TODO: Make material parameters properties of the mesh so you can scrub them and see how it impacts the sim
// TODO: (Including mass density).

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
// TODO: Have both a low and high res triangulation which you can switch between optimizing with
// Low for quick sketches

// TODO: Material damping? / Regularizer?
// TODO: Squishier material model?
// TODO: Throttle contraction _rate_?
// TODO: Throttle _reaction forces_?
// TODO: Different smoothing kernel?
// TODO: Aiiiiiir resistance?

// TODO: Squishier floor?
// TODO: Small timestep? (TODO: Should do at least a first validation by resampling buggy u-trajectory to a smaller timestep_

AppSoftLoco::AppSoftLoco() {
    setWindowTitle("AppSoftLoco");
	this->showReflections = false;
	this->showGroundPlane = false;
	this->DEFAULT_CAM_TARGET______________ = P3D(0., 0.);
	this->DEFAULT_CAM_DISTANCE____________ = -4.5;
	this->SPOOF_2D_CAMERA = true;
	this->resetCamera();

	// const vector<string> TEST_CASES = {
	// 	"swingup",      // 0
	// 	"tri",          // 1
	// 	"tentacle",     // 2
	// 	"mini_swingup", // 3
	// 	"3ball",        // 4
	// 	"walker",       // 5
	// 	"jumping_cube", // 6
	// 	"6ball",        // 7
	// 	"sugar",        // 8
	// 	"T"             // 9
	// };
	string TEST_CASE = "2biped"; // TEST_CASES[1];

	// TODO: Could have yJ_curr, mJ_curr (but then all functions need two arguments)

	// -- // mesh
	char fName[128]; strcpy(fName, "../Apps/Plush/data/tri/"); strcat(fName, TEST_CASE.data());
	mesh = new CSTSimulationMesh2D();
	mesh->timeStep = .01;
	mesh->spawnSavedMesh(fName, true);
	// mesh->rotate_mesh(PI);
	mesh->nudge_mesh_up();
	mesh->applyYoungsModulusAndPoissonsRatio(3e5, .25); // FORNOW
	mesh->addGravityForces(V3D(0., -10.)); 

	if (TEST_CASE == "tri") { mesh->pinToFloor(); }
	if (TEST_CASE == "swingup") { mesh->pinToLeftWall(); }
	if (mesh->pins.empty()) { mesh->add_contacts_to_boundary_nodes(); }

	if (TEST_CASE == "tri") {
		// mesh->rig_boundary_simplices();
		mesh->add_tendon_from_vecInt({ 0, 2 });
	}
	// mesh->rig_boundary_simplices();
	// mesh->rig_all_lower_simplices();

	for (size_t i = 0; i < mesh->tendons.size(); ++i) {
		mesh->tendons[i]->SPEC_COLOR = kelly_color(i);
	}

	// FORNOW
	if (mesh->pins.empty()) { for (int _ = 0; _ < 1000; ++_) { mesh->xvPair_INTO_Mesh(mesh->solve_dynamics()); } }

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
		ik->REGULARIZE_u = false;
		ik->SUBSEQUENT_u = false;
		mesh->HIGH_PRECISION_NEWTON = false;
		ik->COMpJ.back() += V3D(1., 0.); 
		appIsRunning = true;
		ENABLE_SPLINE_INTERACTION = false;
	}


	// FORNOW
	{
		for (int i = 0; i < ik->T(); ++i) {
			vector<P3D *> positions;
			for (int z = 0; z < ik->Z; ++z) {
				P3D *position = new P3D(dfrac(z, ik->Z - 1), 0.);
				positions.push_back(position);
			}
			all_positions.push_back(positions);
		}

		for (int i = 0; i < ik->T(); ++i) {
			vector<P3D *> tangents;
			for (int z = 0; z < ik->Z; ++z) {
				P3D *tangent = new P3D(dfrac(z, ik->Z - 1), 0.);
				tangents.push_back(tangent);
			}
			all_tangents.push_back(tangents);
		}
		splinePositionsDragger = new P2DDragger_v2(flatten(all_positions), &splinePositionsFrame, true, -1., 1.);
		splineTangentsDragger  = new P2DDragger_v2(flatten(all_tangents) , &splineTangentsFrame,  true);
		push_back_handler2(splinePositionsDragger);
		push_back_handler2(splineTangentsDragger);
	}

	scrubber = new Scrubber(&PREVIEW_i, PREVIEW_LENGTH());
	push_back_handler2(scrubber);

	mainMenu->addGroup("app");
	mainMenu->addVariable("draw F", mesh->DRAW_NODAL_FORCES);
	mainMenu->addButton("save_uJ", [&]() { save_uJ(); });
	mainMenu->addButton("load_uJ", [&]() { load_uJ(); });
	mainMenu->addButton("populatePreviewTraj", [&]() { populatePreviewTrajec(); });
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
	// mainMenu->addButton("PROJECT SPLINE", [&]() {for (int t = 0; t < ik->T(); ++t) { all_positions[t][0]->y() = 0.; } });
	mainMenu->addButton("CHECK CONSTRAINTS", [&]() { ik->constraints->checkConstraints(ik->ymJ_curr); });
	mainMenu->addButton("GRADIENT MAGNITUDE", [&]() { cout << "|G| = " << stack_vec_dRowVector(ik->calculate_dOdymJ(ik->ymJ_curr, ik->xJ_curr)).squaredNorm() << endl; });
	mainMenu->addButton("OBJECTIVE_VALUE", [&]() { cout << " O  = " << ik->calculate_OJ(ik->ymJ_curr)   << endl; }); 
	mainMenu->addButton("Q_VALUE",         [&]() { cout << " Q  = " << ik->calculate_QJ(ik->uJ_curr()) << endl; }); 
	mainMenu->addButton("R_VALUE",         [&]() { cout << " R  = " << ik->calculate_RJ(ik->uJ_curr()) << endl; }); 
	mainMenu->addButton("FD_TEST_dOJdymJ", [&]() { ik->FD_TEST_dOJdymJ(ik->ymJ_curr, ik->xJ_curr); });
	mainMenu->addVariable("FD_TEST_STEPSIZE", ik->FD_TEST_STEPSIZE);
	mainMenu->addVariable("_DYNAMICS_MAX_ITERATIONS", mesh->_DYNAMICS_MAX_ITERATIONS);
	mainMenu->addVariable("_DYNAMICS_SOLVE_RESIDUAL", mesh->_DYNAMICS_SOLVE_RESIDUAL);
	mainMenu->addVariable("FPS", this->desiredFrameRate);
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
		splinePositionsFrame.glAffineTransform();
		// --
		glPointSize(5.);
		glLineWidth(2.);
		// --
		glBegin(GL_POINTS); {
			for (int i = 0; i < ik->T(); ++i) {
				set_color(kelly_color(i));
				for (auto &position : all_positions[i]) {
					glP3D(*position);
				}
			}
		} glEnd();
		// --
		auto uJ_curr_T = ik->zipunzip(ik->uJ_of_ymJ(ik->ymJ_curr));
		auto K_range = linspace(ik->K, 0., 1.);
		for (int t = 0; t < ik->T(); ++t) {
			glBegin(GL_LINE_STRIP); {
				set_color(kelly_color(t));
				glvecP3D(zip_vec_dVector2vecP3D({ vecDouble2dVector(K_range), uJ_curr_T[t]/mesh->tendons[t]->get_alphaz() }));
			} glEnd();
		} 
	} glMasterPop();

	glMasterPush(); {
		splineTangentsFrame.glAffineTransform();
		// --
		glPointSize(5.);
		glLineWidth(2.);
		// --
		for (int t = 0; t < ik->T(); ++t) { 
			set_color(kelly_color(t));
			for (auto &GL_PRIMITIVE : { GL_LINES, GL_POINTS }) {
				glBegin(GL_PRIMITIVE); {
					for (int z = 0; z < ik->Z; ++z) {
						auto &tangent = all_tangents[t][z];
						// --
						glP3D(P3D(tangent->x(), 0.));
						glP3D(P3D(*tangent));
					}
				} glEnd();
			}
		}
	} glMasterPop();

	DRAW_HANDLERS = false;
	PlushApplication::drawScene(); 
	draw_floor2d();

	// TODO: Tangent interaction
	if (ENABLE_SPLINE_INTERACTION) {
		for (int z = 0; z < ik->Z; ++z) {
			for (int t = 0; t < ik->T(); ++t) { ik->ymJ_curr[z][t]           = all_positions[t][z]->y() * mesh->tendons[t]->get_alphaz(); }
			for (int t = 0; t < ik->T(); ++t) { ik->ymJ_curr[z][ik->T() + t] = all_tangents[t][z]->y() * mesh->tendons[t]->get_alphaz(); }
		}
		ik->xJ_curr = ik->xJ_of_ymJ(ik->ymJ_curr);
	}

	if (!PLAY_PREVIEW) {
		// desiredFrameRate = 30;
		// --
		PREVIEW_i = 0;
		// --
		ik->draw();
		mesh->draw(); // FORNOW 

	} else if (PLAY_PREVIEW) { 
		// desiredFrameRate = 100;
		// -- 


		if (POPULATED_PREVIEW_TRAJEC) {
			mesh->draw(xJ_preview[PREVIEW_i], uJ_preview[PREVIEW_i], (PREVIEW_i != 0) ? xJ_preview[PREVIEW_i - 1] : ik->xm1_curr); 
			// --
			glMasterPush(); {
				auto K_range = linspace(uJ_preview.size(), 0., 1.);
				vector<XYPlot*> plots;

				vector<double> x_spoof = { dfrac(PREVIEW_i, PREVIEW_LENGTH()) };
				concat_in_place(x_spoof, x_spoof);
				vector<double> y_spoof = { -1., 1. };
				plots.push_back(new XYPlot(x_spoof, y_spoof));

				for (int t = 0; t < ik->T(); ++t) {
					vector<double> u_K; for (int k = 0; k < (int) uJ_preview.size(); ++k) { u_K.push_back(uJ_preview[k][t] / mesh->balphaz[t]); }
					XYPlot *plot_K = new XYPlot(K_range, u_K);
					plot_K->THIN_LINES = true;
					plot_K->DRAW_POINTS = false;
					plot_K->SPEC_COLOR = kelly_color(t);
					plots.push_back(plot_K);
				}
				for (auto &plot : plots) {
					*plot->origin = P3D(-2., -1.);
					*plot->top_right = *plot->origin + V3D(2., 2.);
				};
				XYPlot::uniformize_axes(plots);
				for (auto &plot : plots) { plot->draw(); }
				for (auto &plot : plots) { delete plot; }
			} glMasterPop();
		}

	}

	PlushApplication::recordVideo();
}

void AppSoftLoco::process() {
	if (!PLAY_PREVIEW) {
		POPULATED_PREVIEW_TRAJEC = false;
		if (SOLVE_IK) {
			ik->step();
			// -- // BEG***
			for (int t = 0; t < ik->T(); ++t) {
				for (int z = 0; z < ik->Z; ++z) {
					all_positions[t][z]->y() = ik->ymJ_curr[z][t]            / mesh->tendons[t]->get_alphaz();
					all_tangents [t][z]->y() = ik->ymJ_curr[z][ik->T() + t]  / mesh->tendons[t]->get_alphaz();
				}
			}
			// ***END
		}
	}
}

void AppSoftLoco::save_uJ() {
	FILE *fp = fopen("../Apps/Plush/data/uJtmp.xxx", "w");

	for (auto &frame : uJ_preview) {
		for (int i = 0; i < frame.size(); ++i) { 
			fprintf(fp, "%lf ", frame[i]);
		}
		fprintf(fp, "\n");
	}

	fclose(fp); 
}

void AppSoftLoco::load_uJ() {
	FILE *fp = fopen("../Apps/Plush/data/uJtmp.xxx", "r");

	const int LINESZ = 1024;
	char line[LINESZ]; 
	vector<vector<double>> uJ_vecVecDouble;
	double u_double;
	while (fgets(line, LINESZ, fp) != NULL) { 
		vector<double> u_vec;
		// https://stackoverflow.com/questions/10826953/sscanf-doesnt-move-scans-same-integer-everytime-c 
		int nums_now, bytes_now;
		int bytes_consumed = 0, nums_read = 0;
		while ((nums_now = sscanf(line + bytes_consumed, "%lf %n", &u_double, &bytes_now)) > 0) {
			bytes_consumed += bytes_now; nums_read += nums_now;
			u_vec.push_back(u_double);
		} 

		uJ_vecVecDouble.push_back(u_vec);
	}

	fclose(fp);

	uJ_preview.clear();
	for (auto u_vec : uJ_vecVecDouble) {
		uJ_preview.push_back(vecDouble2dVector(u_vec));
	}
	concat_in_place(uJ_preview, uJ_preview); // FORNOW
	// uJ_preview.front().setZero();
	// uJ_preview.back().setZero();
	xJ_preview = ik->solve_trajectory(mesh->timeStep, ik->xm1_curr, ik->vm1_curr, uJ_preview); 
	POPULATED_PREVIEW_TRAJEC = true;
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
	if (key == 'r') {
		ik->xJ_curr = ik->xJ_of_ymJ(ik->ymJ_curr);
		POPULATED_PREVIEW_TRAJEC = false;
	}
	// --
	if (PlushApplication::onCharacterPressedEvent(key, mods)) { return true; } 
    return false;
} 

void AppSoftLoco::populatePreviewTrajec() {
	POPULATED_PREVIEW_TRAJEC = true;

	uJ_preview.clear();
	for (int _ = 0; _ < NUM_PREVIEW_CYCLES; ++_) {
		concat_in_place(uJ_preview, ik->uJ_curr());
		// for (int _ = 0; _ < 5; ++_) { uJ_preview.push_back(ik->uJ_curr().back()); } // FORNOW (TODO)
	}
	xJ_preview = ik->solve_trajectory(mesh->timeStep, ik->xm1_curr, ik->vm1_curr, uJ_preview);

	// --
	// prepend_in_place(uJ_preview, ZERO_dVector(ik->T()));// (*)
	// prepend_in_place(xJ_preview, ik->xm1_curr);// (*)
}
