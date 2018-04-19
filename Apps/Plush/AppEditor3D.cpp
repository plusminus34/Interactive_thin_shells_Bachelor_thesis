#include "AppEditor3D.h"

AppEditor3D::AppEditor3D() { 


	// loadDesign(DESIGN_PATH_.data());

	{
		// showGroundPlane = true;
		showDesignEnvironmentBox = true;
	}

	design = new PlateMesh(camera, drag_plane);
	design->BAKE_TETS = false;
	// design->MAX_TET_VOLUME = .005;

	string TEST_CASE = "Hexy";

	if (TEST_CASE == "Hexy") {
		int N = 3;
		// --
		double dtheta = 2.*PI / double(N);

		// --

		vector<P3D> loop;
		auto store = [&](const V3D &v) {
			P3D p = P3D(v[0], 0., v[1]);
			loop.push_back(p);
		};

		double R = 12./100.;
		double L = .5*(70. - 2.*R)/100.;
		L *= 2; // FORNOW
		double W = 06./100.;
		for (int i = 0; i < N; ++i) {
			double theta = double(i) * dtheta;
			double theta_M = theta - .5*dtheta;
			double theta_P = theta + .5*dtheta;
			// --
			V3D r = e_theta_2D(theta);
			V3D r_M = e_theta_2D(theta_M);
			V3D r_P = e_theta_2D(theta_P);
			// --
			store(r_M*R);
			store(r_M*R + r*L);
			store(r_P*R + r*L);
		}

		auto rangeLen = [](const vector<P3D> &loop) {
			vector<int> ret;
			for (size_t i = 0; i < loop.size(); ++i) {
				ret.push_back(i);
			}
			return ret;
		};

		auto offset_loop = [&](const vector<P3D> &loop, const double &y_offset) {
			vector<P3D> ret;
			for (auto &p : loop) {
				ret.push_back(p + V3D(0., y_offset));
			}
			return ret;
		};

		auto join_loops = [&](const vector<int> &a, const vector<int> &b_) {
			auto b = b_;
			std::reverse(b.begin(), b.end());
			// --
			if (a.size() != b.size()) { error("NotImplementedError"); }
			// --
			int N = a.size();
			// --
			vector<vector<int>> ret;
			for (int ii = 0; ii < N; ++ii) {
				int jj = (ii + 1) % N;
				// --
				int i_a = a[ii];
				int i_b = b[ii];
				int j_a = a[jj];
				int j_b = b[jj];
				// --
				ret.push_back({ i_a, i_b, j_b, j_a });
			}
			return ret;
		};

		// loop
		double thickness = W;
		auto loop_ = loop;
		loop = offset_loop(loop_, -.5*thickness);
		vector<P3D> loop2 = offset_loop(loop_, .5*thickness);
		// --
		design->points.clear();
		design->plates.clear();
		// --
		for (auto &p : loop) { design->points.push_back(p); }
		for (auto &p : loop2) { design->points.push_back(p); }
		// --
		vector<int> loop_spec = rangeLen(loop);
		vector<int> loop2_spec; for (auto &i : loop_spec) { loop2_spec.push_back(i + loop.size()); }
		std::reverse(loop2_spec.begin(), loop2_spec.end());
		// --
		design->plates.push_back(new Plate(loop_spec, design));
		design->plates.push_back(new Plate(loop2_spec, design));
		for (auto &spec : join_loops(loop_spec, loop2_spec)) { design->plates.push_back(new Plate(spec, design)); }

	} else if (TEST_CASE == "Canty") {
		vector<P3D> loop;
		auto store = [&](const P3D &p) {
			loop.push_back(P3D(p[0], 0., p[1]));
		};

		double X = 50./100.;
		double Y = 05./100.;
		double Z = 06./100.;
		store(P3D( .5*X,  .5*Y));
		store(P3D(-.5*X,  .5*Y));
		store(P3D(-.5*X, -.5*Y));
		store(P3D( .5*X, -.5*Y)); 

		auto rangeLen = [](const vector<P3D> &loop) {
			vector<int> ret;
			for (size_t i = 0; i < loop.size(); ++i) {
				ret.push_back(i);
			}
			return ret;
		};

		auto offset_loop = [&](const vector<P3D> &loop, const double &y_offset) {
			vector<P3D> ret;
			for (auto &p : loop) {
				ret.push_back(p + V3D(0., y_offset));
			}
			return ret;
		};

		auto join_loops = [&](const vector<int> &a, const vector<int> &b_) {
			auto b = b_;
			std::reverse(b.begin(), b.end());
			// --
			if (a.size() != b.size()) { error("NotImplementedError"); }
			// --
			int N = a.size();
			// --
			vector<vector<int>> ret;
			for (int ii = 0; ii < N; ++ii) {
				int jj = (ii + 1) % N;
				// --
				int i_a = a[ii];
				int i_b = b[ii];
				int j_a = a[jj];
				int j_b = b[jj];
				// --
				ret.push_back({ i_a, i_b, j_b, j_a });
			}
			return ret;
		};

		// loop
		double thickness = Z;
		auto loop_ = loop;
		loop = offset_loop(loop_, -.5*thickness);
		vector<P3D> loop2 = offset_loop(loop_, .5*thickness);
		// --
		design->points.clear();
		design->plates.clear();
		// --
		for (auto &p : loop) { design->points.push_back(p); }
		for (auto &p : loop2) { design->points.push_back(p); }
		// --
		vector<int> loop_spec = rangeLen(loop);
		vector<int> loop2_spec; for (auto &i : loop_spec) { loop2_spec.push_back(i + loop.size()); }
		std::reverse(loop2_spec.begin(), loop2_spec.end());
		// --
		design->plates.push_back(new Plate(loop_spec, design));
		design->plates.push_back(new Plate(loop2_spec, design));
		for (auto &spec : join_loops(loop_spec, loop2_spec)) { design->plates.push_back(new Plate(spec, design)); }

		design->MAX_TET_VOLUME = .00010;

	}

	design->LOAD = true;
	push_back_handler(design);

	mainMenu->addGroup("xxx");

	// mainMenu->addVariable("mode", design->mode) -> setItems({"Mesh", "Tendon"});

	mainMenu->addVariable("MAX_TET_VOLUME",  design->MAX_TET_VOLUME);
	// mainMenu->addVariable("TETRAHEDRALIZE",  design->PLEASE_REBUILD_SIMULATION_MESH, "key=TAB");

	// mainMenu->addButton("SAVE_DESIGN_", [&]() { cout << "TODO: " << DESIGN_PATH_ << endl; });
	// mainMenu->addButton("LOAD_DESIGN_", [&]() { cout << "TODO: " << DESIGN_PATH_ << endl; });
	// mainMenu->addButton("EXPO_PLUSHIE", [&]() { cout << "TODO: " << PLUSHIE_PATH << endl; });
	// mainMenu->addButton("TEST_PLUSHIE", [&]() { cout << "TODO: " << PLUSHIE_PATH << endl; });

	menuScreen->performLayout(); 



}

void AppEditor3D::process() { }

////////////////////////////////////////////////////////////////////////////////
// draw ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void AppEditor3D::drawScene() { 

		if (view == View::Designer) {
		PlushApplication::DRAW_HANDLERS = true;
		// --
		this->showReflections = false;
		this->showGroundPlane = false;
		// --
		design->SIMULATION_STARTED = false;
		// --
		design->BAKE_TETS = false;
		design->BAKE_TDNS = false;
		// --
		design->DRAW_DESIGNER = true;
		design->DRAW_BAKE = false;
		design->DRAW_SIMULATION = false;
	} else if (view == View::Checker) {
		PlushApplication::DRAW_HANDLERS = false;
		// --
		this->showReflections = false;
		this->showGroundPlane = false;
		// --
		design->SIMULATION_STARTED = false;
		// --
		design->BAKE_TETS = true;
		design->BAKE_TDNS = true;
		// --
		design->DRAW_DESIGNER = false;
		design->DRAW_BAKE = true;
		design->DRAW_SIMULATION = false; }
	else if (view == View::Simulator) {
		PlushApplication::DRAW_HANDLERS = false;
		// --
		this->showReflections = true; // FORNOW
		this->showGroundPlane = true; // FORNOW
		// --
		design->BAKE_TETS = false;
		design->BAKE_TDNS = false;
		// --
		design->DRAW_DESIGNER = false;
		design->DRAW_BAKE = false;
		design->DRAW_SIMULATION = true; 

		if (!design->SIMULATION_STARTED) {
			design->SIMULATION_STARTED = true;
			// --
			if (design->simMesh != nullptr) {
				design->reset_simulation();
			}
		} else {
			if (design->simMesh != nullptr) {
				if (appIsRunning) { design->step_simulation(); }
			}
		}
	} else {
		error("NotImplemented"); 
	}


	design->draw();

	// if (plushie != nullptr) {
	// 	glMasterPush(); {
	// 		glTranslated(2., 0., 0.);
	// 		plushie->draw();
	// 	} glMasterPop(); 
	// }

}

////////////////////////////////////////////////////////////////////////////////
// glfw ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool AppEditor3D::onMouseMoveEvent(double xPos, double yPos) { 
	if (PlushApplication::onMouseMoveEvent(xPos, yPos)) { return true; }
	return false;
}

bool AppEditor3D::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (PlushApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) { return true; } 
	return false;
}

bool AppEditor3D::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (PlushApplication::onMouseWheelScrollEvent(xOffset, yOffset)) { return true; }
	return false;
}

bool AppEditor3D::onKeyEvent(int key, int action, int mods) {
	if (PlushApplication::onKeyEvent(key, action, mods)) { return true; }
	return false;
}

bool AppEditor3D::onCharacterPressedEvent(int key, int mods) {

	if (key == ' ') {
		view = View((view + 1) % 3); 
		return true;
	}
	if (PlushApplication::onCharacterPressedEvent(key, mods)) { return true; }
	return false;
}

bool AppEditor3D::processCommandLine(const std::string& cmdLine) {
	if (PlushApplication::processCommandLine(cmdLine)) { return true; }
	return false;
}

////////////////////////////////////////////////////////////////////////////////
// loading/saving //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void AppEditor3D::loadFile(const char* fName) {
	string stripped = string(fName);
	stripped = stripped.substr(0, stripped.find('.'));
	loadDesign(stripped.data()); 
}

void AppEditor3D::loadDesign(const char* fName) {
}
 
void AppEditor3D::saveFile(const char* fName) {
}

////////////////////////////////////////////////////////////////////////////////
// misc ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void AppEditor3D::drawAuxiliarySceneInfo() { }
 