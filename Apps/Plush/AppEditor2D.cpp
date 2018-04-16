#include "AppEditor2D.h"

AppEditor2D::AppEditor2D() { 

	mesh = new LoopMesh();
	push_back_handler(mesh);

	/*
	string TEST_CASE = "Canty";
	if (TEST_CASE == "Canty") {
		double X = 50. / 100.;
		double Y = 05. / 100.;
		double Z = 06. / 100.;
		auto store = [this](const P3D &p) {
			mesh->boundary.push_back(p);
		};
		mesh->boundary.clear();
		store(P3D(.5*X, .5*Y));
		store(P3D(-.5*X, .5*Y));
		store(P3D(-.5*X, -.5*Y));
		store(P3D(.5*X, -.5*Y));
	}
	*/

	load_sugar("../Apps/Plush/data/loop/sugar");

	{
		showGroundPlane = false;
		showDesignEnvironmentBox = false;
		showConsole = false;
		showFPS = false;
		appIsRunning = true;
	}

	mainMenu->addGroup("app");
	mainMenu->addVariable("mode", mesh->mode) -> setItems({"Mesh", "Tendon"});
	mainMenu->addVariable("max tri area",  mesh->TRIANGLE_MAX_AREA);
	mainMenu->addVariable("triangulate",   mesh->TRIANGULATE, "key=TAB");
	mainMenu->addVariable("draw axes",     DRAW_AXES);
	mainMenu->addVariable("draw mirror",   mesh->DRAW_MIRROR);
	mainMenu->addVariable("symmetrize",    SYMMETRIZE);
	mainMenu->addVariable("load loop",     LOAD_LOOP);
	mainMenu->addVariable("dump loop",     DUMP_LOOP);
	mainMenu->addVariable("load plushie",  LOAD_PLUSHIE);
	mainMenu->addVariable("dump plushie",   DUMP_PLUSHIE);
	mainMenu->addButton("Num triangles?", [&]() {cout << "NUM_TRIANGLES: " << mesh->triangulated_triangles.size() << endl; });
	mainMenu->addButton("Load tri", [&]() { cout << "Loading tri..." << endl; load_sugar("../Apps/Plush/data/loop/tri"); });
	menuScreen->performLayout(); 
}

AppEditor2D::~AppEditor2D(void){
	{};
}

////////////////////////////////////////////////////////////////////////////////
// step ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void AppEditor2D::handle_toggles() { 
	if (LOAD_LOOP) {
		LOAD_LOOP = false;
		cout << "loading loop..." << endl;
		load_sugar("../Apps/Plush/data/loop/sugar");
	}

	if (DUMP_LOOP) {
		DUMP_LOOP = false;
		cout << "dumping loop..." << endl;
		saveFile("../Apps/Plush/data/loop/sugar");
	}

	if (DUMP_PLUSHIE) {
		DUMP_PLUSHIE = false;
		cout << "dumping plushie..." << endl;
		mesh->dump("../Apps/Plush/data/tri/sugar");
	}

	if (LOAD_PLUSHIE) {
		LOAD_PLUSHIE = false;
		cout << "loading plushie..." << endl;
		preview = new CSTSimulationMesh2D();
		preview->spawnSavedMesh("../Apps/Plush/data/tri/sugar");
	}
 
	if (SYMMETRIZE) {
		SYMMETRIZE = false;
		mesh->sloppy_symmetrize();
	}

}

void AppEditor2D::step() { 
	handle_toggles();
}

////////////////////////////////////////////////////////////////////////////////
// draw ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void AppEditor2D::drawScene() { 

	mesh->draw();

	if (preview != nullptr) {
		glMasterPush(); {
			glTranslated(2., 0., 0.);
			preview->draw();
		} glMasterPop(); 
	}

	if (DRAW_AXES) {
		set_color(LIGHT_CLAY);
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glLineWidth(4);
		glBegin(GL_LINES);
		glP3Dz(P3D(-100., 0.), 1);
		glP3Dz(P3D(100., 0.), 1);
		glP3Dz(P3D(0., -100.), 1);
		glP3Dz(P3D(0., 100.), 1);
		glEnd();
		glPopAttrib();
	}

	dynamic_cast<GLTrackingCamera *>(camera)->rotAboutRightAxis = 0;
	dynamic_cast<GLTrackingCamera *>(camera)->rotAboutUpAxis = 0;
 
}

////////////////////////////////////////////////////////////////////////////////
// glfw ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool AppEditor2D::onMouseMoveEvent(double xPos, double yPos) { 
	if (PlushApplication::onMouseMoveEvent(xPos, yPos)) { return true; }
	return false;
}

bool AppEditor2D::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (PlushApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) { return true; } 
	return false;
}

bool AppEditor2D::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (PlushApplication::onMouseWheelScrollEvent(xOffset, yOffset)) { return true; }
	return false;
}

bool AppEditor2D::onKeyEvent(int key, int action, int mods) {
	if (PlushApplication::onKeyEvent(key, action, mods)) { return true; }
	return false;
}

bool AppEditor2D::onCharacterPressedEvent(int key, int mods) {
	if (PlushApplication::onCharacterPressedEvent(key, mods)) { return true; }
	return false;
}

bool AppEditor2D::processCommandLine(const std::string& cmdLine) {
	if (PlushApplication::processCommandLine(cmdLine)) {
		return true;
	}
	return false;
}

////////////////////////////////////////////////////////////////////////////////
// loading/saving //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void AppEditor2D::loadFile(const char* fName) {
	string stripped = string(fName);
	stripped = stripped.substr(0, stripped.find('.'));
	load_sugar(stripped.data()); 
}

void AppEditor2D::load_sugar(const char* fName) {
	mesh->boundary.clear();
	mesh->sketches.clear();

	char vName[128];
	char sketchName[128];
	apply_suffix(fName, "loop", vName);
	apply_suffix(fName, "sketch", sketchName); 
	FILE* v_fp = fopen(vName, "r");
	FILE* sketch_fp = fopen(sketchName, "r");

	P3D p;
	while (fscanf(v_fp, "%lf %lf", &p[0], &p[1]) != EOF) {
		mesh->boundary.push_back(new P3D(p));
	} 
	fclose(v_fp); 
 
	vector<vector<P3D>> tendons_as_vecVecP2D;
	if (sketch_fp != nullptr) {

		P3D s = P3D();

		const int LINESZ = 1024;
		char line[LINESZ]; 
		while (fgets(line, LINESZ, sketch_fp) != NULL) { 
			vector<P3D> tendon;
			int nums_now, bytes_now;
			int bytes_consumed = 0, nums_read = 0;
			while ((nums_now = sscanf(line + bytes_consumed, "%lf %lf %n", &s[0], &s[1], &bytes_now)) > 0) {
				bytes_consumed += bytes_now; nums_read += nums_now;
				tendon.push_back(s);
			} 

			tendons_as_vecVecP2D.push_back(tendon);
		}
		fclose(sketch_fp); 
	}

	for (auto &tendon : tendons_as_vecVecP2D) {
		vector<P3D *> *sketch = new vector<P3D *>();
		for (auto &p : tendon) {
			sketch->push_back(new P3D(p));
		}
		mesh->sketches.push_back(sketch);
	}

 }
 
void AppEditor2D::saveFile(const char* fName) {
	char vName[128];
	char sketchName[128];
	apply_suffix(fName, "loop", vName);
	apply_suffix(fName, "sketch", sketchName); 
	FILE* v_fp = fopen(vName, "w");
	FILE* sketch_fp = fopen(sketchName, "w");


	for (auto &p : mesh->boundary) {
		fprintf(v_fp, "%lf %lf\n", p->x(), p->y());
	}

	if (sketch_fp != nullptr) {
		for (auto &sketch : mesh->sketches) {
			for (auto &p : *sketch) {
				fprintf(sketch_fp, "%lf %lf ", p->x(), p->y());
			}
			fprintf(sketch_fp, "\n");
		} 
	}

	fclose(v_fp); 
	fclose(sketch_fp); 
}

////////////////////////////////////////////////////////////////////////////////
// misc ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void AppEditor2D::process() { AppEditor2D::step(); }
void AppEditor2D::drawAuxiliarySceneInfo() { }
void AppEditor2D::restart() { }
 
/* if (selectedNodeID != -1){
	Plane plane(camera->getCameraTarget(), V3D(camera->getCameraPosition(), camera->getCameraTarget()).unit());
	P3D targetPinPos; getRayFromScreenCoords(xPos, yPos).getDistanceToPlane(plane, &targetPinPos);
	plushie->setPinnedNode(selectedNodeID, targetPinPos);
	return true;
} */
// selectedNodeID = plushie->getSelectedNodeID(lastClickedRay);
