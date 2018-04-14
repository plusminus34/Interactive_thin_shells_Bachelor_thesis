#include "AppEditor3D.h"

AppEditor3D::AppEditor3D() { 

	design = new LoopMesh();
	push_back_handler(design);

	load_sugar("../Apps/Plush/data/loop/sugar");

	{
		showGroundPlane = true;
		showDesignEnvironmentBox = true;
	}

	mainMenu->addGroup("xxx");

	mainMenu->addVariable("mode", design->mode) -> setItems({"Mesh", "Tendon"});

	mainMenu->addVariable("MAX_TET_VOLUME",  design->TRIANGLE_MAX_AREA);
	mainMenu->addVariable("TETRAHEDRALIZE",  design->TRIANGULATE, "key=TAB");

	mainMenu->addButton("SAVE_DESIGN_", [&]() { cout << "TODO: " << DESIGN_PATH_ << endl; });
	mainMenu->addButton("LOAD_DESIGN_", [&]() { cout << "TODO: " << DESIGN_PATH_ << endl; });
	mainMenu->addButton("EXPO_PLUSHIE", [&]() { cout << "TODO: " << PLUSHIE_PATH << endl; });
	mainMenu->addButton("TEST_PLUSHIE", [&]() { cout << "TODO: " << PLUSHIE_PATH << endl; });

	menuScreen->performLayout(); 
}

void AppEditor3D::process() { }

////////////////////////////////////////////////////////////////////////////////
// draw ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void AppEditor3D::drawScene() { 

	design->draw();

	if (plushie != nullptr) {
		glMasterPush(); {
			glTranslated(2., 0., 0.);
			plushie->draw();
		} glMasterPop(); 
	}

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
	load_sugar(stripped.data()); 
}

void AppEditor3D::load_sugar(const char* fName) {
	design->boundary.clear();
	design->sketches.clear();

	char vName[128];
	char sketchName[128];
	apply_suffix(fName, "loop", vName);
	apply_suffix(fName, "sketch", sketchName); 
	FILE* v_fp = fopen(vName, "r");
	FILE* sketch_fp = fopen(sketchName, "r");

	P3D p;
	while (fscanf(v_fp, "%lf %lf", &p[0], &p[1]) != EOF) {
		design->boundary.push_back(new P3D(p));
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
		design->sketches.push_back(sketch);
	}

 }
 
void AppEditor3D::saveFile(const char* fName) {
	char vName[128];
	char sketchName[128];
	apply_suffix(fName, "loop", vName);
	apply_suffix(fName, "sketch", sketchName); 
	FILE* v_fp = fopen(vName, "w");
	FILE* sketch_fp = fopen(sketchName, "w");


	for (auto &p : design->boundary) {
		fprintf(v_fp, "%lf %lf\n", p->x(), p->y());
	}

	if (sketch_fp != nullptr) {
		for (auto &sketch : design->sketches) {
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

void AppEditor3D::drawAuxiliarySceneInfo() { }
 