#include "AppArt.h"
#include <PlushHelpers\helpers_star.h>
#include "P2DDragger.h"
 
AppArt::AppArt() {
	setWindowTitle("AppArt");

	pixels = (float *) malloc(sizeof(float)*W()*H()); 

	int N = 1;
	for (int i = 0; i < N; ++i) {
		auto mesh = new CSTSimulationMesh2D();
		mesh->spawnSavedMesh("../Apps/Plush/data/tri/tentacle");
		mesh->nudge_mesh(V3D(0., -.33, 0.));
		mesh->pinToFloor();
		mesh->rig_boundary_simplices();
		mesh->applyYoungsModulusAndPoissonsRatio(1e4, .25);
		mesh->rotate_mesh(2.*PI*dfrac(i, N));
		mesh->move_pins(mesh->X);
		meshes.push_back(mesh);
	}

	for (auto &mesh : meshes) {
		auto solver = new SoftIKSolver(mesh);
		solver->SPEC_COM = false;
		solver->SPEC_FREESTYLE = true;
		solver->toggle_Z(mesh->nodes[2]);
		solver->h_alphac_ = 10000.;
		solver->c_alphac_ = 10.;
		solvers.push_back(solver);
	}

	push_back_handler(new P2DDragger(&test_P3D));

	mainMenu->addGroup("app");
	menuScreen->performLayout(); 

}
 

void AppArt::drawScene() {
	PlushApplication::drawScene();
 
	glMasterPush(); {
		glDisable(GL_DEPTH_TEST);
		set_color(WHITE);
		glBegin(GL_QUADS); {
			const vector<P3D> DOUBLE_UNIT_BOX = { P3D(1., 1.), P3D(-1., 1.), P3D(-1., -1.), P3D(1., -1.) };
			glvecP3D(DOUBLE_UNIT_BOX);
		} glEnd();
	} glMasterPop();

	glMasterPush(); {
		glDisable(GL_DEPTH_TEST);
		for (auto &mesh : meshes) {
			mesh->draw();
		}
	} glMasterPop();

	glMasterPush(); {
		glDisable(GL_DEPTH_TEST);
		for (auto &solver : solvers) {
			solver->draw();
		}
	} glMasterPop();

	glMasterPush(); {
		glDisable(GL_DEPTH_TEST);
		set_color(ORCHID);
		glPointSize(5.);
		glBegin(GL_POINTS); {
			glP3D(test_P3D);
		} glEnd();
	} glMasterPop();

}

void AppArt::process() {

	for (auto &mesh : meshes) {
		mesh->nodes[2]->setTargetPosition(test_P3D);
	}

	for (auto &solver : solvers) {
		solver->step();
	}

	for (auto &solver : solvers) {
		solver->mesh->balphac = solver->alphac_curr;
	}

	for (auto &mesh : meshes) {
	    // mesh->xvPair_INTO_Mesh(mesh->solve_dynamics());
		mesh->xvPair_INTO_Mesh(mesh->solve_statics());
	}

}

bool AppArt::onMouseMoveEvent(double xPos, double yPos) {
    return(PlushApplication::onMouseMoveEvent(xPos, yPos));
}

bool AppArt::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
    return(PlushApplication::onMouseButtonEvent(button, action, mods, xPos, yPos));
}

bool AppArt::onMouseWheelScrollEvent(double xOffset, double yOffset) {
    return(PlushApplication::onMouseWheelScrollEvent(xOffset, yOffset));
}

bool AppArt::onKeyEvent(int key, int action, int mods) {	
	return PlushApplication::onKeyEvent(key, action, mods);
}

bool AppArt::onCharacterPressedEvent(int key, int mods) {
	return PlushApplication::onCharacterPressedEvent(key, mods);
}

bool AppArt::processCommandLine(const std::string& cmdLine) {
	return PlushApplication::processCommandLine(cmdLine);
}

void AppArt::loadFile(const char* fName) { } 
void AppArt::saveFile(const char* fName) { }

	/*
	glMasterPush(); {
		glEnable(GL_STENCIL_TEST);
		glStencilFunc(GL_ALWAYS, 1, 1);
		glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
		glColorMask(0, 0, 0, 0);
		mesh->draw();
		glReadPixels(0, 0, W(), H(), GL_STENCIL_INDEX, GL_FLOAT, pixels);
	} glMasterPop();

	Eigen::MatrixXf M = Eigen::Map<Eigen::MatrixXf>(pixels, W(), H()); 

	Eigen::MatrixXf K; K.setOnes(7, 7);
	M = conv2(M, K);
	M = conv2(M, K);

	glWindowPos2i(0, 0); 
	glDrawPixels(W(), H(), GL_RED, GL_FLOAT, M.data());
	*/

