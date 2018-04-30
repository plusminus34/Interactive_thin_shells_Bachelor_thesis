#include "AppXD.h"
#include <PlushHelpers\helpers_star.h> // NOTE: When in doubt, include this. 

// This is the (working version of the) demo app for Plush.
// This app shows how to make 2D and 3D meshes, and integrate them forward in time.
// Useful (though mostly non-essential) methods can be found in
// the parent class PlushApplication.h and the helpers library PlushHelpers/*.

// Software is provided as is, but please send any strange bugs to:
// -- // James Bern (jamesmbern@gmail.com)

AppXD::AppXD() {
	setWindowTitle("AppXD");

	// -- // Camera

	// NOTE: Uncomment this to disable 3D camera.
	// SPOOF_2D_CAMERA = true;

	// NOTE: Mess with these to change the default camera params.
	// ----- You can reset the camera by pressing 'c' and print out the current
	// ----- camera params with 'C'
	// DEFAULT_CAM_ROT_ABOUT_RIGHT_AXIS = 0.;
	// DEFAULT_CAM_ROT_ABOUT_UP_AXIS___ = 0.;
	// DEFAULT_CAM_DISTANCE____________ = -3.;
	// DEFAULT_CAM_TARGET______________ = P3D();

	// -- // Basic 2D Construction

	// NOTE: spawnSavedMesh(...) loads the node positions, elements, and tendons
	// NOTE: nudge_mesh(...) can be used to reposition the mesh
	// ----- This must be done before pinning.
	// NOTE: applyYoungsModulusAndPoissonsRatio(E, nu) sets the material parameters
	// ----- for the Neo Hookean model.
	// ----- For polyurethane foams, a reasonable choice of E is
	// ----- around 3e4 for coarse meshes, and 3e5 for dense meshes.
	// ----- nu doesn't seem to matter as much
	// NOTE: The pin*() methods add zero length springs to e.g. the top of the mesh.
	tri_mesh = new CSTSimulationMesh2D();
	tri_mesh->spawnSavedMesh("../Apps/Plush/data/tri/T");
	tri_mesh->nudge_mesh(-V3D(.75, 0., 0.));
	tri_mesh->addGravityForces(V3D(0., -10., 0.));
	tri_mesh->applyYoungsModulusAndPoissonsRatio(1e4, .25);
	tri_mesh->pinToCeiling(); 
 
	// -- // Simple Tendon Construction

	// NOTE: Uncomment this to start playing with tendons.
	// ----- This code adds a tendon between nodes 0 and N - 1
	// ----- then sets its contraction alphac to half its assembly length alphaz;
	// int N = tri_mesh->nodes.size();
	// tri_mesh->add_tendon_from_vecInt({ 0, N - 1 });
	// auto tendon_ptr = tri_mesh->tendons[0];
	// tendon_ptr->set_alphac_over_alphaz(.5);

	// -- // Basic 3D Construction

	tet_mesh = new CSTSimulationMesh3D();
	tet_mesh->spawnSavedMesh("../Apps/Plush/data/tet/brick");
	tet_mesh->nudge_mesh(V3D(.75, 0., 0.));
	tet_mesh->addGravityForces(V3D(0., -100., 0.));
	tet_mesh->applyYoungsModulusAndPoissonsRatio(1e4, .25);
	tet_mesh->pinToCeiling(); 

	// -- // Basic nanogui

	// NOTE: This is "simple mode" style to add variables to nanogui.
	// NOTE: DRAW_NODAL_FORCES is a useful tool for debugging.
	mainMenu->addGroup("app");
	mainMenu->addVariable("Draw F2D", tri_mesh->DRAW_NODAL_FORCES);
	mainMenu->addVariable("Draw F3D", tet_mesh->DRAW_NODAL_FORCES);
	mainMenu->addVariable("Transparent 3D", tet_mesh->DRAW_BOUNDARY_TRANSPARENT);
	menuScreen->performLayout(); 
}

void AppXD::drawScene() { 
	// NOTE: This increments PlushApplication::t,
	// ----- draws the handlers, and flashes errors spawned with PlushHelpers/error.h
	PlushApplication::drawScene();

	// NOTE: To draw a mesh, call mesh->draw(x, alphac, x0);
	// ----- This draws the mesh at position x
	// ----- Draws tendons according to alphac (this only affects color)
	// ----- Draws nodal forces according to x0 (this only affects contacts)
	// NOTE: The default mesh->draw() will use mesh->x, mesh->balphac, and mesh->x0
	tri_mesh->draw();
	tet_mesh->draw();

	// NOTE: Uncomment this method to draw a 2D floor.
	// draw_floor2d(); 
}

void AppXD::process() {
	// NOTE: Updating the position of a mesh is done in two stages.
	// ----- mesh->solve_statics(...) or mesh->solve_dynamics(...) 
	// ----- return a std::pair<dVector, dVector> which is the new x and v respectively.
	// ----- For statics v will always be 0.
	// ----- mesh->xvPair_INTO_Mesh(xv) just stores x and v on the mesh
	// ----- i.e. sets mesh->x and mesh->v
	tri_mesh->xvPair_INTO_Mesh(tri_mesh->solve_dynamics());
	tet_mesh->xvPair_INTO_Mesh(tet_mesh->solve_dynamics());
}

bool AppXD::onMouseMoveEvent(double xPos, double yPos) {
    return(PlushApplication::onMouseMoveEvent(xPos, yPos));
}

bool AppXD::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
    return(PlushApplication::onMouseButtonEvent(button, action, mods, xPos, yPos));
}

bool AppXD::onMouseWheelScrollEvent(double xOffset, double yOffset) {
    return(PlushApplication::onMouseWheelScrollEvent(xOffset, yOffset));
}

bool AppXD::onKeyEvent(int key, int action, int mods) {	
	return PlushApplication::onKeyEvent(key, action, mods);
}

bool AppXD::onCharacterPressedEvent(int key, int mods) {
	return PlushApplication::onCharacterPressedEvent(key, mods);
}

bool AppXD::processCommandLine(const std::string& cmdLine) {
	return PlushApplication::processCommandLine(cmdLine);
}
