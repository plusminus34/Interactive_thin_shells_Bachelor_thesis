#include <GUILib/GLUtils.h>
#include "BenderApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <MathLib/Plane.h>
#include <FEMSimLib/CSTSimulationMesh2D.h>
#include <FEMSimLib/CSTSimulationMesh3D.h>
#include <FEMSimLib/MassSpringSimulationMesh3D.h>
#include <GUILib/GLUtils.h>

#include <algorithm>
#include <cmath>

#include "RotationMount.h"
#include "MountedPointSpring2D.h"

#define EDIT_BOUNDARY_CONDITIONS

//#define CONSTRAINED_DYNAMICS_DEMO

BenderApp::BenderApp() 
{
	setWindowTitle("Test FEM Sim Application...");

	int nRows = 40;
	int nCols = 8;
	BenderSimulationMesh2D::generateSquareTriMesh("../data/FEM/2d/triMeshTMP.tri2d", -1, 0, 0.05, 0.05, nRows, nCols);

	femMesh = new BenderSimulationMesh2D();
	femMesh->readMeshFromFile("../data/FEM/2d/triMeshTMP.tri2d");
	//femMesh->addGravityForces(V3D(0, -9.8, 0));
	femMesh->addGravityForces(V3D(0, 0, 0));

	// initialize the parameter vector
	pullXi();

	//set two mounts with pins
	for (int i = 0; i < nCols; ++i) {
		addMountedNode(i, 0);
		addMountedNode((nRows-1)*nCols + i, 1);
	}

	// set some curve to "uppermost" column
	{
		auto curve_normalized = [](double z) -> double 
		{
			return(std::sin(z*PI));
		};

		auto curve = [curve_normalized](double z, double lb, double ub) -> double
		{
			return(curve_normalized((z-lb)/(ub-lb)) * (ub-lb));
		};

		double lb = femMesh->nodes[0*nCols + 0]->getWorldPosition().at(0);
		double ub = femMesh->nodes[(nRows-1)*nCols + 0]->getWorldPosition().at(0);
		for(int i = 0; i < nRows; ++i) {
			int id = i*nCols + 0;
			P3D pt = femMesh->nodes[id]->getWorldPosition();
			//V3D d(-pt.at(0)*0.2, curve(pt.at(0), lb, ub)*0.2 , 0);
			V3D d(0, curve(pt.at(0), lb, ub)*0.2 , 0);
			//V3D d(0, 0.2 , 0);
			femMesh->setNodePositionObjective(id, pt + d);
		}
	}


	showGroundPlane = false;

	mainMenu->addGroup("FEM Sim options");
	mainMenu->addVariable("Static solve", computeStaticSolution);
	mainMenu->addVariable("Optimize Objective", optimizeObjective);
	mainMenu->addVariable("Check derivatives", checkDerivatives);
	menuScreen->performLayout();


}

BenderApp::~BenderApp()
{
}

//triggered when mouse moves
bool BenderApp::onMouseMoveEvent(double xPos, double yPos) {
	//lastClickedRay = getRayFromScreenCoords(xPos, yPos);

	lastMovedRay = currentRay;
	currentRay = getRayFromScreenCoords(xPos, yPos);
	

	if (selectedNodeID != -1){
		int selectedMountID = selected_mount;
		if (selectedMountID < 0) {
			//Plane plane(camera->getCameraTarget(),V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
			//P3D targetPinPos; 
			//getRayFromScreenCoords(xPos,yPos).getDistanceToPlane(plane,&targetPinPos);
			//femMesh->setPinnedNode(selectedNodeID,targetPinPos);
			//return true;
		}
		else {
			Plane plane(camera->getCameraTarget(),V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
			P3D targetPos;
			P3D lastPos;
			currentRay.getDistanceToPlane(plane,&targetPos);
			lastMovedRay.getDistanceToPlane(plane, &lastPos);
			V3D delta = targetPos - lastPos;
			//V3D delta = targetPos - femMesh->nodes[selectedNodeID]->getWorldPosition();
			dynamic_cast<RotationMount*>(femMesh->mounts[selectedMountID])->shift(delta);
			//updateMountEnergy();
			return true;
		}
	}
	
	else if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;
	return false;
}

//triggered when mouse buttons are pressed
bool BenderApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
#ifdef EDIT_BOUNDARY_CONDITIONS

	if (button == GLFW_MOUSE_BUTTON_RIGHT) {
		if (action == GLFW_PRESS && mods & GLFW_MOD_CONTROL) {
			femMesh->removePinnedNodeConstraints();
			for (Mount * m : femMesh->mounts) {
				m->reset();
			}
		}
	}

	if (button == GLFW_MOUSE_BUTTON_LEFT){
		lastClickedRay = lastMovedRay;
		if (action == GLFW_PRESS && mods & GLFW_MOD_CONTROL) {
			int selectedNodeID_temp = femMesh->getSelectedNodeID(lastClickedRay);
			if (selectedNodeID_temp >= 0) {
				if (selected_mount >= 0 && selected_mount <= 2) {
					addMountedNode(selectedNodeID_temp, selected_mount);
				}
				return true;
			}
		}
		else if (action == GLFW_PRESS && mods & GLFW_MOD_SHIFT) {
			selectedNodeID = femMesh->getSelectedNodeID(lastClickedRay);
			//if (selectedNodeID >= 0) {
			//	if (selected_mount >= 0 && selected_mount <= 9) {
			//		addMountedNode(selectedNodeID, selected_mount);
			//	}
			//	return true;
			//}
		}

		else {
			selectedNodeID = -1;
		}
	}
#endif

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

//	Logger::consolePrint("--> %d\n", selectedNodeID);
//	Logger::consolePrint("--> %lf %lf %lf\n", lastClickedRay.origin[0], lastClickedRay.origin[1], lastClickedRay.origin[2]);
//	Logger::consolePrint("--> %lf %lf %lf\n", lastClickedRay.direction[0], lastClickedRay.direction[1], lastClickedRay.direction[2]);
	return false;
}

//triggered when using the mouse wheel
bool BenderApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	
	if (selected_mount >= 0) {
		std::cout << "Offsets: " << xOffset << " " << yOffset << std::endl;
		Plane plane(camera->getCameraTarget(),V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
		P3D origin; 
		currentRay.getDistanceToPlane(plane,&origin);
		dynamic_cast<RotationMount*>(femMesh->mounts[selected_mount])->rotate(origin, yOffset * 0.05);
		return(true);
	}
	
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool BenderApp::onKeyEvent(int key, int action, int mods) {	
	
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool BenderApp::onCharacterPressedEvent(int key, int mods) {
#ifdef EDIT_BOUNDARY_CONDITIONS
	if (!mods) {
		if (key >= '0' && key <= '1') {
			int mount_id = key - '0';
			selected_mount = mount_id;
			std::cout << "selected mount: " << selected_mount << std::endl;
		}
		else if (key == 'x') {
			selected_mount = -1;
			std::cout << "no mount selected" << std::endl;
		}
	}
#endif	
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

	return false;
}


void BenderApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);
	if (fNameExt == "tri2d") {
		delete femMesh;
		femMesh = new BenderSimulationMesh2D;
		femMesh->readMeshFromFile(fName);
		Logger::consolePrint("...Done!");
	} else if (fNameExt == "obj") {
		std::cerr << "Error: functionality not implemented. (" << __FILE__ << ":" << __LINE__ << ")" << std::endl;
		exit(1);
		//delete femMesh;
		//femMesh = new CSTSimulationMesh3D;
		//femMesh->readMeshFromFile(fName);
		//Logger::consolePrint("...Done!");
	} else {
		Logger::consolePrint("...but how to do with that?");
	}

}

void BenderApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}


// Run the App tasks
void BenderApp::process() {
	//do the work here...

	//add mouse drag tempEEUnits
	//femMesh->tempEEUnits.clear();
	//femMesh->tempEEUnits.push_back(new CSTDrag2D(this, nodes[9], P3D(-5, 20)));
	//
	double simulationTime = 0;
	double maxRunningTime = 1.0 / desiredFrameRate;
	femMesh->checkDerivatives = checkDerivatives != 0;


	auto solve_mesh = [&] ()
	{
		if (computeStaticSolution)
			femMesh->solve_statics();
		else {
			femMesh->solve_dynamics(simTimeStep);
		}
	};


	//if we still have time during this frame, or if we need to finish the physics step, do this until the simulation time reaches the desired value
	while (simulationTime < 1.0 * maxRunningTime) {

		simulationTime += simTimeStep;

		if(optimizeObjective) {

			// compute dO/dxi	[n_par x 1]
			computeDoDxi(dOdxi);

			// compute gamma with linesearch	[scalar]
			// (TODO)
			double gamma;
			{
				auto O_formal = [&] (dVector const & xi) {
					return peekOofXi(xi);
				};

				auto f = [&] (dVector const & xi) {
					return(O_formal(xi));
				};

				
				double gamma_start = 1.0;
				int maxIter = 10;


				//double gamma_old = gamma_start;
				double gamma_test = gamma_start;
				dVector xi_old = xi;
				double f_init = femMesh->computeO();
				dVector search_direction = dOdxi;

				dVector xi_new;
				dVector xi_test;

				for(int j = 0; j < maxIter; ++j) {
					xi_test = xi_old - gamma_test * search_direction;
					double f_new = f(xi_test);

					if(!isfinite(f_new)) {
						gamma_test /= 2.0;
					}
					else if(f_new > f_init && j <= maxIter) {
						gamma_test /= 2.0;
					}
					else {
						break;
					}
				}
				gamma = gamma_test;
			}

			// update xi
			// (TODO) xi = xi - gamma * dO/dxi
			xi = xi - gamma * dOdxi;
			pushXi();

			solve_mesh();
		}
		else {
			solve_mesh();
		}
	}
}



void BenderApp::pullXi()
{
	/*
	xi.resize(0);
	for(Mount const * m: femMesh->mounts) {
		xi.insert(xi.end(), m->parameters.begin(), m->parameters.end());
	}
	*/
	int n_parameters = 0;
	for(Mount const * m: femMesh->mounts) {
		n_parameters += m->parameters.size();
	}
	xi.resize(n_parameters);
	int i = 0;
	for(Mount const * m: femMesh->mounts) {
		for(double p : m->parameters) {
			xi[i++] = p;
		}
	}
}

void BenderApp::pushXi()
{
	/*
	int i = 0;
	for(Mount const * m: femMesh->mounts) {
		int n_par = m->parameters.size();
		std::copy(xi.begin()+i; xi.begin()+i+n_par, m->parameters.begin());
		i += n_par;
	}
	*/
	int i = 0;
	for(Mount * m: femMesh->mounts) {
		for(double & p : m->parameters) {
			p = xi[i++];
		}
	}
}


void BenderApp::computeDoDxi(dVector & dodxi)
{
	dodxi.resize(xi.size());

	// compute dO/dx [length(x) x 1]
	femMesh->computeDoDx(dOdx);

	// compute dF/dxi [length(x) x xi]
	deltaFdeltaxi.resize(xi.size());
	for(int i = 0; i < xi.size(); ++i) {
		deltaFdeltaxi[i].resize(femMesh->x.size());
		deltaFdeltaxi[i].setZero();
	}
	for(BaseEnergyUnit* pin : femMesh->pinnedNodeElements) {
		dynamic_cast<MountedPointSpring2D*>(pin)->addDeltaFDeltaXi(deltaFdeltaxi);
	}

	// get dF/dx  (Hessian from FEM simulation)  [lengh(x) x length(x)]
	SparseMatrix H(femMesh->x.size(), femMesh->x.size());
	DynamicArray<MTriplet> hessianEntries(0);
	femMesh->energyFunction->addHessianEntriesTo(hessianEntries, femMesh->x);

	H.setFromTriplets(hessianEntries.begin(), hessianEntries.end());
	
	// solve dF/dx * y = dF/dxi		(y is dx/dxi)
	Eigen::SimplicialLDLT<SparseMatrix> solver;
	H *= -1.0;
	solver.compute(H);
	if (solver.info() != Eigen::Success) {
		std::cerr << "Eigen::SimplicialLDLT decomposition failed." << std::endl;
		exit(1);
	}
	// solve for each parameter xi
	deltaxdeltaxi.resize(xi.size());
	for(int i = 0; i < xi.size(); ++i) {
		deltaxdeltaxi[i] = solver.solve(deltaFdeltaxi[i]);
	}

	// do/dxi = do/dx * dx/dxi
	for(int i = 0; i < xi.size(); ++i) {
		dodxi[i] = dOdx.transpose() * deltaxdeltaxi[i];
	}

}



double BenderApp::peekOofXi(dVector const & xi_in) {

	// store the current state of the mesh
	dVector x_temp = femMesh->x;
	dVector v_temp = femMesh->v;
	//dVector m_temp = femMesh->m;
	dVector f_ext_temp = femMesh->f_ext;
	dVector xSolver_temp = femMesh->xSolver;

	// store curent parameters xi
	dVector xi_temp = xi;

	// new parameters
	xi = xi_in;
	pushXi();
	femMesh->solve_statics();
	double O = femMesh->computeO();

	// set mesh to old state
	xi = xi_temp;
	pushXi();
	femMesh->x = x_temp;
	femMesh->v = v_temp;
	//femMesh->m = m_temp;
	femMesh->f_ext = f_ext_temp;
	femMesh->xSolver = xSolver_temp;

	return(O);
}



// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void BenderApp::drawScene() {
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glColor3d(1,1,1);
	femMesh->drawSimulationMesh();

#ifdef CONSTRAINED_DYNAMICS_DEMO
	glColor3d(1, 1, 1);
	drawCircle(0, 0, 1, 100);
#endif
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void BenderApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void BenderApp::restart() {

}


void BenderApp::addMountedNode(int node_id, int mount_id)
{
	femMesh->setMountedNode(node_id, femMesh->nodes[node_id]->getWorldPosition(), mount_id);
	std::cout << "pinned nodes are: ";
	for(BaseEnergyUnit* np : femMesh->pinnedNodeElements) {
		std::cout << dynamic_cast<MountedPointSpring2D*>(np)->node->nodeIndex << " ";
	}
	std::cout << std::endl;
}


bool BenderApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}
