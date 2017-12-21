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
#include "OptimizationLib/GradientDescentFunctionMinimizer.h"
#include "OptimizationLib/BFGSFunctionMinimizer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "RotationMount.h"
#include "MountedPointSpring2D.h"

#define EDIT_BOUNDARY_CONDITIONS

//#define CONSTRAINED_DYNAMICS_DEMO

BenderApp::BenderApp() 
{
	setWindowTitle("Test FEM Sim Application...");

	int nRows = 30;
	int nCols = 7;
	double length = 2.0;
	double height = 0.1;
	BenderSimulationMesh2D::generateSquareTriMesh("../data/FEM/2d/triMeshTMP.tri2d", -length/2.0, 0, length/(nRows-1), height/(nCols-1), nRows, nCols);

	femMesh = new BenderSimulationMesh2D();
	femMesh->readMeshFromFile("../data/FEM/2d/triMeshTMP.tri2d");
	//femMesh->addGravityForces(V3D(0, -9.8, 0));
	femMesh->addGravityForces(V3D(0, 0, 0));


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

	// menu

	mainMenu->addGroup("FEM Sim options");
	mainMenu->addVariable("Static solve", computeStaticSolution);
	mainMenu->addVariable("Optimize Objective", optimizeObjective);
	mainMenu->addVariable("Check derivatives", checkDerivatives);
	mainMenu->addVariable("Approx. line search", approxLineSearch);
	mainMenu->addButton("set state as target", [this](){
		                                             femMesh->setNodeGlobalNodePositionObjective(femMesh->x);
	                                                 });
	
	initInteractionMenu(mainMenu);

	menuScreen->performLayout();


	//set two mounts with pins
	addRotationMount();
	addRotationMount();
	for (int i = 0; i < nCols; ++i) {
		addMountedNode(i, 0);
		addMountedNode((nRows-1)*nCols + i, 1);
	}
	pullXi();


	// prepare minimizer and objective funciton
	//minimizer = new GradientDescentFunctionMinimizer();
	
	minimizers.push_back(new GradientDescentFunctionMinimizer(maxIterations, solveResidual, maxLineSearchIterations, false));
	minimizers.push_back(new BFGSFunctionMinimizer           (maxIterations, solveResidual, maxLineSearchIterations, false));

	minimizer = minimizers[comboBoxOptimizationAlgorithm->selectedIndex()];
	
	objectiveFunction = new NodePositionObjectiveFunction(this);

}

BenderApp::~BenderApp()
{
}

void BenderApp::initInteractionMenu(nanogui::FormHelper* menu)
{

	// add selection of optimization algorithm
	menu->addGroup("Optimization");
	{
		nanogui::Widget *selection = new nanogui::Widget(menu->window());
		menu->addWidget("", selection);
		selection->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
			nanogui::Alignment::Middle, 0, 4));
		comboBoxOptimizationAlgorithm = new nanogui::ComboBox(selection, { "gradient descent", "quasi Newton: BFGS"});
		comboBoxOptimizationAlgorithm->setCallback([this](int idx){minimizer = minimizers[idx]; });
		comboBoxOptimizationAlgorithm->setSelectedIndex(1);


		menu->addVariable("max Iterations", maxIterations);
		menu->addVariable("solve residual", solveResidual);
		menu->addVariable("max linesearch iter", maxLineSearchIterations);
	}

	
	// add selection for interaction mode
	 menu->addGroup("Mode");
	{
		nanogui::Widget *modes = new nanogui::Widget(menu->window());
		menu->addWidget("", modes);
		modes->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 4));

		buttonsInteractionMode[0] = new nanogui::Button(modes, "View");
		buttonsInteractionMode[0]->setFlags(nanogui::Button::RadioButton);
		buttonsInteractionMode[0]->setCallback([this](){switchInteractionMode(InteractionMode::VIEW);});
		buttonsInteractionMode[0]->setPushed(true);
		switchInteractionMode(InteractionMode::VIEW);
		buttonsInteractionMode[1] = new nanogui::Button(modes, "Select");
		buttonsInteractionMode[1]->setFlags(nanogui::Button::RadioButton);
		buttonsInteractionMode[1]->setCallback([this](){switchInteractionMode(InteractionMode::SELECT);});
		buttonsInteractionMode[2] = new nanogui::Button(modes, "Drag");
		buttonsInteractionMode[2]->setFlags(nanogui::Button::RadioButton);
		buttonsInteractionMode[2]->setCallback([this](){switchInteractionMode(InteractionMode::DRAG);});
		buttonsInteractionMode[3] = new nanogui::Button(modes, "Edit");
		buttonsInteractionMode[3]->setFlags(nanogui::Button::RadioButton);
		buttonsInteractionMode[3]->setCallback([this](){switchInteractionMode(InteractionMode::DRAW);});
	}

	// add combo box for selected mount/handle
	menu->addGroup("Mounts");
	{
		nanogui::Widget *selection = new nanogui::Widget(menu->window());
		menu->addWidget("", selection);
		selection->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
			nanogui::Alignment::Middle, 0, 4));
		comboBoxMountSelection = new nanogui::ComboBox(selection, { "Combo box item 1", "Combo box item 2", "Combo box item 3"});
		comboBoxMountSelection->setCallback([this](int idx){selected_mount = idx;
		                                                    std::cout << "selected mount: " << selected_mount << std::endl;
															});
	}
	{
		nanogui::Widget *mountStatus = new nanogui::Widget(menu->window());
		menu->addWidget("", mountStatus);
		mountStatus->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
			nanogui::Alignment::Middle, 0, 4));
		nanogui::Button *b;
		b = new nanogui::Button(mountStatus, "toogle xi");
		b->setCallback([this](){femMesh->mounts[selected_mount]->parameterOptimization ^= true;
		                        updateMountSelectionBox();
								});
		b = new nanogui::Button(mountStatus, "toogle active");
		b->setCallback([this](){femMesh->mounts[selected_mount]->active ^= true;
								updateMountSelectionBox();
								});
	}
	{
		nanogui::Widget *mountAdd = new nanogui::Widget(menu->window());
		menu->addWidget("", mountAdd);
		mountAdd->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
			nanogui::Alignment::Middle, 0, 4));
		nanogui::Button *b;
		b = new nanogui::Button(mountAdd, "add mount");
		b->setCallback([this](){addRotationMount();});
		b = new nanogui::Button(mountAdd, "remove mount");
		b->setCallback([this](){removeSelectedMount();});
	}

	menu->addGroup("Tools");
	menu->addVariable("Add/remove nodes", toolMode, true) -> setItems({"pick single node", "brush"});
	
};

void BenderApp::updateMountSelectionBox()
{
	std::vector<std::string> items(femMesh->mounts.size());
	for(int i = 0; i < femMesh->mounts.size(); ++i) {
		std::stringstream item;
		item << "Mount " << i << " | ";
		if(femMesh->mounts[i]->parameterOptimization) {
			item << "xi";
		}
		else {
			item << "  ";
		}
		item << " | ";
		if(femMesh->mounts[i]->active) {
			item << "active";
		}
		else {
			item << "      ";
		}
		item << " |";

		items[i] = item.str();
	}

	comboBoxMountSelection->setItems(items);
	selected_mount = comboBoxMountSelection->selectedIndex();

	menuScreen->performLayout();
}


void BenderApp::switchInteractionMode(InteractionMode mode)
{
	if(mode == InteractionMode::DRAG) {
		if(interactionMode != InteractionMode::DRAG) {
			selectedNodeID = -1;
		}
	}

	interactionMode = mode;
}

void BenderApp::setSelectedMount(int mountID)
{
	if(mountID >= 0) {
		selected_mount = mountID;
		comboBoxMountSelection->setSelectedIndex(mountID);
	}
}


//triggered when mouse moves
bool BenderApp::onMouseMoveEvent(double xPos, double yPos) {
	//lastClickedRay = getRayFromScreenCoords(xPos, yPos);

	lastMovedRay = currentRay;
	currentRay = getRayFromScreenCoords(xPos, yPos);
	
	if(interactionMode == InteractionMode::VIEW) {
		return(GLApplication::onMouseMoveEvent(xPos, yPos));
	}
	else if(interactionMode == InteractionMode::SELECT) {

		return(true);
	}
	else if(interactionMode == InteractionMode::DRAG) {
		if (selectedNodeID != -1)
		{
			Plane plane(camera->getCameraTarget(),V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
			P3D targetPos;
			P3D lastPos;
			currentRay.getDistanceToPlane(plane,&targetPos);
			lastMovedRay.getDistanceToPlane(plane, &lastPos);
			V3D delta = targetPos - lastPos;
			dynamic_cast<RotationMount*>(femMesh->mounts[selected_mount])->shift(delta);
		}
		return(true);
	}
	else if(interactionMode == InteractionMode::DRAW) {

		return(true);
	}
	else {
		return(false);
	}



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
	
	
	return false;
}

//triggered when mouse buttons are pressed
bool BenderApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	
	
	if (button == GLFW_MOUSE_BUTTON_LEFT) {
		if(interactionMode == InteractionMode::VIEW) {
			return(GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos));
		}
		else if(interactionMode == InteractionMode::SELECT) {
			int selectedNodeID_temp = femMesh->getSelectedNodeID(lastMovedRay);
			int selectedMountID_temp = femMesh->getMountIdOfNode(selectedNodeID_temp);
			setSelectedMount(selectedMountID_temp);
			return(true);
		}
		else if(interactionMode == InteractionMode::DRAG) {
			if(action == GLFW_PRESS) {
				lastClickedRay = lastMovedRay;
				selectedNodeID = femMesh->getSelectedNodeID(lastClickedRay);
			}
			else {
				selectedNodeID = -1;
			}
			return(true);
		}
		else if(interactionMode == InteractionMode::DRAW) {
			if(toolMode == ToolMode::PICK_NODE) {
				int selectedNodeID_temp = femMesh->getSelectedNodeID(lastMovedRay);
				addMountedNode(selectedNodeID_temp, selected_mount);
			}
			return(true);
		}
		else {
			return(false);
		}


	}

	if (button == GLFW_MOUSE_BUTTON_RIGHT) {

		if(interactionMode == InteractionMode::VIEW) {
			return(GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos));
		}
		else if(interactionMode == InteractionMode::SELECT) {

			return(true);
		}
		else if(interactionMode == InteractionMode::DRAG) {

			return(true);
		}
		else if(interactionMode == InteractionMode::DRAW) {
			int selectedNodeID_temp = femMesh->getSelectedNodeID(lastMovedRay);
			unmountNode(selectedNodeID_temp, selected_mount);
			return(true);
		}
		else {
			return(false);
		}

	}

	return false;
}

//triggered when using the mouse wheel
bool BenderApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	
	if(interactionMode == InteractionMode::VIEW) {
		return(GLApplication::onMouseWheelScrollEvent(xOffset, yOffset));
	}
	else if(interactionMode == InteractionMode::SELECT) {
		return(true);
	}
	else if(interactionMode == InteractionMode::DRAG) {
		if (selected_mount >= 0) {
			std::cout << "Offsets: " << xOffset << " " << yOffset << std::endl;
			Plane plane(camera->getCameraTarget(),V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
			P3D origin; 
			currentRay.getDistanceToPlane(plane,&origin);
			dynamic_cast<RotationMount*>(femMesh->mounts[selected_mount])->rotate(origin, yOffset * 0.05);
		}
		return(true);
	}
	else if(interactionMode == InteractionMode::DRAW) {

		return(true);
	}
	else {
		return(false);
	}
}

bool BenderApp::onKeyEvent(int key, int action, int mods) {	
	
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool BenderApp::onCharacterPressedEvent(int key, int mods) {

	if (!mods) {
		/*
		if (key >= '0' && key <= '1') {
			int mount_id = key - '0';
			selected_mount = mount_id;
			std::cout << "selected mount: " << selected_mount << std::endl;
		}
		else if (key == 'x') {
			selected_mount = -1;
			std::cout << "no mount selected" << std::endl;
		}
		*/

		auto setInteractionModeAndButtons = [this](InteractionMode mode)
		{
			interactionMode = mode;
			for(auto b : buttonsInteractionMode) {
				b->setPushed(false);
			}
			buttonsInteractionMode[static_cast<int>(mode)]->setPushed(true);
		};

		if(key == 'v') {
			setInteractionModeAndButtons(InteractionMode::VIEW);
		}
		else if(key == 's') {
			setInteractionModeAndButtons(InteractionMode::SELECT);
		}
		else if(key == 'd') {
			setInteractionModeAndButtons(InteractionMode::DRAG);
		}
		else if(key == 'e') {
			setInteractionModeAndButtons(InteractionMode::DRAW);
		}
	}

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

/*
// Run the App tasks
void BenderApp::process() {
	//do the work here...

	//add mouse drag tempEEUnits
	//femMesh->tempEEUnits.clear();
	//femMesh->tempEEUnits.push_back(new CSTDrag2D(this, nodes[9], P3D(-5, 20)));
	//
	simulationTime = 0;
	maxRunningTime = 1.0 / desiredFrameRate;
	femMesh->checkDerivatives = checkDerivatives != 0;



	//if we still have time during this frame, or if we need to finish the physics step, do this until the simulation time reaches the desired value
	int counter = 0;
	while (simulationTime < 1.0 * maxRunningTime) {
		std::cout << "counter = " << counter++ << std::endl;
		

		if(optimizeObjective) {

			pullXi();

			// compute dO/dxi	[n_par x 1]
			computeDoDxi(dOdxi);
std::cout << "dodxi = " << dOdxi[0] << " " << dOdxi[1] << " " << dOdxi[2];
std::cout << std::endl;

			// compute gamma with linesearch	[scalar]
			// (TODO)
			double gamma;
			{
				auto O_formal = [&] (dVector const & xi) {
					return peekOofXi(xi);
				};

				auto O_approx = [&] (dVector const & xi) {
					dVector dxi = xi - BenderApp::xi;
					x_approx = femMesh->x;
					for(int i = 0; i < xi.size(); ++i) {
						for(int j = 0; j < x_approx.size(); ++j) {
							x_approx[j] += deltaxdeltaxi[i][j] * dxi[i];	// note: deltaxdeltaxi should have been computed before within 'computeDoDxi()'
						}
					}
					return(femMesh->computeOofx(x_approx));
				};

				auto f = [&] (dVector const & xi) { 					
					return(approxLineSearch ? O_approx(xi) : O_formal(xi));
				};

				
				double gamma_start = 0.2;
				int maxIter = 30;

				double gamma_test = gamma_start;
				dVector xi_old = xi;
				double f_init = femMesh->computeO();
				//double f_init = f(xi_old);
				//dVector search_direction = dOdxi;

				dVector xi_new;
				dVector xi_test;

				for(int j = 0; j < maxIter; ++j) {
					xi_test = xi_old - gamma_test * dOdxi;
					double f_new = f(xi_test);

					std::cout << "xi_old   " << xi_old[0] << " " << xi_old[1] << " " << xi_old[2] << std::endl;
					std::cout << "xi_test  " << xi_test[0] << " " << xi_test[1] << " " << xi_test[2] << std::endl;
					std::cout << "j " << j << "  gamma: " << gamma_test << "  o_init = " << f_init << "  o_probe = " << f_new << std::endl;

					if(!isfinite(f_new)) {
						gamma_test /= 2.0;
					}
					else if(f_new >= f_init && j <= maxIter) {
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
			xi = xi - 0.5*gamma * dOdxi;
			//xi = xi - gamma * dOdxi;
			pushXi();

			solveMesh();

			double o_new = femMesh->computeO();
			double e_new = femMesh->computeTargetPositionError();
			double delta_o = o_new - o_last;
			double delta_e = e_new - e_last;
			Logger::consolePrint("o | e | delta o | delta e = %10f | %10f | %+-10.7f | %+-10.7f \n", o_new, e_new, delta_o, delta_e);
			o_last = o_new;
			e_last = e_new;

		}
		else {
			solveMesh();
		}



	}
}
*/
// Run the App tasks
void BenderApp::process() {
	//do the work here...

	simulationTime = 0;
	maxRunningTime = 1.0 / desiredFrameRate;
	femMesh->checkDerivatives = checkDerivatives != 0;


	//if we still have time during this frame, or if we need to finish the physics step, do this until the simulation time reaches the desired value
	while (simulationTime < 1.0 * maxRunningTime) {

		if(optimizeObjective) {

			pullXi();

			// minimize here
			minimizer->lineSearchStartValue = 0.1;
			minimizer->maxIterations = maxIterations;
			minimizer->solveResidual = solveResidual;
			minimizer->maxLineSearchIterations = maxLineSearchIterations;
			
			double o_new = 0;
			minimizer->minimize(objectiveFunction, xi, o_new);

			double e_new = femMesh->computeTargetPositionError();
			double delta_o = o_new - o_last;
			double delta_e = e_new - e_last;
			Logger::consolePrint("o | e | delta o | delta e = %10f | %10f | %+-10.7f | %+-10.7f \n", o_new, e_new, delta_o, delta_e);
			o_last = o_new;
			e_last = e_new;

		}
		else {
			solveMesh();
		}

	}
}

void BenderApp::solveMesh() 
{
	if (computeStaticSolution)
	{
		femMesh->solve_statics();
		simulationTime += maxRunningTime + 1.0;
	}
	else {
		femMesh->solve_dynamics(simTimeStep);
		simulationTime += simTimeStep;
	}
}



void BenderApp::pullXi()
{
	int n_parameters = 0;
	for(Mount const * m: femMesh->mounts) {
		if(m->active && m->parameterOptimization) {
			n_parameters += m->parameters.size();
		}
	}
	xi.resize(n_parameters);
	int i = 0;
	for(Mount * m: femMesh->mounts) {
		if(m->active && m->parameterOptimization) {
			m->parametersStartIndex = i;
			for(double p : m->parameters) {
				xi[i++] = p;
			}
		}
	}
}

void BenderApp::pushXi()
{
	for(Mount * m: femMesh->mounts) {
		if(m->active && m->parameterOptimization) {
			int i = 0;
			for(double & p : m->parameters) {
				p = xi[m->parametersStartIndex + (i++)];
			}
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

	//double regularizer_temp = dynamic_cast<FEMEnergyFunction *>(femMesh->energyFunction)->regularizer;
	femMesh->energyFunction->setToStaticsMode(0.0);
	femMesh->energyFunction->addHessianEntriesTo(hessianEntries, femMesh->x);
	femMesh->energyFunction->setToStaticsMode(0.01);

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
		deltaxdeltaxi[i] = solver.solve(-deltaFdeltaxi[i]);
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
	dVector m_temp = femMesh->m;
	dVector f_ext_temp = femMesh->f_ext;
	dVector xSolver_temp = femMesh->xSolver;

	// store current parameters xi
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
	femMesh->m = m_temp;
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

	// draw nodes of mounted points
	for(int i = 0; i < femMesh->pinnedNodeElements.size(); ++i) {
		MountedPointSpring2D * mp = static_cast<MountedPointSpring2D *>(femMesh->pinnedNodeElements[i]);

		double size = 0.005;
		P3D color(0.5, 0.0, 1.0);
		P3D white(1.0, 1.0, 1.0);
		P3D red(1.0, 1.0, 1.0);
		if(!mp->mount->parameterOptimization) {
			color = P3D(0.0, 0.0, 0.8);
		}
		if(!mp->mount->active) {
			color = color*0.5 + red*0.5;
		}
		if(mp->mount == femMesh->mounts[selected_mount]) {
			size *= 1.8;
		}

		mp->draw(femMesh->x, size, color(0), color(1), color(2));
	}

	// draw origin
	P3D p0(0.0, 0.0, 0.0);
	double l = 0.2;
	P3D px(l, 0.0, 0.0);
	P3D py(0.0, l, 0.0);
	P3D pz(0.0, 0.0, l);
	glColor3d(0.8, 0, 0);
	glBegin(GL_LINES);
	glVertex3d(p0[0], p0[1], p0[2]);
	glVertex3d(px[0], px[1], px[2]);
	glEnd();
	glColor3d(0, 0.8, 0);
	glBegin(GL_LINES);
	glVertex3d(p0[0], p0[1], p0[2]);
	glVertex3d(py[0], py[1], py[2]);
	glEnd();
	glColor3d(0, 0, 0.8);
	glBegin(GL_LINES);
	glVertex3d(p0[0], p0[1], p0[2]);
	glVertex3d(pz[0], pz[1], pz[2]);
	glEnd();
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void BenderApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void BenderApp::restart() {

}

void BenderApp::addRotationMount() 
{
	femMesh->addRotationMount();
	pullXi();
	updateMountSelectionBox();
}

void BenderApp::removeSelectedMount()
{
	femMesh->removeMount(selected_mount);
	pullXi();
	updateMountSelectionBox();
}


void BenderApp::addMountedNode(int node_id, int mount_id)
{
	if(node_id < 0) {return;}
	if(mount_id < 0) {return;}

	femMesh->setMountedNode(node_id, femMesh->nodes[node_id]->getCoordinates(femMesh->X), mount_id);
	//std::cout << "pinned nodes are: ";
	//for(BaseEnergyUnit* np : femMesh->pinnedNodeElements) {
	//	std::cout << dynamic_cast<MountedPointSpring2D*>(np)->node->nodeIndex << " ";
	//}
	//std::cout << std::endl;
}

void BenderApp::unmountNode(int node_id, int mount_id)
{
	if(node_id < 0) {return;}
	if(mount_id < 0) {return;}

	femMesh->unmountNode(node_id, mount_id);
}


bool BenderApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}
