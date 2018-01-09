#include <GUILib/GLUtils.h>

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

#include "RotationMount2D.h"
#include "MountedPointSpring.h"
#include "MatchScaledTrajObjective.h"

#include "BenderApp3D.h"

#define EDIT_BOUNDARY_CONDITIONS

//#define CONSTRAINED_DYNAMICS_DEMO

BenderApp3D::BenderApp3D() 
{
	setWindowTitle("Test FEM Sim Application...");





	/*
	int nRows = 24;
	int nCols = 6;
	double length = 2.0;
	double height = 0.1;
	CSTSimulationMesh2D::generateSquareTriMesh("../data/FEM/2d/triMeshTMP.tri2d", -length/2.0, 0, length/(nRows-1), height/(nCols-1), nRows, nCols);

	femMesh = new BenderSimulationMesh<2>();
	femMesh->readMeshFromFile("../data/FEM/2d/triMeshTMP.tri2d");
	//femMesh->addGravityForces(V3D(0, -9.8, 0));
	
	*/

	//femMesh = new BenderSimulationMesh<3>();
	//femMesh->readMeshFromFile("../data/3dModels/cylinder_vertical_1x20.obj");
	//loadFile("../data/3dModels/cylinder_vertical_1x20.obj");

	//loadFile("../data/3dModels/extruded_polygon_1x20.obj");
	//loadFile("../data/3dModels/cylinder_vertical_1x20.obj");
	loadFile("../data/3dModels/extruded_polygon_0p1x2.ply");

	femMesh->addGravityForces(V3D(0, 0, 0));



	/*

	// draw some target trjectory
	targetTrajectory_input.addKnotBack(P3D(-1.0, 0.2, 0.0));
	targetTrajectory_input.addKnotBack(P3D( 0.0, 0.4, 0.0));
	targetTrajectory_input.addKnotBack(P3D( 1.0, 0.2, 0.0));
	
	

	// add a fiber in Mesh to match
	DynamicArray<Node *> matchedFiber(0);
	for(int i = 0; i < nRows; ++i) {
		int id = i*nCols + nCols-1;
		matchedFiber.push_back(femMesh->nodes[id]);
	}

	// add a "MatchScaledTrajObjective"
	targetTrajectory_input.setTValueToLength();
	femMesh->objectives.push_back(new MatchScaledTrajObjective(matchedFiber, targetTrajectory_input));


	*/

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
	/*
	initInteractionMenu(mainMenu);
	*/
	menuScreen->performLayout();

	/*
	// initialize minimization algorithms
	minimizers.push_back(new GradientDescentFunctionMinimizer(maxIterations, solveResidual, maxLineSearchIterations, false));
	minimizers.push_back(new BFGSFunctionMinimizer           (maxIterations, solveResidual, maxLineSearchIterations, false));

	// initialize the ID Solver
	inverseDeformationSolver = new InverseDeformationSolver<2>(femMesh, minimizers[comboBoxOptimizationAlgorithm->selectedIndex()]);


	//set two mounts with pins
	addRotationMount();
	addRotationMount();
	for (int i = 0; i < nCols; ++i) {
		addMountedNode(i, 0);
		addMountedNode((nRows-1)*nCols + i, 1);
	}
	inverseDeformationSolver->pullXi();
	*/
	

}


BenderApp3D::~BenderApp3D()
{

}


void BenderApp3D::pushInputTrajectory(Trajectory3Dplus & trajInput) 
{
	trajInput.setTValueToLength();
	dynamic_cast<MatchScaledTrajObjective *>(femMesh->objectives[0])->setTargetTrajectory(trajInput);
}



void BenderApp3D::initInteractionMenu(nanogui::FormHelper* menu)
{

	// add selection of optimization algorithm
	menu->addGroup("Optimization");
	{
		nanogui::Widget *selection = new nanogui::Widget(menu->window());
		menu->addWidget("", selection);
		selection->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
			nanogui::Alignment::Middle, 0, 4));
		comboBoxOptimizationAlgorithm = new nanogui::ComboBox(selection, { "gradient descent", "quasi Newton: BFGS"});
		comboBoxOptimizationAlgorithm->setCallback([this](int idx){inverseDeformationSolver->minimizer = minimizers[idx]; });
		comboBoxOptimizationAlgorithm->setSelectedIndex(1);


		menu->addVariable("max Iterations", maxIterations);
		menu->addVariable("solve residual", solveResidual);
		menu->addVariable("max linesearch iter", maxLineSearchIterations);
	}


	menu->addGroup("Mode");
	// add selection for active interaction object
	menu->addVariable("Manipulate: ", interactionObject, true) -> setItems({"Mounts", "Target Trajectory"});	
	// add selection for interaction mode
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

void BenderApp3D::updateMountSelectionBox()
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


void BenderApp3D::switchInteractionMode(InteractionMode mode)
{
	if(mode == InteractionMode::DRAG) {
		if(interactionMode != InteractionMode::DRAG) {
			selectedNodeID = -1;
		}
	}

	interactionMode = mode;
}

void BenderApp3D::setSelectedMount(int mountID)
{
	if(mountID >= 0) {
		selected_mount = mountID;
		comboBoxMountSelection->setSelectedIndex(mountID);
	}
}


//triggered when mouse moves
bool BenderApp3D::onMouseMoveEvent(double xPos, double yPos) {

	lastMovedRay = currentRay;
	currentRay = getRayFromScreenCoords(xPos, yPos);
	
	if(interactionMode == InteractionMode::VIEW) {
		return(GLApplication::onMouseMoveEvent(xPos, yPos));
	}

	if(interactionObject == InteractionObject::MOUNTS) {
		if(interactionMode == InteractionMode::SELECT) {
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
				dynamic_cast<RotationMount2D*>(femMesh->mounts[selected_mount])->shift(delta);
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
	if(interactionObject == InteractionObject::OBJECTIVE) {
		if(interactionMode == InteractionMode::SELECT) {
			return(true);
		}
		else if(interactionMode == InteractionMode::DRAG) {
			if (selectedKnotID != -1)
			{
				Plane plane(camera->getCameraTarget(),V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
				P3D targetPos;
				currentRay.getDistanceToPlane(plane,&targetPos);
				targetTrajectory_input.setKnotValue(selectedKnotID, targetPos);
				pushInputTrajectory(targetTrajectory_input);
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
	
	return false;
}

//triggered when mouse buttons are pressed
bool BenderApp3D::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	
	
	if (button == GLFW_MOUSE_BUTTON_LEFT) {
		
		if(interactionMode == InteractionMode::VIEW) {
			return(GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos));
		}		
		
		if(interactionObject == InteractionObject::MOUNTS) {

			if(interactionMode == InteractionMode::SELECT) {
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
		else if(interactionObject == InteractionObject::OBJECTIVE) {
			if(interactionMode == InteractionMode::SELECT) {
				return(true);
			}
			else if(interactionMode == InteractionMode::DRAG) {
				if(action == GLFW_PRESS) {
					lastClickedRay = lastMovedRay;
					selectedKnotID = targetTrajectory_input.getSelectedKnotID(lastClickedRay);
				}
				else {
					selectedKnotID = -1;
				}
				return(true);
			}
			else if(interactionMode == InteractionMode::DRAW) {
				if(action == GLFW_PRESS) {
					Plane plane(camera->getCameraTarget(),V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
					P3D cursorPt;
					currentRay.getDistanceToPlane(plane,&cursorPt);
					targetTrajectory_input.addKnotInteractive(cursorPt);
					pushInputTrajectory(targetTrajectory_input);
				}
				return(true);
			}
			else {
				return(false);
			}
		}
	}

	if (button == GLFW_MOUSE_BUTTON_RIGHT) {

		if(interactionMode == InteractionMode::VIEW) {
			return(GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos));
		}

		if(interactionObject == InteractionObject::MOUNTS) {
			if(interactionMode == InteractionMode::SELECT) {
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
		else if(interactionObject == InteractionObject::OBJECTIVE) {
			if(interactionMode == InteractionMode::SELECT) {
				return(true);
			}
			else if(interactionMode == InteractionMode::DRAG) {
				return(true);
			}
			else if(interactionMode == InteractionMode::DRAW) {
				if(action == GLFW_PRESS) {
					lastClickedRay = lastMovedRay;
					selectedKnotID = targetTrajectory_input.getSelectedKnotID(lastClickedRay);
					if(selectedKnotID >= 0) {
						targetTrajectory_input.removeKnotInteractive(selectedKnotID);
						pushInputTrajectory(targetTrajectory_input);
					}
				}
				return(true);
			}
			else {
				return(false);
			}
		}

	}

	return false;
}

//triggered when using the mouse wheel
bool BenderApp3D::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	
	if(interactionMode == InteractionMode::VIEW) {
		return(GLApplication::onMouseWheelScrollEvent(xOffset, yOffset));
	}

	if(interactionObject == InteractionObject::MOUNTS) {
		if(interactionMode == InteractionMode::SELECT) {
			return(true);
		}
		else if(interactionMode == InteractionMode::DRAG) {
			if (selected_mount >= 0) {
				std::cout << "Offsets: " << xOffset << " " << yOffset << std::endl;
				Plane plane(camera->getCameraTarget(),V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
				P3D origin; 
				currentRay.getDistanceToPlane(plane,&origin);
				dynamic_cast<RotationMount2D*>(femMesh->mounts[selected_mount])->rotate(origin, yOffset * 0.05);
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
	if(interactionObject == InteractionObject::OBJECTIVE) {
		if(interactionMode == InteractionMode::SELECT) {
			return(true);
		}
		else if(interactionMode == InteractionMode::DRAG) {
			return(true);
		}
		else if(interactionMode == InteractionMode::DRAW) {
			return(true);
		}
		else {
			return(false);
		}
	}
}

bool BenderApp3D::onKeyEvent(int key, int action, int mods) {	
	
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool BenderApp3D::onCharacterPressedEvent(int key, int mods) {

	if (!mods) {

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


void BenderApp3D::loadFile(char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);
	if (fNameExt == "tri2d") {
		//delete femMesh;
		femMesh = new BenderSimulationMesh<3>;
		femMesh->readMeshFromFile(fName);
		Logger::consolePrint("...Done!");
	} else if (fNameExt == "obj") {
		//delete femMesh;
		femMesh = new BenderSimulationMesh<3>;
		femMesh->readMeshFromFile(fName);
		Logger::consolePrint("...Done!");
	} else if(fNameExt == "ply") {
		//delete femMesh;
		femMesh = new BenderSimulationMesh<3>;
		femMesh->readMeshFromFile_ply(fName);
		Logger::consolePrint("...Done!");
	}	
	else {
		Logger::consolePrint("...but how to do with that?");
	}

}

void BenderApp3D::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}


// Run the App tasks
void BenderApp3D::process() {
	//do the work here...

	/*
	simulationTime = 0;
	maxRunningTime = 1.0 / desiredFrameRate;
	femMesh->checkDerivatives = checkDerivatives != 0;

	//if we still have time during this frame, or if we need to finish the physics step, do this until the simulation time reaches the desired value
	while (simulationTime < 1.0 * maxRunningTime) {

		if(optimizeObjective) {
			double o_new = 0;
			o_new = inverseDeformationSolver->solveOptimization(solveResidual,
																maxIterations,
																lineSearchStartValue,
																maxLineSearchIterations);

			double e_new = femMesh->computeTargetPositionError();
			double delta_o = o_new - o_last;
			double delta_e = e_new - e_last;
			Logger::consolePrint("o | e | delta o | delta e = %10f | %10f | %+-10.7f | %+-10.7f \n", o_new, e_new, delta_o, delta_e);
			o_last = o_new;
			e_last = e_new;

			break;
		}
		else {
			inverseDeformationSolver->solveMesh(computeStaticSolution, simTimeStep);
			simulationTime += simTimeStep;
		}
	}
	*/
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void BenderApp3D::drawScene() {
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glColor3d(1,1,1);

	// draw mesh
	femMesh->drawSimulationMesh();

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


	
	// draw nodes of mounted points
	for(int i = 0; i < femMesh->pinnedNodeElements.size(); ++i) {
		MountedPointSpring<2> * mp = static_cast<MountedPointSpring<2> *>(femMesh->pinnedNodeElements[i]);

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

	

	// draw target trajectory
	targetTrajectory_input.draw(V3D(0.3, 0.3, 0.3), 2, V3D(0, 0.8, 0), 0.005);
	
	
	// draw objective
	for(MeshObjective * o : femMesh->objectives) {
		o->draw(femMesh->x);
	}
	
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void BenderApp3D::drawAuxiliarySceneInfo() 
{

}

// Restart the application.
void BenderApp3D::restart() 
{

}

void BenderApp3D::addRotationMount() 
{
	femMesh->addMount<RotationMount2D>();
	inverseDeformationSolver->pullXi();
	updateMountSelectionBox();
}

void BenderApp3D::removeSelectedMount()
{
	femMesh->removeMount(selected_mount);
	inverseDeformationSolver->pullXi();
	updateMountSelectionBox();
}


void BenderApp3D::addMountedNode(int node_id, int mount_id)
{
	if(node_id < 0) {return;}
	if(mount_id < 0) {return;}
	femMesh->setMountedNode(node_id, femMesh->nodes[node_id]->getCoordinates(femMesh->X), mount_id);
}

void BenderApp3D::unmountNode(int node_id, int mount_id)
{
	if(node_id < 0) {return;}
	if(mount_id < 0) {return;}
	femMesh->unmountNode(node_id, mount_id);
}


bool BenderApp3D::processCommandLine(const std::string& cmdLine) 
{
	if (GLApplication::processCommandLine(cmdLine)) return true;
	return false;
}
