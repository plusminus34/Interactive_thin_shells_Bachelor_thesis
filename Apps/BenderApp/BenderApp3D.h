#pragma once

#include <GUILib/GLApplication.h>
#include <ControlLib/Robot.h>
#include <RBSimLib/AbstractRBEngine.h>
#include <ControlLib/RobotControlInterface.h>

#include <string>
#include <array>
#include <map>

#include "OptimizationLib/GradientBasedFunctionMinimizer.h"
#include "BenderSimulationMesh.h"
#include "Trajectory3Dplus.h"
#include "InverseDeformationSolver.h"


/**
 * Test App
 */
class BenderApp3D : public GLApplication {
public:
	// Geometry & physics
	BenderSimulationMesh<3> * femMesh;

	// target trajectory for shape of mesh
	Trajectory3Dplus targetTrajectory_input;

	// robot
	Robot * robot = NULL;
	AbstractRBEngine * rbEngine = NULL;
	RobotState startState = RobotState();
	GeneralizedCoordinatesRobotRepresentation * generalizedRobotCoordinates= NULL;

	IK_Solver * ikSolver = NULL;

	// mount base in coords of gripper mesh

	V3D mountBaseOriginMesh_r;
	V3D mountBaseOriginMesh_l;
	Matrix3x3 mountBaseCoordinatesMesh_r;
	Matrix3x3 mountBaseCoordinatesMesh_l;

	V3D mountBaseOriginRB_r;
	V3D mountBaseOriginRB_l;
	V3D mountBaseAxialDirectionRB_r;
	V3D mountBaseAxialDirectionRB_l;

	Matrix3x3 mountBaseCoordinatesRB_r;
	Matrix3x3 mountBaseCoordinatesRB_l;

	// solver for the inverse problem
	InverseDeformationSolver<3> * inverseDeformationSolver;

	// Optimization Algorithms
	std::vector<GradientBasedFunctionMinimizer*> minimizers;
	int maxIterations = 1;
	double solveResidual = 1e-5;
	int maxLineSearchIterations = 15;
	double lineSearchStartValue = 0.1;

	double xiRegularizerValue = -1.0;

	// mesh properties
	double shearModulus = 50, bulkModulus = 50;
	bool autoUpdateShearModulusAndBulkModulus = false;
	
	// diagnostics
	double o_last = 0.0;	// last result for mesh objective function
	double e_last = 0.0; // last result for error of mesh objective

	// state of the app 
	bool computeStaticSolution = true;
	bool optimizeObjective = false;
	bool checkDerivatives = false;

	bool runIkSolver = false;
	int selectedArmIk = -1;
	P3D selectedIkPoint;
	int selectedGeneralizedRobotParameter = -1;
	int selectedXi = 0;

	RigidBody * right_gripper;
	RigidBody * left_gripper;
	
	enum InteractionObject {MOUNTS, OBJECTIVE, IKROBOT};
	InteractionObject interactionObject = IKROBOT;
	enum InteractionMode {VIEW, SELECT, DRAG, DRAW};
	InteractionMode interactionMode = VIEW;
	enum ToolMode {PICK_NODE, BRUSH};
	ToolMode toolMode = PICK_NODE;

	// app parameters
	double simTimeStep = 1/100.0;
	double simulationTime;
	double maxRunningTime;

	// visualization robot
	bool showMesh = false;
	bool showAbstract = true;
	bool showRotationAxes = true;
	bool highlightSelected = true;
	bool showMOI = false;
	bool showCDPs = false;
	

	// Interaction
	Ray lastClickedRay = Ray(P3D(0, 0, 0), V3D(0, 0, 1));
	Ray lastMovedRay = Ray(P3D(0, 0, 0), V3D(0, 0, 1));
	Ray currentRay = Ray(P3D(0, 0, 0), V3D(0, 0, 1));
	int selectedNodeID = -1;
	int selectedKnotID = -1;
	int selected_mount = -1;

	// menu elements
	nanogui::ComboBox * comboBoxMountSelection;
	std::array<nanogui::Button *, 4> buttonsInteractionMode;
	nanogui::ComboBox * comboBoxOptimizationAlgorithm;

public:
	// constructor
	BenderApp3D();
	// destructor
	virtual ~BenderApp3D(void);
	// Run the App tasks
	virtual void process();
	// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
	virtual void drawScene();
	// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
	virtual void drawAuxiliarySceneInfo();
	// Restart the application.
	virtual void restart();

	// add a mount
	void addRotationMount();
	void removeSelectedMount();
	// add a node to a mount
	void addMountedNode(int node_id, int mount_id);
	void unmountNode(int node_id, int mount_id);


	//input callbacks...

	//all these methods should returns true if the event is processed, false otherwise...
	//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
	virtual bool onKeyEvent(int key, int action, int mods);
	//this one gets triggered on UNICODE characters only...
	virtual bool onCharacterPressedEvent(int key, int mods);
	//triggered when mouse buttons are pressed
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	//triggered when mouse moves
	virtual bool onMouseMoveEvent(double xPos, double yPos);
	//triggered when using the mouse wheel
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);

	virtual bool processCommandLine(const std::string& cmdLine);
	
	//virtual void saveFile(const char* fName);
	//virtual void loadFile(char* fName);

	void pushInputTrajectory(Trajectory3Dplus & trajInput);

	void initInteractionMenu(nanogui::FormHelper* menu);
	void updateMountSelectionBox();

	void switchInteractionMode(InteractionMode mode);
	void setSelectedMount(int mountID);

};



