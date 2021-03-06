#pragma once

#include <GUILib/GLApplication.h>
#include <ControlLib/Robot.h>
#include <RBSimLib/AbstractRBEngine.h>
#include <ControlLib/RobotControlInterface.h>
#include <ControlLib/YuMiControlInterface.h>

#include <string>
#include <array>
#include <map>

#include "OptimizationLib/GradientBasedFunctionMinimizer.h"
#include "BenderSimulationMesh.h"
#include "Trajectory3Dplus.h"
#include "InverseDeformationSolver.h"
#include "RobotMount.h"

class BenderExperimentConfiguration;


class PerformanceStudy {


public:

	Timer timer;
	bool has_been_started = false;
	//double timed_convergence_goal = 1e-8;

	std::vector<double> convergence_goals;
	std::vector<double> time_convergence_goals;
	std::vector<int> it_convergence_goals;
	int convergence_goals_reached;

	std::vector<double> error_goals;
	std::vector<double> time_error_goals;
	std::vector<int> it_error_goals;
	int error_goals_reached;

	//double error_goal;
	//double time_error_goal;
	//int it_error_goal;
	//bool error_goal_reached;

	double error_converged = -1;

	

	int it = 0;
	double t = 0;
	double e_last;

public:
	
	PerformanceStudy();

	void start();

	bool post(double e);

};



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
	RobotParameters * robotMountParameters;

	IDCustomYuMiControlInterface* robotControlInterface = NULL;

	IK_Solver * ikSolver = NULL;

	DynamicArray<Node *> matchedFiber;

	// solver for the inverse problem
	InverseDeformationSolver<3> * inverseDeformationSolver;

	// Experiment setup
	BenderExperimentConfiguration * config;

	// Optimization Algorithms
	enum OptimizationAlgorithms {GD_MINIMIZER, BFGS_MINIMIZER};
	OptimizationAlgorithms selectedMinimizationAlgorithm = BFGS_MINIMIZER;
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
	
	bool synchronizePhysicalRobot = false;

	bool runIkSolver = false;
	int selectedArmIk = -1;
	P3D selectedIkPoint;
	int selectedGeneralizedRobotParameter = -1;
	int selectedXi = 0;

	RigidBody * right_gripper;
	RigidBody * left_gripper;
	
	enum InteractionObject {MOUNTS, OBJECTIVE_TRAJECTORY, OBJECTIVE_NODE, IKROBOT};
	InteractionObject interactionObject = OBJECTIVE_NODE;
	enum InteractionMode {VIEW, SELECT, DRAG, DRAW};
	InteractionMode interactionMode = VIEW;
	enum ToolMode {PICK_NODE, BRUSH};
	ToolMode toolMode = PICK_NODE;

	enum CameraView {FRONT, RIGHT, FRONTRIGHT, FRONTRIGHTABOVE};
	CameraView cameraView = FRONT;


	// app parameters
	double simTimeStep = 1/100.0;
	double simulationTime;
	double maxRunningTime;

	// visualization robot
	bool showMesh = true;
	bool showAbstract = false;
	bool showRotationAxes = false;
	bool highlightSelected = true;
	bool showMOI = false;
	bool showCDPs = false;

	// visualize FEM mesh
	bool showFEMSurface = true;
	

	// Interaction
	Ray lastClickedRay = Ray(P3D(0, 0, 0), V3D(0, 0, 1));
	Ray lastMovedRay = Ray(P3D(0, 0, 0), V3D(0, 0, 1));
	Ray currentRay = Ray(P3D(0, 0, 0), V3D(0, 0, 1));
	int selectedNodeID = -1;
	int hoveredNodeID = -1;
	int selectedKnotID = -1;
	int selectedMountID = -1;
	int selectedObjectiveID = -1;
	int hoveredObjectiveID = -1;

	// mount base in rigid body coordinates (for manipualation of the IK)
	V3D mountBaseOriginRB_r;
	V3D mountBaseOriginRB_l;


	// menu elements
	nanogui::Screen *menuScreen2 = nullptr;
	nanogui::FormHelper *menu2 = nullptr;

	nanogui::ComboBox * comboBoxMountSelection;
	std::array<nanogui::Button *, 4> buttonsInteractionMode;
	//nanogui::ComboBox * comboBoxOptimizationAlgorithm;

	nanogui::Button *connectRobotButton;
	nanogui::Button *synchronizeRobotButton;


	bool do_performance_study = false;
	PerformanceStudy performanceStudy;
	//// timer for convergence of specific setup
	//bool measure_convergence_time = true;
	//Timer timer_convergence;
	////double timed_convergence_goal = 1e-8;
	//std::vector<double> convergence_goals = {1e-4, 3.3e-5, 1e-5, 3.3e-6, 1e-6, 3.3e-7, 1e-7, 3.3e-8, 1e-8, 3.3e-9, 1e-9};
	//double error_goals = 
	//int steps_optimization = 33;
	//bool timer_is_running = false;
	//int i_step = -1;

	int i_temp = 0;

	std::fstream outfile_convergence;


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
	

	void setupExperiment(BenderExperimentConfiguration & config);

	void initInteractionMenu(nanogui::FormHelper* menu);
	void updateMountSelectionBox();

	void switchInteractionMode(InteractionMode mode);
	void setSelectedMount(int mountID);
	void pushInputTrajectory(Trajectory3Dplus & trajInput);

	P3D getRayPointViewNormal(Ray const & ray, P3D const & pointOnPlane);

	void setCameraPosition(CameraView cameraView);

	void write_performance_study_to_file(std::string const & fileName);

};






class Gripper;
class FemMount;

class BenderExperimentConfiguration{

public:

	V3D gravity;
	bool use_joint_angle_constraints = true;
	bool use_collision_avoidance = true;
	bool initialize_robot_state_with_IK = true;

	// fem mesh
	double maxTetVolume = 0.5e-6;

	double massDensity = 43.63;
	double youngsModulus = 2.135e4;
	double poissonRatio = 0.376;

	std::string fem_model_filename;
	P3D fem_offset;
	double fem_scale = 1.0;
	std::vector<FemMount> femMounts;
	
	int n_nodes_matched_fiber = -1;
	P3D matched_fiber_start;
	P3D matched_fiber_end;

	// Grippers
	std::vector<Gripper> grippers;

	// limits for distance between grippers (namely, the mount-point-origins of the grippers)
	std::vector<std::tuple<int, int, double, double, double, double> > distanceLimitsGrippers;	// contains: pair i,j of grippers; lower limit; upper limit; stiffness; epsilon

	// color
	V3D femObjColor = V3D(0.7, 1.0, 0.7);

public:


};


class Gripper {
public:
	P3D mount_origin_surfacemesh;				// defined with respect to the surface-mash of the model
	Matrix3x3 mount_orientation_surfacemesh;
	//RigidBody * gripper_body;
	std::string rigidBody_name;

	std::vector<AxisAlignedBoundingBox> contact_regions;	// relative to the mount_origin & mount_orientation

	enum Side {RIGHT, LEFT};
	enum FingerType {FINGER_ABB_STANDARD, PLANE_ABB_FINGERTIPS_PLUS_5, WAFFLE_40x40};

public:
	Gripper() {};
	Gripper(P3D origin, V3D dir_1, V3D dir_2, std::string const & rigidBody_name);

	void makeYuMiGripper(P3D mountOrigin_baseplate, Side side, FingerType type, double gripper_width);
	void makeYuMiGripper_default_mounting(Side side, FingerType type, double gripper_width);
	void makeFloorGripper(P3D mountOrigin_baseplate);
	void setContactRegions(Side side, FingerType type, double gripper_width, P3D mountOrigin_baseplate);

	void addContactRegion(P3D pt1, P3D pt2);

};

class FemMount {
public:
	P3D mount_origin;
	Matrix3x3 mount_orientation;
public: 
	FemMount(P3D origin, V3D dir_1, V3D dir_2);
};
