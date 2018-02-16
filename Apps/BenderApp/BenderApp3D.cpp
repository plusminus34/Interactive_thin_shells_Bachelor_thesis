#include <GUILib/GLUtils.h>

#include <Utils/Timer.h>

#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <MathLib/Plane.h>

#include <LazyFEMSimLib/CSTSimulationMesh3D.h>
#include <LazyFEMSimLib/MassSpringSimulationMesh3D.h>
#include <GUILib/GLUtils.h>

#include <RBSimLib/ODERBEngine.h>
//#include <ControlLib/YuMiControlInterface.h>
#include "IDCustomYuMiControlInterface.h"


#include "OptimizationLib/GradientDescentFunctionMinimizer.h"
#include "OptimizationLib/BFGSFunctionMinimizer.h"
#include "MathLib/Transformation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <tuple>

#include "RotationMount3D.h"
#include "RobotMount.h"
#include "MountedPointSpring.h"
#include "MatchScaledTrajObjective.h"
#include "ParameterConstraintObjective.h"
#include "RBSphereCollision.h"

#include "BenderApp3D.h"

#define EDIT_BOUNDARY_CONDITIONS

//#define CONSTRAINED_DYNAMICS_DEMO

BenderApp3D::BenderApp3D() 
{

	const double rod_length = 0.3;
	const P3D rod_center(0.0, 0.35, 0.50);

	// fem mesh coarsness
	double maxTetVolume = 0.5e-6;1.35e-6;//1.35e-6;

	// measured (physical) values for "the green foam" are: (density, young, poisson) = (43.63, 2.135e4, 0.376)
	double massDensity = 43.63;//130;//50;
	double youngsModulus = 1.5e4;//3.0e4;//3.0e4;
	double poissonRatio = 0.25;//0.376;//0.25;

	//double massDensity = 43.63;
	//double youngsModulus = 2.135e4;
	//double poissonRatio = 0.376;

	double shearModulus = youngsModulus / (2 * (1 + poissonRatio));
	double bulkModulus = youngsModulus / (3 * (1 - 2 * poissonRatio));


	setWindowTitle("Test FEM Sim Application...");

	// prepare recording of screen
	screenRecorder = new ScreenRecorder();


	////////////////////////
	// FEM Mesh
	////////////////////////

	// create some points on centerline of the mesh
	DynamicArray<P3D> centerlinePts;
	{
		int m = 11;
		centerlinePts.resize(m);
		V3D pt1(-0.15, 0.0, 0.0);
		V3D pt2(0.15, 0.0, 0.0);
		double dt = 1.0 / static_cast<double>(m-1);
		for(int i = 0; i < m; ++i) {
			centerlinePts[i] = pt1 + (pt2 - pt1) * dt*static_cast<double>(i);
		}
	}

	// create mesh
	femMesh = new BenderSimulationMesh<3>;
	femMesh->readMeshFromFile_ply("../data/3dModels/square_rod_0p03x0p3.ply", &centerlinePts,
								  massDensity, shearModulus, bulkModulus,
								  1.0, rod_center,
								  maxTetVolume);
	femMesh->addGravityForces(V3D(0, -9.8, 0));	


	// add a fiber in Mesh to match
	auto node_sequence_from_points = [&](DynamicArray<P3D> & pts, double tolerance,
										 DynamicArray<Node*> & fiber)
	{
		fiber.resize(0);
		for(P3D pt : pts) {
			pt += rod_center;
			for(Node * node : femMesh->nodes) {
				if((pt - node->getCoordinates(femMesh->X)).length() < tolerance) {
					fiber.push_back(node);
					break;
				}
			}
		}
	};
	node_sequence_from_points(centerlinePts, 0.0001, matchedFiber);


	// initialize minimization algorithms
	minimizers.push_back(new GradientDescentFunctionMinimizer(maxIterations, solveResidual, maxLineSearchIterations, false));
	minimizers.push_back(new BFGSFunctionMinimizer           (maxIterations, solveResidual, maxLineSearchIterations, false));

	// create ID Solver
	inverseDeformationSolver = new InverseDeformationSolver<3>(femMesh, minimizers[selectedMinimizationAlgorithm]);
	
	// set the target trajectory
	if(true){
		// draw some target trjectory
		targetTrajectory_input.addKnotBack(rod_center + P3D(-rod_length*0.5, 0.05, 0.0));
		targetTrajectory_input.addKnotBack(rod_center + P3D( 0.0,  0.1, 0.0));
		targetTrajectory_input.addKnotBack(rod_center + P3D( rod_length*0.5, 0.05, 0.0));

		// add a "MatchScaledTrajObjective"
		targetTrajectory_input.setTValueToLength();
		femMesh->objectives.push_back(new MatchScaledTrajObjective(matchedFiber, targetTrajectory_input));
	}

	////////////////////////
	// Robot
	////////////////////////
	
	// load robot
	std::string fnameRB = "../data/rbs/yumi/yumi_simplified.rbs";

	auto loadRobot = [&] (std::string const & fname)
	{
		delete robot;
		delete rbEngine;

		rbEngine = new ODERBEngine();
		rbEngine->loadRBsFromFile(fname.c_str());
		robot = new Robot(rbEngine->rbs[0]);
		startState = RobotState(robot);
		setupSimpleRobotStructure(robot);
	};
	loadRobot(fnameRB);

	right_gripper = rbEngine->getRBByName("link_7_r");
	left_gripper = rbEngine->getRBByName("link_7_l");
	

	robot->setHeading(-PI / 2.0);


	// find mount point on gripper

	//P3D mount_point_surfacemesh_r = (P3D(0.628,0.085,0.344) + P3D(0.647,0.075,0.356)) * 0.5;
	//P3D mount_point_surfacemesh_l = (P3D(0.628,-0.085,0.344) + P3D(0.647,-0.075,0.356)) * 0.5;
	P3D mount_point_surfacemesh_r = (P3D(0.634457, 0.093577, 0.335758) + P3D(0.656349, 0.0844, 0.348425)) * 0.5;
	P3D mount_point_surfacemesh_l = (P3D(0.634453, -0.093602, 0.335783) + P3D(0.656353, -0.084374, 0.3484)) * 0.5;

	V3D mount_axialDirection_surfacemesh_r = (P3D(0.627,0.084,0.344) - P3D(0.619,0.076,0.351)).unit();
	V3D mount_axialDirection_surfacemesh_l = (P3D(0.647,-0.075,0.356) - P3D(0.639,-0.067,0.363)).unit();

	//V3D mount_zDirection_r = -(P3D(0.628,0.085,0.344) - P3D(0.647,0.075,0.356));
	//V3D mount_zDirection_l = -(P3D(0.628,-0.085,0.344) - P3D(0.647,-0.075,0.356));
	V3D mount_zDirection_r = -(P3D(0.656349, 0.0844, 0.348425) - P3D(0.634457, 0.093577, 0.335758)).unit();
	V3D mount_zDirection_l = -(P3D(0.656353, -0.084374, 0.3484) - P3D(0.634453, -0.093602, 0.335783)).unit();

	mountBaseOriginRB_r = right_gripper->meshTransformations[0].transform(mount_point_surfacemesh_r);
	mountBaseOriginRB_l = left_gripper->meshTransformations[0].transform(mount_point_surfacemesh_l);


	mountBaseAxialDirectionRB_r = right_gripper->meshTransformations[0].transform(mount_axialDirection_surfacemesh_r);
	mountBaseAxialDirectionRB_l = right_gripper->meshTransformations[0].transform(mount_axialDirection_surfacemesh_l);

	mountBaseZDirectionRB_r = right_gripper->meshTransformations[0].transform(mount_zDirection_r);
	mountBaseZDirectionRB_l = right_gripper->meshTransformations[0].transform(mount_zDirection_l);

	//mountBaseOriginRB_l = V3D(0.0);
	mountBaseCoordinatesRB_l << 0.0, 0.0, 1.0,
		0.0, -1.0, 0.0,
		1.0, 0.0, 0.0;

	//mountBaseOriginRB_r = V3D(0.0);
	mountBaseCoordinatesRB_r << 0.0, 0.0, 1.0,
		0.0, -1.0, 0.0,
		1.0, 0.0, 0.0;

	////////////////////////
	// IK Solver for Robot
	////////////////////////
	delete ikSolver;
	ikSolver = new IK_Solver(robot, true);
	// new target: right gripper
	ikSolver->ikPlan->endEffectors.push_back(IK_EndEffector());
	ikSolver->ikPlan->endEffectors.back().endEffectorRB = right_gripper;

	ikSolver->ikPlan->endEffectors.back().endEffectorLocalCoords = mountBaseOriginRB_r;
	ikSolver->ikPlan->endEffectors.back().targetEEPos = P3D(-rod_length*0.5, 0.0, 0.0) + rod_center;

	ikSolver->ikPlan->endEffectors.back().endEffectorLocalOrientation(0) = mountBaseAxialDirectionRB_r;
	ikSolver->ikPlan->endEffectors.back().endEffectorLocalOrientation(1) = V3D(0.0, 1.0, 0.0);
	ikSolver->ikPlan->endEffectors.back().endEffectorLocalOrientation(2) = mountBaseZDirectionRB_r;
	ikSolver->ikPlan->endEffectors.back().orientationMask = V3D(1.0, 0.0, 1.0);

	ikSolver->ikPlan->endEffectors.back().targetEEOrientation(0) = V3D(1.0, 0.0, 0.0).unit();
	ikSolver->ikPlan->endEffectors.back().targetEEOrientation(1) = V3D(0.0, 1.0, 0.0);
	ikSolver->ikPlan->endEffectors.back().targetEEOrientation(2) = V3D(0.0, 0.0, 1.0);

	ikSolver->ikPlan->endEffectors.back().lengthScaleOrientation = +0.5;

	// new target: left gripper
	ikSolver->ikPlan->endEffectors.push_back(IK_EndEffector());
	ikSolver->ikPlan->endEffectors.back().endEffectorRB = left_gripper;

	ikSolver->ikPlan->endEffectors.back().endEffectorLocalCoords = mountBaseOriginRB_l;
	ikSolver->ikPlan->endEffectors.back().targetEEPos = P3D(+rod_length*0.5, 0.0, 0.0) + rod_center;

	ikSolver->ikPlan->endEffectors.back().endEffectorLocalOrientation(0) = mountBaseAxialDirectionRB_l;
	ikSolver->ikPlan->endEffectors.back().endEffectorLocalOrientation(1) = V3D(0.0, 1.0, 0.0);
	ikSolver->ikPlan->endEffectors.back().endEffectorLocalOrientation(2) = mountBaseZDirectionRB_l;
	ikSolver->ikPlan->endEffectors.back().orientationMask = V3D(1.0, 0.0, 1.0);

	ikSolver->ikPlan->endEffectors.back().targetEEOrientation(0) = V3D(-1.0, 0.0, 0.0).unit();
	ikSolver->ikPlan->endEffectors.back().targetEEOrientation(1) = V3D(0.0, 1.0, 0.0);
	ikSolver->ikPlan->endEffectors.back().targetEEOrientation(2) = V3D(0.0, 0.0, 1.0);

	ikSolver->ikPlan->endEffectors.back().lengthScaleOrientation = +0.5;

	for(int i = 0; i < 100; ++i) {
		ikSolver->ikEnergyFunction->regularizer = 100;
		ikSolver->ikOptimizer->checkDerivatives = false;
		ikSolver->solve();
// 		testGeneralizedCoordinateRepresentation(robot);
	}

	// create generalized parametrization of the robot
	generalizedRobotCoordinates = new GeneralizedCoordinatesRobotRepresentation(robot);



	// create a parameter set with the above robot coordinatespr
	robotMountParameters = new RobotParameters(generalizedRobotCoordinates);
	inverseDeformationSolver->parameterSets.push_back(robotMountParameters);
	
	// create constraints for that parameter set (i.e. the joint angles)
	{
		ParameterSet * jointAnglePars = inverseDeformationSolver->parameterSets.back();
		int n = jointAnglePars->getNPar();
		for(int i = 0; i < n; ++i) {
			inverseDeformationSolver->objectiveFunction->parameterConstraints.
				push_back(new ParameterConstraintObjective(jointAnglePars, i,
														   true, true,
														   1000, 0.0873,
														   -2.0*PI, 2.0*PI));
		}
	}

	// create Collision avoidance Objective
	RBSphereCollisionObjective * rbsco = new RBSphereCollisionObjective(static_cast<RobotParameters*>(inverseDeformationSolver->parameterSets.back()),
								   rbEngine->rbs,
								   0.00, 10000.0, 0.002);
	inverseDeformationSolver->objectiveFunction->parameterConstraints.push_back(rbsco);

	
	// create a mount on the left gripper
	femMesh->addMount<RobotMount>(inverseDeformationSolver->parameterSets.back());
	RobotMount * mount_left_gripper = static_cast<RobotMount*>(femMesh->mounts.back());
	int mount_id_left_gripper = femMesh->mounts.size()-1;
	mount_left_gripper->robotPart = left_gripper;
	// create a mount on the right gripper
	femMesh->addMount<RobotMount>(inverseDeformationSolver->parameterSets.back());
	RobotMount * mount_right_gripper = static_cast<RobotMount*>(femMesh->mounts.back());
	int mount_id_right_gripper = femMesh->mounts.size()-1;
	mount_right_gripper->robotPart = right_gripper;
	

	
	// set rotation mounts
	auto set_rotation_mount_from_plane = [&](int dim, double val, double tolerance) 
	{
		std::vector<int> nodes_id(0);
		for(int i = 0; i < (int)femMesh->nodes.size(); ++i) {
			P3D pt = femMesh->nodes[i]->getCoordinates(femMesh->X);
			if(pt[dim] > val-tolerance && pt[dim] < val+tolerance) {
				nodes_id.push_back(i);
			}
		}
		if(nodes_id.size() > 0) {
			addRotationMount();
			int i_mount = femMesh->mounts.size()-1;
			for(int i : nodes_id) {
				addMountedNode(i, i_mount);
			}
		}
	};


	
	// set robot mount
	auto add_node_to_robot_mount = [&](int nodeId, int mountId, 
										V3D & mount_origin_mesh, Matrix3x3 & mount_coordinates_mesh, 
										V3D & mount_origin_body, Matrix3x3 & mount_coordinates_body)
	{
		V3D x0 = femMesh->nodes[nodeId]->getCoordinates(femMesh->X);
		V3D x0_mount = static_cast<V3D>(mount_coordinates_mesh  * (x0 - mount_origin_mesh));
		V3D x0_body = static_cast<V3D>(mount_coordinates_body * x0_mount + mount_origin_body);

		femMesh->setMountedNode(nodeId, static_cast<P3D>(x0_body), mountId);
	};

	auto set_robot_mount_from_plane = [&](int dim, double val, double tolerance,
										int mountId, 
										V3D & mount_origin_mesh, Matrix3x3 & mount_coordinates_mesh, 
										V3D & mount_origin_body, Matrix3x3 & mount_coordinates_body)
	{
		std::vector<int> nodes_id(0);
		for(int i = 0; i < (int)femMesh->nodes.size(); ++i) {
			P3D pt = femMesh->nodes[i]->getCoordinates(femMesh->X);
			if(pt[dim] > val-tolerance && pt[dim] < val+tolerance) {
				nodes_id.push_back(i);
			}
		}

		for(int i : nodes_id) {
			add_node_to_robot_mount(i, mountId,
									mount_origin_mesh, mount_coordinates_mesh,
									mount_origin_body, mount_coordinates_body);
		}
	};

	mountBaseOriginMesh_r = V3D(-rod_length*0.5, 0.0, 0.0) + rod_center + V3D(-0.005, 0.0, 0.0);
	mountBaseCoordinatesMesh_r << 0.0, 0.0, 1.0,
									0.0, -1.0, 0.0,
									1.0, 0.0, 0.0;

	mountBaseOriginMesh_l = V3D(+rod_length*0.5, 0.0, 0.0) + rod_center + V3D(+0.005, 0.0, 0.0);
	mountBaseCoordinatesMesh_l << 0.0, 0.0, 1.0,
								0.0, -1.0, 0.0,
								1.0, 0.0, 0.0;


	//mountBaseOriginRB_l = V3D(0.0);
	Quaternion gripper_left_orientation = left_gripper->state.orientation;
	Matrix3x3 gripper_left_orientation_matrix = gripper_left_orientation.getRotationMatrix();
	mountBaseCoordinatesRB_l = gripper_left_orientation_matrix.inverse()*mountBaseCoordinatesMesh_l;
	//mountBaseOriginRB_r = V3D(0.0);
	Quaternion gripper_right_orientation = right_gripper->state.orientation;
	Matrix3x3 gripper_right_orientation_matrix = gripper_right_orientation.getRotationMatrix();
	mountBaseCoordinatesRB_r = gripper_right_orientation_matrix.inverse()*mountBaseCoordinatesMesh_r;



	set_robot_mount_from_plane(0, -rod_length*0.5, 0.001*rod_length,
								mount_id_right_gripper,
								mountBaseOriginMesh_r, mountBaseCoordinatesMesh_r,
								mountBaseOriginRB_r, mountBaseCoordinatesRB_r);
	set_robot_mount_from_plane(0, +rod_length*0.5, 0.001*rod_length,
								mount_id_left_gripper,
								mountBaseOriginMesh_l, mountBaseCoordinatesMesh_l,
								mountBaseOriginRB_l, mountBaseCoordinatesRB_l);


	//set_rotation_mount_from_plane(0, -rod_length/2.0, 0.01);
	//set_rotation_mount_from_plane(0, +rod_length/2.0, 0.01);


	// initialize the ID Solver
	inverseDeformationSolver->pullXi();
	// set a regularizer for the IDSolver, update the regularizer solution for the IK Solver
	inverseDeformationSolver->objectiveFunction->setReferenceStateP();



	///////////////////////////////
	// Physical robot interface
	///////////////////////////////
	robotControlInterface = new IDCustomYuMiControlInterface(robot, robotMountParameters);
	//robotControlInterface = new YuMiControlInterface(robot);

	robotControlInterface->controlPositionsOnly = true;
	static_cast<YuMiControlInterface *>(robotControlInterface)->sendControlInputsDelayed = false;

	///////////////////////////////
	// Window
	///////////////////////////////

	// camera view
	camera->setCameraTarget(P3D(rod_center) - V3D(0.0, 0.0, 0.25));

	glfwSetWindowSize(glfwWindow, 1920, 1080);
	//glfwSetWindowSize(glfwWindow, 1280, 720);

	desiredFrameRate = 90;


	////////////////////////
	// menu 
	////////////////////////

	initInteractionMenu(mainMenu);
	menuScreen->performLayout();
	updateMountSelectionBox();

}


BenderApp3D::~BenderApp3D()
{
	delete screenRecorder;
}




void BenderApp3D::initInteractionMenu(nanogui::FormHelper* menu)
{

	menu->addVariable("desired frame rate", desiredFrameRate);
	// 
	menu->addGroup("FEM Sim options");
	{
		menu->addVariable("Static solve", computeStaticSolution);
	}
	//
	menu->addGroup("Robot Visualization");
	menu->addVariable("Show mesh", showMesh);
	menu->addVariable("Show abstract", showAbstract);
	menu->addVariable("Show Rotation Axes", showRotationAxes);
	menu->addVariable("Highlight selected", highlightSelected);
	menu->addVariable("Show MOI", showMOI);
	menu->addVariable("Show CDP", showCDPs);



	// add selection of optimization algorithm
	menu->addGroup("Optimization");
	{
		menu->addVariable("Optimize Objective", optimizeObjective);

		menu->addButton("set state as target", [this](){
			femMesh->setNodeGlobalNodePositionObjective(femMesh->x);
		});

		nanogui::Widget *selection = new nanogui::Widget(menu->window());
		menu->addWidget("", selection);
		selection->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
			nanogui::Alignment::Middle, 0, 4));
		nanogui::ComboBox * comboBoxOptimizationAlgorithm = new nanogui::ComboBox(selection, { "gradient descent", "quasi Newton: BFGS"});
		comboBoxOptimizationAlgorithm->setCallback([this](int idx){selectedMinimizationAlgorithm = static_cast<OptimizationAlgorithms>(idx);
		                                                           inverseDeformationSolver->minimizer = minimizers[idx]; });
		comboBoxOptimizationAlgorithm->setSelectedIndex(selectedMinimizationAlgorithm);


		menu->addVariable("max Iterations", maxIterations);
		menu->addVariable("solve residual", solveResidual);
		menu->addVariable("line search start val", lineSearchStartValue);
		menu->addVariable("max linesearch iter", maxLineSearchIterations);
		menu->addVariable("regularizer FEM Position", inverseDeformationSolver->femMesh->meshPositionRegularizer.r);
		menu->addVariable("regularizer FEM Energy", inverseDeformationSolver->femMesh->meshEnergyRegularizer.r);
		menu->addVariable("regularizer joint angles", inverseDeformationSolver->objectiveFunction->parameterValueRegularizer.r);
	}

	menu->addGroup("Interaction Mode");
	// add selection for active interaction object
	menu->addVariable("Manipulate: ", interactionObject, true)->setItems({"Mounts", "Target Trajectory", "Target Node Position", "IKROBOT"});
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

		menu->addVariable("selected rob par", selectedGeneralizedRobotParameter);
		menu->addVariable("selected glob par", selectedXi);
	}

	// add combo box for selected mount/handle
	menu->addGroup("Mounts");
	{
		nanogui::Widget *selection = new nanogui::Widget(menu->window());
		menu->addWidget("", selection);
		selection->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
			nanogui::Alignment::Middle, 0, 4));
		comboBoxMountSelection = new nanogui::ComboBox(selection, { "no mounts available"});
		comboBoxMountSelection->setCallback([this](int idx){selectedMountID = idx;
		                                                    std::cout << "selected mount: " << selectedMountID << std::endl;
															});
	}
	{
		nanogui::Widget *mountStatus = new nanogui::Widget(menu->window());
		menu->addWidget("", mountStatus);
		mountStatus->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
			nanogui::Alignment::Middle, 0, 4));
		nanogui::Button *b;
		b = new nanogui::Button(mountStatus, "toogle xi");
		b->setCallback([this](){femMesh->mounts[selectedMountID]->parameterOptimization ^= true;
		                        updateMountSelectionBox();
								});
		b = new nanogui::Button(mountStatus, "toogle active");
		b->setCallback([this](){femMesh->mounts[selectedMountID]->active ^= true;
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
	
	/////////////////////
	// second window
	////////////////////
	menu->addWindow(Eigen::Vector2i(280, 0), "");

	// synchronize physical robot
	nanogui::Widget *physicalRobotControlTools = new nanogui::Widget(menu->window());
	menu->addWidget("", physicalRobotControlTools);
	physicalRobotControlTools->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 4));

	connectRobotButton = new nanogui::Button(physicalRobotControlTools, "CONNECT");
	connectRobotButton->setFlags(nanogui::Button::ToggleButton);
	connectRobotButton->setChangeCallback([&](bool is_pressed){
		if(is_pressed) {
			// try to connect
			if(robotControlInterface && robotControlInterface->isConnected() == false) {
				robotControlInterface->openCommunicationPort();
			}
		}
		else {	// button not pressed
			// try to disconnect
			if(robotControlInterface) {
				robotControlInterface->closeCommunicationPort();
			}
		}
		connectRobotButton->setPushed(robotControlInterface->isConnected());
	});

	menu->addVariable("Sync. Robot", synchronizePhysicalRobot);




	// screen recorder
	
	screenRecorder->attachToNanoGui(menu);

};

void BenderApp3D::updateMountSelectionBox()
{
	std::vector<std::string> items(femMesh->mounts.size());
	for(int i = 0; i < (int)femMesh->mounts.size(); ++i) {
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
	selectedMountID = comboBoxMountSelection->selectedIndex();

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
		selectedMountID = mountID;
		comboBoxMountSelection->setSelectedIndex(mountID);
	}
}

void BenderApp3D::pushInputTrajectory(Trajectory3Dplus & trajInput) 
{
	trajInput.setTValueToLength();
	for(MeshObjective * objective : femMesh->objectives) {
		MatchScaledTrajObjective * trajObj = dynamic_cast<MatchScaledTrajObjective *>(objective);
		if(trajObj) {
			trajObj->setTargetTrajectory(trajInput);
			return;
		}
	}
}

P3D BenderApp3D::getRayPointViewNormal(Ray const & ray, P3D const & pointOnPlane)
{
	Plane plane(pointOnPlane, V3D(camera->getCameraPosition(), camera->getCameraTarget()).unit());
	P3D Pt;
	currentRay.getDistanceToPlane(plane, &Pt);
	return(Pt);
}


//triggered when mouse moves
bool BenderApp3D::onMouseMoveEvent(double xPos, double yPos) {

	lastMovedRay = currentRay;
	currentRay = getRayFromScreenCoords(xPos, yPos);
	
	hoveredNodeID = -1;

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
				P3D pt_selected_node = femMesh->nodes[selectedNodeID]->getWorldPosition();
				Plane plane(pt_selected_node,V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
				P3D targetPos;
				P3D lastPos;
				currentRay.getDistanceToPlane(plane,&targetPos);
				lastMovedRay.getDistanceToPlane(plane, &lastPos);
				V3D delta = targetPos - lastPos;
				dynamic_cast<RotationMount3D*>(femMesh->mounts[selectedMountID])->shift(delta);
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
	if(interactionObject == InteractionObject::OBJECTIVE_TRAJECTORY) {
		if(interactionMode == InteractionMode::SELECT) {
			return(true);
		}
		else if(interactionMode == InteractionMode::DRAG) {
			if (selectedKnotID != -1)
			{
				P3D knotPos = targetTrajectory_input.getKnotValue(selectedKnotID);
				Plane plane(knotPos,V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
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
	if (interactionObject == InteractionObject::OBJECTIVE_NODE) {
		if (interactionMode == InteractionMode::SELECT) {
			return(true);
		}
		else if (interactionMode == InteractionMode::DRAG ||
				 interactionMode == InteractionMode::DRAW) {
			if (selectedObjectiveID >= 0) {
				NodePositionObjective * objective = dynamic_cast<NodePositionObjective *>(femMesh->objectives[selectedObjectiveID]);
				if (objective) {
					P3D targetPosOld = objective->targetPosition;
					P3D targetPosNew = getRayPointViewNormal(currentRay, targetPosOld);
					objective->targetPosition = targetPosNew;
				}
			}
			else {
				hoveredObjectiveID = femMesh->getSelectedNodePositionObjectiveID(currentRay);
				if(hoveredObjectiveID < 0) {
					hoveredNodeID = femMesh->getSelectedSurfaceNodeID(currentRay, true);
				}
			}
			return(true);
		}
		else {
			return(false);
		}
	}
	if(interactionObject == InteractionObject::IKROBOT) {
		if(interactionMode == InteractionMode::SELECT) {
			return(true);
		}
		else if(interactionMode == InteractionMode::DRAG) {
			if(selectedArmIk == 0) {
				RigidBody* selectedRigidBody = right_gripper;
				//P3D selectedPoint = static_cast<P3D>(mountBaseOriginRB_r);
				P3D selectedPoint = selectedIkPoint;
				P3D targetPoint;
				Ray ray = getRayFromScreenCoords(xPos, yPos);
				V3D viewPlaneNormal = V3D(camera->getCameraPosition(), camera->getCameraTarget()).unit();
				ray.getDistanceToPlane(Plane(selectedPoint, viewPlaneNormal), &targetPoint);
				ikSolver->ikPlan->endEffectors[selectedArmIk].targetEEPos = targetPoint;
			}
			else if(selectedArmIk == 1) {
				RigidBody* selectedRigidBody = left_gripper;
				//P3D selectedPoint = static_cast<P3D>(mountBaseOriginRB_l);
				P3D selectedPoint = selectedIkPoint;
				P3D targetPoint;
				Ray ray = getRayFromScreenCoords(xPos, yPos);
				V3D viewPlaneNormal = V3D(camera->getCameraPosition(), camera->getCameraTarget()).unit();
				ray.getDistanceToPlane(Plane(selectedPoint, viewPlaneNormal), &targetPoint);
				ikSolver->ikPlan->endEffectors[selectedArmIk].targetEEPos = targetPoint;
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
					addMountedNode(selectedNodeID_temp, selectedMountID);
				}
				return(true);
			}
			else {
				return(false);
			}
		}
		else if(interactionObject == InteractionObject::OBJECTIVE_TRAJECTORY) {
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
					P3D ref_pt;
					if(targetTrajectory_input.getKnotCount() == 0) {
						ref_pt = camera->getCameraTarget();
					}
					else {
						double dist = targetTrajectory_input.getDistanceToRay(currentRay, &ref_pt);
					}
					Plane plane(ref_pt,V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
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
		else if (interactionObject == InteractionObject::OBJECTIVE_NODE) {
			if (interactionMode == InteractionMode::SELECT) {
				return(true);
			}
			else if (interactionMode == InteractionMode::DRAG || 
					 interactionMode == InteractionMode::DRAW) {
				if (action == GLFW_PRESS) {
					lastClickedRay = lastMovedRay;
					if(hoveredObjectiveID >= 0) {
						selectedObjectiveID = hoveredObjectiveID;
						return(true);
					}
					else if(hoveredNodeID >= 0) {
						selectedObjectiveID = femMesh->setNodePositionObjectiveNoDuplicate(hoveredNodeID, femMesh->nodes[hoveredNodeID]->getWorldPosition());
						return(true);
					}
				}
				else {
					hoveredObjectiveID = selectedObjectiveID;
					selectedObjectiveID = -1;
				}
				return(true);
			}
			else {
				return(false);
			}
		}
		else if(interactionObject == InteractionObject::IKROBOT) {
			if(interactionMode == InteractionMode::SELECT) {
				return(true);
			}
			else if(interactionMode == InteractionMode::DRAG) {
				if(action == GLFW_PRESS) {
					selectedArmIk = 0;
					//selectedIkPoint = static_cast<P3D>(mountBaseOriginRB_r);
					selectedIkPoint = right_gripper->getWorldCoordinates(static_cast<P3D>(mountBaseOriginRB_r));
					runIkSolver = true;
				}
				else if(action == GLFW_RELEASE) {
					selectedArmIk = -1;
					runIkSolver = false;
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
				unmountNode(selectedNodeID_temp, selectedMountID);
				return(true);
			}
			else {
				return(false);
			}
		}
		else if(interactionObject == InteractionObject::OBJECTIVE_TRAJECTORY) {
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
		else if (interactionObject == InteractionObject::OBJECTIVE_NODE) {
			if (interactionMode == InteractionMode::DRAG || 
				interactionMode == InteractionMode::DRAW) {
				if (action == GLFW_PRESS) {
					lastClickedRay = lastMovedRay;

					if(hoveredObjectiveID >= 0) {
						femMesh->removeObjective(hoveredObjectiveID);
						selectedObjectiveID = -1;
						hoveredObjectiveID = -1;
					}
					else if(hoveredNodeID >= 0) {
						selectedObjectiveID = femMesh->setNodePositionObjectiveNoDuplicate(hoveredNodeID, femMesh->nodes[hoveredNodeID]->getWorldPosition());
						femMesh->removeObjective(selectedObjectiveID);
						selectedObjectiveID = -1;
						hoveredObjectiveID = -1;
					}
					
				}
				return(true);
			}
			else {
				return(false);
			}
		}
		else if(interactionObject == InteractionObject::IKROBOT) {
			if(interactionMode == InteractionMode::SELECT) {
				return(true);
			}
			else if(interactionMode == InteractionMode::DRAG) {
				if(action == GLFW_PRESS) {
					selectedArmIk = 1;
					selectedIkPoint = left_gripper->getWorldCoordinates(static_cast<P3D>(mountBaseOriginRB_l));
					runIkSolver = true;
				}
				else if(action == GLFW_RELEASE) {
					selectedArmIk = -1;
					runIkSolver = false;
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
			if(selectedMountID >= 0) {
				std::cout << "Offsets: " << xOffset << " " << yOffset << std::endl;
				
				Plane plane(camera->getCameraTarget(),V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
				P3D origin; 
				currentRay.getDistanceToPlane(plane,&origin);
				dynamic_cast<RotationMount3D*>(femMesh->mounts[selectedMountID])->rotate(origin, yOffset * 0.05, 0.0, 0.0);
			}
			else if(selectedXi >= 0 && selectedXi < inverseDeformationSolver->xi.size()) {
				std::cout << "yOffset: " << yOffset << std::endl;
				
				inverseDeformationSolver->xi[selectedXi] += yOffset * 0.05;
				inverseDeformationSolver->pushXi();
			}
			else if(selectedGeneralizedRobotParameter >= 0 && selectedGeneralizedRobotParameter < 20) {
				RobotMount * m = dynamic_cast<RobotMount *>(femMesh->mounts[0]);
				if(m) {
					m->move(selectedGeneralizedRobotParameter, 0.05*yOffset);
				}
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

	return(false);
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



// Run the App tasks
void BenderApp3D::process() {
	//do the work here...

	Timer timer_simulation_one_frame;

	simulationTime = 0;
	maxRunningTime = 1.0 / desiredFrameRate;
	femMesh->checkDerivatives = false;

	//if we still have time during this frame, or if we need to finish the physics step, do this until the simulation time reaches the desired value
	while (simulationTime < 1.0 * maxRunningTime) {


	if(optimizeObjective) {	
			if(measure_convergence_time && (! timer_is_running))
			{
				timer_is_running = true;
				timer_convergence.restart();
			}
			if(timer_is_running) {i_step++;}
			
			
			//inverseDeformationSolver->objectiveFunction->setRegularizerValue(xiRegularizerValue);
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

			if (measure_convergence_time && timer_is_running && i_step == steps_optimization) {
			//if(measure_convergence_time && timer_is_running && i_step > 0 && std::fabs(delta_o) < timed_convergence_goal) {
				double t = timer_convergence.timeEllapsed();
				std::cout << "converged, time was:" << t << "s  (at step # " << i_step << ")" << std::endl;
				char c;
				std::cin >> c;
			}

			break;
		}
		else if(runIkSolver) {
			ikSolver->ikEnergyFunction->regularizer = 100;
			ikSolver->ikOptimizer->checkDerivatives = false;
			ikSolver->solve();
			//testGeneralizedCoordinateRepresentation(robot);
			// sync the parameters of the BenderApp
			generalizedRobotCoordinates->syncGeneralizedCoordinatesWithRobotState();
			//break;
		}
		if(!optimizeObjective) {
			inverseDeformationSolver->solveMesh(computeStaticSolution, simTimeStep);
			simulationTime += simTimeStep;
		}
	}

	// output diff begin/end of center line
	V3D delta_centerline = matchedFiber.back()->getWorldPosition() - matchedFiber.front()->getWorldPosition();
	std::cout << "diff centerline: " << delta_centerline(0) << " " << delta_centerline(1) << " " << delta_centerline(2) << std::endl;
	delta_centerline = matchedFiber[matchedFiber.size()/2]->getWorldPosition() - matchedFiber.front()->getWorldPosition();
	std::cout << "diff centerline_half: " << delta_centerline(0) << " " << delta_centerline(1) << " " << delta_centerline(2) << std::endl;

	

	if(synchronizePhysicalRobot) {
		if(robotControlInterface && robotControlInterface->isConnected()) {
			static double t_sum = 0;
			t_sum += timer_simulation_one_frame.timeEllapsed();
			if(t_sum >= 0.1) {
				robotControlInterface->syncPhysicalRobotWithSimRobot(t_sum);
				t_sum = 0.0;
			}
		}
		else {
			synchronizePhysicalRobot = false;
		}
	}

	
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void BenderApp3D::drawScene() {
	

	// draw mesh
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	//glColor3d(0.8,0.8,1);
	glColor3d(1,1,1);
	femMesh->drawSimulationMesh();

	// draw origin
	P3D p0(0.0, 0.0, 0.0);
	double l = 0.1;
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
	if(true) {
		for(int i = 0; i < (int)femMesh->pinnedNodeElements.size(); ++i) {
			MountedPointSpring<2> * mp = static_cast<MountedPointSpring<2> *>(femMesh->pinnedNodeElements[i]);

			double size = 0.001;
			P3D color(0.3, 0.3, 0.3);
			P3D white(1.0, 1.0, 1.0);
			P3D red(1.0, 1.0, 1.0);
			if(!mp->mount->parameterOptimization) {
				color = P3D(0.0, 0.0, 0.8);
			}
			if(!mp->mount->active) {
				color = color*0.5 + red*0.5;
			}
			if(selectedMountID >= 0 && mp->mount == femMesh->mounts[selectedMountID]) {
				size *= 1.7;
			}

			mp->draw(femMesh->x, size, color(0), color(1), color(2));
		}
	}

	

	// draw target trajectory
	if(targetTrajectory_input.getKnotCount() > 0) {
		targetTrajectory_input.draw(V3D(0.3, 0.3, 0.3), -2, V3D(0, 0.8, 0), 0.0025);
	}
	
	glEnable(GL_LIGHTING);
	// draw objectives
	for(int i = 0; i < (int)femMesh->objectives.size(); ++i) {
		MeshObjective * o = femMesh->objectives[i];

		MeshObjective::HighlightLevel level = MeshObjective::HighlightLevel::NONE;
		if(i == selectedObjectiveID) {
			level = MeshObjective::HighlightLevel::SELECTED;
		}
		else if(i == hoveredObjectiveID) {
			level = MeshObjective::HighlightLevel::HOVERED;
		}

		o->draw(femMesh->x, level);
	}


	
	// draw robot
	int flags = 0;
	if(showAbstract)
		flags |= SHOW_ABSTRACT_VIEW;
	if(highlightSelected)
		flags |= HIGHLIGHT_SELECTED;
	if (showMesh)
		flags |= SHOW_MESH;
	if (showMOI){
		flags |= SHOW_MOI_BOX;
		Logger::consolePrint("total mass: %lf\n", robot->mass);
	}
	if (showRotationAxes)
		flags |= SHOW_JOINTS;
	if (showCDPs)
		flags |= SHOW_CD_PRIMITIVES;

	//glPushMatrix();

	glEnable(GL_LIGHTING);

	// draw hovered-over node
	if(hoveredNodeID > 0) {
		// draw node
		glColor3d(1, 0.5, 0.0);
		drawSphere(femMesh->nodes[hoveredNodeID]->getWorldPosition(), 0.0015);
	}

	//// draw hovered-over node-objective
	//if(hoveredObjectiveID > 0) {
	//	NodePositionObjective * obj = dynamic_cast<NodePositionObjective *>(femMesh->objectives[hoveredObjectiveID]);
	//	if(obj) {
	//		// draw node
	//		glColor3d(1, 0.5, 0.2);
	//		drawSphere(obj->targetPosition, 0.0015);
	//	}
	//}

	
	rbEngine->drawRBs(flags);

	
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
	inverseDeformationSolver->parameterSets.push_back(new EulerRotationParameters);
	femMesh->addMount<RotationMount3D>(inverseDeformationSolver->parameterSets.back());
	inverseDeformationSolver->pullXi();
	updateMountSelectionBox();
}

void BenderApp3D::removeSelectedMount()
{
	std::cerr << "error: not properly implemented yet" << std::endl;
	exit(3);
	
	femMesh->removeMount(selectedMountID);
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
