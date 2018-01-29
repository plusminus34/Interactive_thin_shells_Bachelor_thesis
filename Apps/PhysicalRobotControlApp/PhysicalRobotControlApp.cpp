#include <GUILib/GLUtils.h>

#include "PhysicalRobotControlApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <RBSimLib/ODERBEngine.h>
#include <ControlLib/SimpleLimb.h>
#include <ControlLib/PololuServoControlInterface.h>
#include <ControlLib/YuMiControlInterface.h>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

PhysicalRobotControlApp::PhysicalRobotControlApp() {
	setWindowTitle("Physical Robot Control");

    loadFile("../data/rbs/yumi/yumi.rbs");

    showMesh = true;
    showRotationAxes = false;

	//desiredFrameRate = 5;

	mainMenu->addGroup("IK App Visualization options");

	mainMenu->addVariable("Draw Meshes", showMesh);
	mainMenu->addVariable("Draw Joints", showRotationAxes);

	mainMenu->addGroup("Motor Controller");

	nanogui::Widget *tools = new nanogui::Widget(mainMenu->window());
	mainMenu->addWidget("", tools);
    tools->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 4));

	nanogui::Button* button = new nanogui::Button(tools, "");
    button->setCallback([this]() {
        if (rci && rci->isConnected() == false) {
			std::cout << "openCommunicationPort" << std::endl;
            rci->openCommunicationPort();

			RobotState rs(robot);

			//File path to load home position
			struct passwd *pw = getpwuid(getuid());
			std::string homeDir = pw->pw_dir;
			std::string homePath = homeDir + homeFilePath;
			const char* homeStatePath = homePath.c_str();

			// Option 1: always start from home position
			if(startAtHomeState){
				//std::cout << "start at home state" << std::endl;
				rs.readFromFile(homeStatePath);
				robot->setState(&rs);
				ikSolver->ikPlan->setTargetIKStateFromRobot();

				if(syncPhysicalRobot){
					rci->syncPhysicalRobotWithSimRobot();
				}
			} else { // Option 2: Read robot state from physical YuMi and start simulation from it...
				rci->syncSimRobotWithPhysicalRobot();
				ikSolver->ikPlan->setTargetIKStateFromRobot();
			}

			//Option to save home file
			if(saveCurrentAsHomePosition){
				rs.writeToFile(homeStatePath);
			}
        }
        else {
            if (rci) {
				std::cout << "closeCommunicationPort" << std::endl;
                rci->closeCommunicationPort();
            }
        }
    });
	button->setIcon(ENTYPO_ICON_CYCLE);
	button->setTooltip("Connect/Disconnect");

	button = new nanogui::Button(tools, "");
	button->setCallback([this]() { if (rci && rci->isConnected()) rci->toggleMotorPower(); });
	button->setIcon(ENTYPO_ICON_POWER_PLUG);
	button->setTooltip("Motors on/off");

	button = new nanogui::Button(tools, "");
	//button->setCallback([this]() { if (rci && rci->isConnected()) rci->driveMotorPositionsToZero(); });
	button->setCallback([this]() { if (rci) rci->driveMotorPositionsToTestPos1(ikSolver); });
	button->setIcon(ENTYPO_ICON_HOME);
	button->setTooltip("GoToZero");

    button = new nanogui::Button(tools, "");
	//button->setCallback([this]() { if (rci && rci->isConnected()) rci->driveMotorPositionsToTestPos(); });
	button->setCallback([this]() { if (rci) rci->driveMotorPositionsToTestPos2(ikSolver); });
    button->setIcon(ENTYPO_ICON_HOME);
    button->setTooltip("GoToTestPos");

//	mainMenu->addVariable("Follow Trajectory", playFFTrajectory);
	mainMenu->addVariable("duration", trajDuration)->setSpinnable(true);
	mainMenu->addVariable("control positions only", controlPositionsOnly);
    mainMenu->addVariable("sync physical robot", syncPhysicalRobot);
	mainMenu->addVariable("request position", requestPosition);
	mainMenu->addVariable("speed", speed)->setSpinnable(true);


	mainMenu->addGroup("Gripper Controller");

	nanogui::Widget *toolsGrip = new nanogui::Widget(mainMenu->window());
	mainMenu->addWidget("", toolsGrip);
	toolsGrip->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 4));

	button = new nanogui::Button(toolsGrip, "");
	button->setCallback([this]() { if (rci) rci->grip("left"); });
	button->setIcon(ENTYPO_ICON_ALIGN_LEFT);
	button->setTooltip("Open / Close LEFT gripper");

	button = new nanogui::Button(toolsGrip, "");
	button->setCallback([this]() { if (rci) rci->grip("right"); });
	button->setIcon(ENTYPO_ICON_ALIGN_RIGHT);
	button->setTooltip("Open / Close RIGHT gripper");

	menuScreen->performLayout();

	showGroundPlane = false;
}

void PhysicalRobotControlApp::loadRobot(const char* fName) {
	delete robot;
	delete rbEngine;
//	delete poseSolver;

	rbEngine = new ODERBEngine();
	rbEngine->loadRBsFromFile(fName);
	robot = new Robot(rbEngine->rbs[0]);
	startState = RobotState(robot);
	setupSimpleRobotStructure(robot);

	delete ikSolver;
	ikSolver = new IK_Solver(robot, true);

	//TODO: we will need a much better way of setting motor parameters...
	for (int i = 0; i < robot->getJointCount(); i++) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;

		hj->motor.motorID = i;

		if (i == 0) {
			//settings for the BK DS-3002HV
			hj->motor.pwmMin = 910;//depends on the type of servomotor
			hj->motor.pwmMax = 2090;//depends on type of servomotor
			hj->motor.pwmFor0Deg = 1430; //this depends on how the horn is mounted...
			hj->motor.pwmFor45Deg = 1870; //this depends on how the horn is mounted...
		}

		if (i == 1) {
			//settings for the TURNIGY S306G-HV
			hj->motor.pwmMin = 910;//depends on the type of servomotor
			hj->motor.pwmMax = 2100;//depends on type of servomotor
			hj->motor.pwmFor0Deg = 1430; //this depends on how the horn is mounted...
			hj->motor.pwmFor45Deg = 1865; //this depends on how the horn is mounted...
  //			hj->motor.flipMotorAxis = true;
		}

		if (i == 2) {
			//settings for the MKS DS95
			hj->motor.pwmMin = 800;//depends on the type of servomotor
			hj->motor.pwmMax = 2160;//depends on type of servomotor
			hj->motor.pwmFor0Deg = 1390; //this depends on how the horn is mounted...
			hj->motor.pwmFor45Deg = 1935; //this depends on how the horn is mounted...
 //			hj->motor.flipMotorAxis = true;
		}

	}

	delete rci;
    rci = new YuMiControlInterface(robot);
}

void PhysicalRobotControlApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);

	if (fNameExt.compare("rbs") == 0)
		loadRobot(fName);
	if (fNameExt.compare("rs") == 0) {
		if (robot) {
			robot->loadReducedStateFromFile(fName);
			startState = RobotState(robot);
			ikSolver->ikPlan->setTargetIKStateFromRobot();
		}
	}
}

PhysicalRobotControlApp::~PhysicalRobotControlApp(void) {
//	delete poseSolver;
	delete rbEngine;
	delete robot;
}

// Restart the application.
void PhysicalRobotControlApp::restart() {
    loadFile("../data/rbs/yumi/yumi.rbs");
}

// Run the App tasks
Timer t;

void PhysicalRobotControlApp::process() {

	ikSolver->ikEnergyFunction->regularizer = 150; //Default: 100
	ikSolver->ikOptimizer->checkDerivatives = true;
	ikSolver->solve();

	//rci->printJointValues();

    if (rci && syncPhysicalRobot) {
		//updateSpeedParameter();

		//double dt = 1.0 / desiredFrameRate;
		double dt = t.timeEllapsed();
        rci->controlPositionsOnly = controlPositionsOnly;
		t.restart();
        rci->syncPhysicalRobotWithSimRobot(dt);
		t.timeEllapsed();
		Logger::consolePrint("It's been %lf s since timer restart\n", t.timeEllapsed());

    }
}

//triggered when mouse moves
bool PhysicalRobotControlApp::onMouseMoveEvent(double xPos, double yPos) {
	Ray ray = getRayFromScreenCoords(xPos, yPos);

	if (robot && selectedRigidBody == NULL){
		for (int i = 0; i < robot->getRigidBodyCount(); i++)
			robot->getRigidBody(i)->selected = false;

		highlightedRigidBody = NULL;
		P3D pLocal;
		double tMin = DBL_MAX;
		for (int i = 0; i < robot->getRigidBodyCount(); i++)
			if (robot->getRigidBody(i)->getRayIntersectionPointTo(ray, &pLocal)) {
				double tVal = ray.getRayParameterFor(robot->getRigidBody(i)->getWorldCoordinates(pLocal));
				if (tVal < tMin) {
					tMin = tVal;
					highlightedRigidBody = robot->getRigidBody(i);
				}
			}
		if (highlightedRigidBody){
			highlightedRigidBody->selected = true;
			Logger::consolePrint("highlighted rb %s, positioned at %lf %lf %lf\n", highlightedRigidBody->name.c_str(), highlightedRigidBody->getCMPosition().x(), highlightedRigidBody->getCMPosition().y(), highlightedRigidBody->getCMPosition().z());
		}
	}

	if (selectedRigidBody) {
		V3D viewPlaneNormal = V3D(camera->getCameraPosition(), camera->getCameraTarget()).unit();
		ray.getDistanceToPlane(Plane(selectedRigidBody->getWorldCoordinates(selectedPoint), viewPlaneNormal), &targetPoint);
//		ray.getDistanceToPoint(selectedRigidBody->getWorldCoordinates(selectedPoint), &targetPoint);
		ikSolver->ikPlan->endEffectors.back().targetEEPos = targetPoint;
		return true;
	}

	if (GLApplication::onMouseMoveEvent(xPos, yPos)) return true;

	return false;
}

//triggered when mouse buttons are pressed
bool PhysicalRobotControlApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {

	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		selectedRigidBody = highlightedRigidBody;
		if (selectedRigidBody) {
			selectedRigidBody->getRayIntersectionPointTo(getRayFromScreenCoords(xPos, yPos), &selectedPoint);
			getRayFromScreenCoords(xPos, yPos).getDistanceToPoint(selectedRigidBody->getWorldCoordinates(selectedPoint), &targetPoint);
			ikSolver->ikPlan->endEffectors.push_back(IK_EndEffector());
			ikSolver->ikPlan->endEffectors.back().endEffectorLocalCoords = selectedPoint;
			ikSolver->ikPlan->endEffectors.back().endEffectorRB = selectedRigidBody;
			ikSolver->ikPlan->endEffectors.back().targetEEPos = targetPoint;
		}
	}

	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE){
		ikSolver->ikPlan->endEffectors.clear();
		selectedRigidBody = NULL;
	}

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	return false;
}

//triggered when using the mouse wheel
bool PhysicalRobotControlApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool PhysicalRobotControlApp::onKeyEvent(int key, int action, int mods) {

	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool PhysicalRobotControlApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;
	return false;
}

void PhysicalRobotControlApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void PhysicalRobotControlApp::drawScene() {
	glColor3d(1, 1, 1);
	glDisable(GL_LIGHTING);

	int flags = SHOW_ABSTRACT_VIEW | HIGHLIGHT_SELECTED;
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

	glEnable(GL_LIGHTING);
	glPushMatrix();

	rbEngine->drawRBs(flags);

//	RobotState rs(robot);
//	if (rci && requestPosition)
//		rci->syncSimRobotWithPhysicalRobot();
//	glTranslated(0, 0, 1);
//	rbEngine->drawRBs(flags);
//	robot->setState(&rs);

//	drawSphere(robot->getRBByName("link_7_l")->getWorldCoordinates(P3D(0.017977, -0.0169495, 0.01949)), 0.05);
//	drawSphere(robot->getRBByName("link_7_r")->getWorldCoordinates(P3D(0.0200485, -0.0189025, -0.0217355)), 0.05);

	glPopMatrix();
	glDisable(GL_LIGHTING);

	if (selectedRigidBody){
		glColor3d(1,0,0);
		glBegin(GL_LINES);
			gl_Vertex3d(selectedRigidBody->getWorldCoordinates(selectedPoint));
			gl_Vertex3d(targetPoint);
		glEnd();
	}
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void PhysicalRobotControlApp::drawAuxiliarySceneInfo() {

}

bool PhysicalRobotControlApp::processCommandLine(const std::string& cmdLine) {
	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

void PhysicalRobotControlApp::updateSpeedParameter(){
//	HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(0));
//	hj->motor.targetYuMiTCPSpeed = speed;
}


