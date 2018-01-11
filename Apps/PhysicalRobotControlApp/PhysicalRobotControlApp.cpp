#include <GUILib/GLUtils.h>

#include "PhysicalRobotControlApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <RBSimLib/ODERBEngine.h>
#include <ControlLib/SimpleLimb.h>
#include <ControlLib/PololuServoControlInterface.h>
#include <ControlLib/YuMiControlInterface.h>

PhysicalRobotControlApp::PhysicalRobotControlApp() {
	setWindowTitle("Physical Robot Control");

    loadFile("../data/rbs/yumi/yumi.rbs");

    showMesh = true;
    showRotationAxes = false;

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
            rci->openCommunicationPort();

//            // Option 1: always start from home position
//            if(startAtHomePosition){
//                RobotState rs(robot);
//                rs.readFromFile( from home );
//                robot->setState(&rs);
//                ikSolver->ikPlan->setTargetIKStateFromRobot();
//                rci->syncPhysicalRobotWithSimRobot();
//            } else { // Option 2: Read robot state from physical YuMi and start simulation from it...
//                RobotState rs(robot);
//                rci->syncSimRobotWithPhysicalRobot();
//                ikSolver->ikPlan->setTargetIKStateFromRobot();
//            }

//            //Option to save home file
//            if(saveCurrentAsHomePosition){
//                RobotState newState(robot);
//                RobotState.writeToFile();
//            }

//            robot->setState(&rs);
        }
        else {
            if (rci) {
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
	button->setCallback([this]() { if (rci && rci->isConnected()) rci->driveMotorPositionsToZero(); });
	button->setIcon(ENTYPO_ICON_HOME);
	button->setTooltip("GoToZero");

    button = new nanogui::Button(tools, "");
    button->setCallback([this]() { if (rci && rci->isConnected()) rci->driveMotorPositionsToTestPos(); });
    button->setIcon(ENTYPO_ICON_HOME);
    button->setTooltip("GoToTestPos");

//	mainMenu->addVariable("Follow Trajectory", playFFTrajectory);
	mainMenu->addVariable("duration", trajDuration)->setSpinnable(true);
	mainMenu->addVariable("control positions only", controlPositionsOnly);
    mainMenu->addVariable("sync physical robot", syncPhysicalRobot);

	menuScreen->performLayout();

	showGroundPlane = false;

//    int nPts = 10;
//    for (int i = 0; i < nPts; i++) {
//        double t = (double)i / (nPts - 1);
//        FFTrajectory.addKnot(t, sin(2 * M_PI * t) * RAD(45));
//    }
//    FFTrajectory.addKnot(1, 0);

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
void PhysicalRobotControlApp::process() {
	double dt = 1.0 / desiredFrameRate;

    ikSolver->ikEnergyFunction->regularizer = 100;
    ikSolver->ikOptimizer->checkDerivatives = true;
    ikSolver->solve();

    if (rci && syncPhysicalRobot) {
        rci->controlPositionsOnly = controlPositionsOnly;
        rci->syncPhysicalRobotWithSimRobot(dt);
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

    RobotState rs(robot);
    if (rci)
        rci->syncSimRobotWithPhysicalRobot();
    glTranslated(0, 0, 1);
    rbEngine->drawRBs(flags);
    robot->setState(&rs);

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

