#pragma warning(disable : 4996)

#include <RobotDesignerLib/IntelligentRobotEditingWindow.h>
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/LocomotionEngineManagerIP.h>
#include <RBSimLib/ODERBEngine.h>

//maybe this editing window should just extend mopt, and then we create it directly (or not for "normal" execution...)


//move joints. When ALT is down (or the one that is same as in design window!) then move just joint and/or everything lower down in hierarchy
//resize bones. When ALT is down, move only child. Otherwise move both the child and parent joints...

IntelligentRobotEditingWindow::IntelligentRobotEditingWindow(int x, int y, int w, int h, RobotDesignerApp* rdApp) : GLWindow3D(x, y, w, h) {
	this->rdApp = rdApp;

	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.25;
	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -1.5;

	dynamic_cast<GLTrackingCamera*>(this->camera)->setCameraTarget(P3D(0, -0.25, 0));

	tWidget = new TranslateWidget(AXIS_X | AXIS_Y | AXIS_Z);
	tWidget->visible = false;

}

void IntelligentRobotEditingWindow::addMenuItems() {

}

IntelligentRobotEditingWindow::~IntelligentRobotEditingWindow(){
	delete tWidget;
}

void IntelligentRobotEditingWindow::drawAuxiliarySceneInfo(){
	glClear(GL_DEPTH_BUFFER_BIT);

	preDraw();
	postDraw();

	glClear(GL_DEPTH_BUFFER_BIT);
}

//triggered when using the mouse wheel
bool IntelligentRobotEditingWindow::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (highlightedRigidBody) {
		if (highlightedRigidBody->pJoints.size() == 1 && (highlightedRigidBody->cJoints.size() > 0 || highlightedRigidBody->rbProperties.endEffectorPoints.size() > 0)){
			ReducedRobotState rs(rdApp->robot);
			rdApp->robot->setState(&rdApp->prd->defaultRobotState);

			P3D p1 = highlightedRigidBody->pJoints[0]->getWorldPosition();
			P3D p2;

			if (highlightedRigidBody->cJoints.size() == 1){
				p2 = highlightedRigidBody->cJoints[0]->getWorldPosition();
			} else {
				for (auto& eeItr : highlightedRigidBody->rbProperties.endEffectorPoints){
					p2 += highlightedRigidBody->getWorldCoordinates(eeItr.coords);
				}
				p2 /= highlightedRigidBody->rbProperties.endEffectorPoints.size();
			}

			V3D vec = V3D(p1, p2) * ((100 + yOffset) / 100.0);
			P3D midPoint = (p1 + p2) / 2;
			DynamicArray<double> currentDesignParameters;
			rdApp->prd->getCurrentSetOfParameters(currentDesignParameters);

//			Logger::consolePrint("offset: %lf, vec: %lf %lf %lf\n", (100 + yOffset) / 100.0, vec[0], vec[1], vec[2]);

			V3D offset1(p1, midPoint + vec  * -0.5);
			V3D offset2(p2, midPoint + vec * 0.5);

			offset1 = highlightedRigidBody->getLocalCoordinates(offset1);
			offset2 = highlightedRigidBody->getLocalCoordinates(offset2);

			if (glfwGetKey(this->rdApp->glfwWindow, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) {
				//scale the selected bone by moving just the child... and propagate all changes throughout the offspring hierarchy...
	
				//adapt the child coords of the parent joint
				int pStartIndex = rdApp->prd->jointParamMap[highlightedRigidBody->pJoints[0]].index;
				for (int i = 0; i < 3; i++)
					currentDesignParameters[pStartIndex + 3 + i] += offset1[i];

				//and now, the parent coords of the child joint
				if (highlightedRigidBody->cJoints.size() == 1) {
					int pStartIndex = rdApp->prd->jointParamMap[highlightedRigidBody->cJoints[0]].index;
					for (int i = 0; i < 3; i++)
						currentDesignParameters[pStartIndex + i] += offset2[i];
				}
				else {
					int pStartIndex = rdApp->prd->eeParamMap[highlightedRigidBody].index;
					for (int i = 0; i < 3; i++)
						currentDesignParameters[pStartIndex + i] += offset2[i];
				}
			}
			else {
				//rescaling the bone with as few global changes as possible

				int pStartIndex = rdApp->prd->jointParamMap[highlightedRigidBody->pJoints[0]].index;

				//for the parent joint, adjust its coordinates in the child (e.g. highlighted) rigid body frame
				for (int i = 0; i < 3;i++)
					currentDesignParameters[pStartIndex + 3 + i] += offset1[i];

				//and then, for the same joint, adjust its coordinates in the frame of the parent rigid body...
				V3D offset1P = highlightedRigidBody->pJoints[0]->parent->getLocalCoordinates(highlightedRigidBody->getWorldCoordinates(offset1));
				for (int i = 0; i < 3; i++)
					currentDesignParameters[pStartIndex + i] += offset1P[i];

				if (highlightedRigidBody->cJoints.size() == 1){

					//adjust the coordinates of the child joint, both in coord frame of its parent and child rigid bodies...
					int pStartIndex = rdApp->prd->jointParamMap[highlightedRigidBody->cJoints[0]].index;
					for (int i = 0; i < 3; i++)
						currentDesignParameters[pStartIndex + i] += offset2[i];

					V3D offset2P = highlightedRigidBody->cJoints[0]->child->getLocalCoordinates(highlightedRigidBody->getWorldCoordinates(offset2));
					for (int i = 0; i < 3; i++)
						currentDesignParameters[pStartIndex + 3 + i] += offset2P[i];
				}
				else {
					int pStartIndex = rdApp->prd->eeParamMap[highlightedRigidBody].index;
					for (int i = 0; i < 3; i++)
						currentDesignParameters[pStartIndex + i] += offset2[i];
				}
			}

			rdApp->robot->setState(&rs);
			rdApp->prd->setParameters(currentDesignParameters);

		}

		return true;
	}

	return GLWindow3D::onMouseWheelScrollEvent(xOffset, yOffset);
}

bool IntelligentRobotEditingWindow::onMouseMoveEvent(double xPos, double yPos){
	if (Robot* robot = rdApp->robot) {
		preDraw();
		Ray ray = getRayFromScreenCoords(xPos, yPos);
		postDraw();

		if (tWidget->visible) {
			if (tWidget->onMouseMoveEvent(xPos, yPos) == true) {
//				selectedFP->coords = tWidget->pos;
			}
		}
		else {
			ReducedRobotState rs(robot);
			robot->setState(&rdApp->prd->defaultRobotState);

			//check if any of the joints are highlighted...
			for (int i = 0; i < robot->getJointCount(); i++)
				robot->getJoint(i)->selected = false;

			highlightedJoint = NULL;
			double tMinJ = DBL_MAX;

			for (int i = 0; i < robot->getJointCount(); i++) {
				P3D p;
				double dist = ray.getDistanceToPoint(robot->getJoint(i)->getWorldPosition(), &p);
				double tVal = ray.getRayParameterFor(p);
				if (dist < robot->getJoint(i)->parent->abstractViewCylinderRadius * 1.2 && tVal < tMinJ) {
					tMinJ = tVal;
					highlightedJoint = robot->getJoint(i);
				}
			}

			for (int i = 0; i < robot->getRigidBodyCount(); i++)
				for (uint j = 0; j < robot->getRigidBody(i)->rbProperties.endEffectorPoints.size(); j++)
					robot->getRigidBody(i)->rbProperties.endEffectorPoints[j].selected = false;

			highlightedEE = NULL;
			double tMinEE = DBL_MAX;

			for (int i = 0; i < robot->getRigidBodyCount(); i++)
				for (uint j = 0; j < robot->getRigidBody(i)->rbProperties.endEffectorPoints.size(); j++) {
					P3D p;
					double dist = ray.getDistanceToPoint(robot->getRigidBody(i)->getWorldCoordinates(robot->getRigidBody(i)->rbProperties.endEffectorPoints[j].coords), &p);
					double tVal = ray.getRayParameterFor(p);
					if (dist < robot->getRigidBody(i)->abstractViewCylinderRadius * 1.2 && tVal < tMinEE) {
						tMinEE = tVal;
						highlightedEE = &robot->getRigidBody(i)->rbProperties.endEffectorPoints[j];
					}
				}



			for (int i = 0; i < robot->getRigidBodyCount(); i++)
				robot->getRigidBody(i)->selected = false;

			highlightedRigidBody = NULL;

			P3D pLocal;
			double tMinB = DBL_MAX;
			for (int i = 0; i < robot->getRigidBodyCount(); i++) {
				if (robot->getRigidBody(i)->getRayIntersectionPointTo(ray, &pLocal)) {
					double tVal = ray.getRayParameterFor(robot->getRigidBody(i)->getWorldCoordinates(pLocal));

					if (tVal < tMinB) {
						tMinB = tVal;
						highlightedRigidBody = robot->getRigidBody(i);
					}
				}
			}

			if (highlightedJoint && highlightedRigidBody)
				if (highlightedJoint->parent == highlightedRigidBody || highlightedJoint->child == highlightedRigidBody) {
					highlightedRigidBody = NULL;
					tMinB = DBL_MAX;
				}

			if (highlightedEE && highlightedRigidBody)
				for (uint j = 0; j < highlightedRigidBody->rbProperties.endEffectorPoints.size(); j++)
					if (&highlightedRigidBody->rbProperties.endEffectorPoints[j] == highlightedEE) {
						highlightedRigidBody = NULL;
						tMinB = DBL_MAX;
						break;
					}

			if (highlightedJoint && (tMinJ > tMinB || tMinJ > tMinEE))
				highlightedJoint = NULL;

			if (highlightedEE && (tMinEE > tMinB || tMinEE > tMinJ))
				highlightedEE = NULL;

			if (highlightedRigidBody && (tMinB > tMinEE || tMinB > tMinJ))
				highlightedRigidBody = NULL;

			if (highlightedJoint)
				highlightedJoint->selected = true;

			if (highlightedEE)
				highlightedEE->selected = true;

			if (highlightedRigidBody)
				highlightedRigidBody->selected = true;

			robot->setState(&rs);

		}

	}


	return GLWindow3D::onMouseMoveEvent(xPos, yPos);
}

bool IntelligentRobotEditingWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos){
	int shiftDown = glfwGetKey(this->rdApp->glfwWindow, GLFW_KEY_LEFT_SHIFT);
	if (GlobalMouseState::lButtonPressed && shiftDown == GLFW_PRESS) {
		return true;
	}

	tWidget->visible = false;

	//if a joint or EE is selected, turn tWidget on and sync it ...



	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void IntelligentRobotEditingWindow::setViewportParameters(int posX, int posY, int sizeX, int sizeY){
	GLWindow3D::setViewportParameters(posX, posY, sizeX, sizeY);
}

void IntelligentRobotEditingWindow::drawScene() {
	glColor3d(1, 1, 1);
	glDisable(GL_LIGHTING);
	drawDesignEnvironmentBox(GLContentManager::getTexture("../data/textures/ground_TileLight2.bmp"));
	glEnable(GL_LIGHTING);

	if (!rdApp->robot)
		return;

	int flags = SHOW_ABSTRACT_VIEW | SHOW_BODY_FRAME | SHOW_JOINTS | HIGHLIGHT_SELECTED;

	ReducedRobotState rs(rdApp->robot);
	rdApp->robot->setState(&rdApp->prd->defaultRobotState);

	glEnable(GL_LIGHTING);
	if (rdApp->simWindow->rbEngine)
		rdApp->simWindow->rbEngine->drawRBs(flags);

/*
	for (int i = 0; i < rdApp->robot->getJointCount(); i++)
		drawSphere(rdApp->prd->initialJointMorphology[rdApp->robot->getJoint(i)].worldCoords, 0.015);

	if (highlightedRigidBody && highlightedRigidBody->pJoints.size() == 1 && (highlightedRigidBody->cJoints.size() > 0 || highlightedRigidBody->rbProperties.endEffectorPoints.size() > 0)){
		P3D p1 = highlightedRigidBody->pJoints[0]->getWorldPosition();
		P3D p2;
		if (highlightedRigidBody->cJoints.size() == 1)
			p2 = highlightedRigidBody->cJoints[0]->getWorldPosition();
		else {
			for (auto& eeItr : highlightedRigidBody->rbProperties.endEffectorPoints){
				p2 += highlightedRigidBody->getWorldCoordinates(eeItr.coords);
			}
			p2 /= highlightedRigidBody->rbProperties.endEffectorPoints.size();
		}

		drawSphere(p1, 0.015);
		drawSphere(p2, 0.015);

	}
*/

	rdApp->robot->setState(&rs);
}

void IntelligentRobotEditingWindow::setupLights() {
	GLfloat bright[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	GLfloat mediumbright[] = { 0.3f, 0.3f, 0.3f, 1.0f };

	glLightfv(GL_LIGHT1, GL_DIFFUSE, bright);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, mediumbright);
	glLightfv(GL_LIGHT3, GL_DIFFUSE, mediumbright);
	glLightfv(GL_LIGHT4, GL_DIFFUSE, mediumbright);


	GLfloat light0_position[] = { 0.0f, 10000.0f, 10000.0f, 0.0f };
	GLfloat light0_direction[] = { 0.0f, -10000.0f, -10000.0f, 0.0f };

	GLfloat light1_position[] = { 0.0f, 10000.0f, -10000.0f, 0.0f };
	GLfloat light1_direction[] = { 0.0f, -10000.0f, 10000.0f, 0.0f };

	GLfloat light2_position[] = { 0.0f, -10000.0f, 0.0f, 0.0f };
	GLfloat light2_direction[] = { 0.0f, 10000.0f, -0.0f, 0.0f };

	GLfloat light3_position[] = { 10000.0f, -10000.0f, 0.0f, 0.0f };
	GLfloat light3_direction[] = { -10000.0f, 10000.0f, -0.0f, 0.0f };

	GLfloat light4_position[] = { -10000.0f, -10000.0f, 0.0f, 0.0f };
	GLfloat light4_direction[] = { 10000.0f, 10000.0f, -0.0f, 0.0f };


	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
	glLightfv(GL_LIGHT2, GL_POSITION, light2_position);
	glLightfv(GL_LIGHT3, GL_POSITION, light3_position);
	glLightfv(GL_LIGHT4, GL_POSITION, light4_position);


	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light0_direction);
	glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, light1_direction);
	glLightfv(GL_LIGHT2, GL_SPOT_DIRECTION, light2_direction);
	glLightfv(GL_LIGHT3, GL_SPOT_DIRECTION, light3_direction);
	glLightfv(GL_LIGHT4, GL_SPOT_DIRECTION, light4_direction);


	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glEnable(GL_LIGHT3);
	glEnable(GL_LIGHT4);
}

