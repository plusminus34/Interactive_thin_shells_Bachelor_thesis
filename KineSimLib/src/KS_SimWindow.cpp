#pragma warning(disable : 4996)

//window3d and application should both extend some base class that has camera, knows how to draw ground, reflections, etc...


#include <KineSimLib/KS_SimWindow.h>


SimWindow::SimWindow(int x, int y, int w, int h, GLApplication* glApp) : GLWindow3D(x, y, w, h) {
	this->glApp = glApp;

	simTimeStep = 1;
	//	Globals::g = 0;

	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.25;
	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -1.5;

	showReflections = true;
	showGroundPlane = true;
}

void SimWindow::addMenuItems() {
	
}

SimWindow::~SimWindow(){
	clear();
}

void SimWindow::clear(){

}

void SimWindow::reset(){
	
}

void SimWindow::drawAuxiliarySceneInfo(){
	glClear(GL_DEPTH_BUFFER_BIT);

	preDraw();
	postDraw();

	glClear(GL_DEPTH_BUFFER_BIT);
}

bool SimWindow::onMouseMoveEvent(double xPos, double yPos){
	
	return GLWindow3D::onMouseMoveEvent(xPos, yPos);
}

bool SimWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos){

	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void SimWindow::setViewportParameters(int posX, int posY, int sizeX, int sizeY){

	GLWindow3D::setViewportParameters(posX, posY, sizeX, sizeY);
}

KS_MechanicalAssembly* SimWindow::loadMechanism(const char* fName) {
	delete mechanism;
	mechanism = new KS_MechanicalAssembly();
	mechanism->readFromFile("../data/KineSimApp/fourBar.mech");
	return mechanism;
}

void SimWindow::loadController() {
	delete UIController;
	
	//UIController = new KS_UIMechanismController(mechanism);

}

void SimWindow::drawScene() {
	glEnable(GL_LIGHTING);

	P3D p0(0.0, 0.0, 0.0);
	double l = 0.2;
	//double dtheta = 0.1;
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
	drawSphere(p0, l / 16);
	mechanism->draw();
}


void SimWindow::doPhysicsStep(double simStep) {
	/*activeController->applyControlSignals(simStep);
	rbEngine->applyForceTo(robot->root, perturbationForce * forceScale, P3D());
	rbEngine->step(simStep);
	robot->bFrame->updateStateInformation();

	//integrate forward in time the motion of the weels...
	for (uint j = 0; j < activeController->motionPlan->endEffectorTrajectories.size(); j++) {
		LocomotionEngine_EndEffectorTrajectory* eeTraj = &activeController->motionPlan->endEffectorTrajectories[j];
		if (eeTraj->isWheel) {
			RigidBody* rb = eeTraj->endEffectorRB;
			int eeIndex = eeTraj->CPIndex;
			int meshIndex = rb->rbProperties.endEffectorPoints[eeIndex].meshIndex;
			Joint* wheelJoint = rb->rbProperties.endEffectorPoints[eeIndex].wheelJoint;

			if (meshIndex >= 0 && wheelJoint)
				rb->meshTransformations[meshIndex].R = wheelJoint->computeRelativeOrientation().getRotationMatrix() * rb->rbProperties.endEffectorPoints[eeIndex].initialMeshTransformation.R;
		}
	}*/
}

bool SimWindow::advanceSimulation(double dt) {
	if (!activeController)
		return false;

	/*if (activeController == kinematicController || activeController == pololuMaestroController){
		activeController->computeControlSignals(dt);
		activeController->applyControlSignals(dt);
		motionPhaseReset = activeController->advanceInTime(dt);
	}
	else {
		double simulationTime = 0;

		while (simulationTime < dt) {
			simulationTime += simTimeStep;

			activeController->computeControlSignals(simTimeStep);
			doPhysicsStep(simTimeStep);

			motionPhaseReset = activeController->advanceInTime(simTimeStep) || motionPhaseReset;
//			break;
		}
	}*/

	return true;//motionPhaseReset=
}
