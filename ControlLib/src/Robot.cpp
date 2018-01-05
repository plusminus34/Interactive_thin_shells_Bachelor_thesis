
#include <ControlLib/Robot.h>
#include <stdio.h>
#include <ControlLib/SimpleLimb.h>

#include <RBSimLib/HingeJoint.h>
#include <RBSimLib/UniversalJoint.h>
#include <RBSimLib/BallAndSocketJoint.h>
#include <RBSimLib/AbstractRBEngine.h>

/**
	the constructor
*/
Robot::Robot(RigidBody* root){
	this->root = root;
	if (root == NULL)
		throwError("Can't build a robot without a root!\n");

	//gather the list of jointList for the robot
	jointList.clear();
	DynamicArray<RigidBody*> bodies;
	bodies.push_back(root);

	while (bodies.size()>0) {
		if (bodies[0]->pJoints.size() > 1) 
			throwError("Possible kinematic loop detected in robot definition. Not currently allowed...\n");
		//add all the children jointList to the list
		for (uint i = 0;i<bodies[0]->cJoints.size();i++) {
			jointList.push_back(bodies[0]->cJoints[i]);
			bodies.push_back(bodies[0]->cJoints[i]->child);
		}
		bodies.erase(bodies.begin());
	}

	//index the jointList properly...
	for (uint i = 0;i<jointList.size();i++)
		jointList[i]->jIndex = i;

	//compute the mass of the robot
	mass = root->rbProperties.mass;
	for (uint i = 0; i < jointList.size(); i++)
		mass += jointList[i]->child->rbProperties.mass;

	bFrame = new BodyFrame(this);
}

/**
	the destructor
*/
Robot::~Robot(void){
	delete bFrame;
}

/**
	uses the state of the robot to populate the input
*/
void Robot::populateState(RobotState* state, bool useDefaultAngles) {
	//we'll push the root's state information - ugly code....
	state->setPosition(root->state.position);
	state->setOrientation(root->state.orientation);
	state->setVelocity(root->state.velocity);
	state->setAngularVelocity(root->state.angularVelocity);
	state->setHeadingAxis(Globals::worldUp);

	state->setJointCount(jointList.size());
	state->setAuxiliarJointCount(auxiliaryJointList.size());

	//now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
	//we are assuming here that each joint is revolute!!!

	for (uint i=0;i<jointList.size();i++){
		if (!useDefaultAngles){
			state->setJointRelativeOrientation(getRelativeOrientationForJoint(jointList[i]), i);
			state->setJointRelativeAngVelocity(getRelativeAngularVelocityForJoint(jointList[i]), i);
		}
		else {
			state->setJointRelativeOrientation(Quaternion(), i);
			state->setJointRelativeAngVelocity(V3D(), i);
			HingeJoint* hj = dynamic_cast<HingeJoint*>(jointList[i]);
			if (hj)
				state->setJointRelativeOrientation(getRotationQuaternion(hj->defaultAngle, hj->rotationAxis), i);
		}
	}

	for (uint i = 0; i<auxiliaryJointList.size(); i++) {
		if (!useDefaultAngles) {
			state->setAuxiliaryJointRelativeOrientation(getRelativeOrientationForJoint(auxiliaryJointList[i]), i);
			state->setAuxiliaryJointRelativeAngVelocity(getRelativeAngularVelocityForJoint(auxiliaryJointList[i]), i);
		}
		else {
			state->setAuxiliaryJointRelativeOrientation(Quaternion(), i);
			state->setAuxiliaryJointRelativeAngVelocity(V3D(), i);
		}
	}
}

/**
	This method populates the state of the current robot with the values that are passed
	in the dynamic array. The same conventions as for the getState() method are assumed.
*/
void Robot::setState(RobotState* state){
	//kinda ugly code....
	root->state.position = state->getPosition();
	root->state.orientation = state->getOrientation();
	root->state.orientation.toUnit();
	root->state.velocity = state->getVelocity();
	root->state.angularVelocity = state->getAngularVelocity();

	//now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
	//we are assuming here that each joint is revolute!!!
	for (uint j=0;j<jointList.size();j++){
		setRelativeOrientationForJoint(jointList[j], state->getJointRelativeOrientation((int)j).toUnit());
		setRelativeAngularVelocityForJoint(jointList[j], state->getJointRelativeAngVelocity((int)j));
		//and now set the linear position and velocity
		jointList[j]->fixJointConstraints(true, true, true, true);
	}

	for (uint j = 0; j<auxiliaryJointList.size(); j++) {
		setRelativeOrientationForJoint(auxiliaryJointList[j], state->getAuxiliaryJointRelativeOrientation((int)j).toUnit());
		setRelativeAngularVelocityForJoint(auxiliaryJointList[j], state->getAuxiliaryJointRelativeAngVelocity((int)j));
		//and now set the linear position and velocity
		auxiliaryJointList[j]->fixJointConstraints(true, true, true, true);
	}

}

/**
	makes sure the state of the robot is consistent with all the joint types...
*/
void Robot::fixJointConstraints() {
	for (size_t j = 0; j<jointList.size(); j++)
		jointList[j]->fixJointConstraints(true, true, true, true);

	for (size_t j = 0; j<auxiliaryJointList.size(); j++)
		auxiliaryJointList[j]->fixJointConstraints(true, true, true, true);
}

/**
	This method is used to compute the center of mass of the articulated figure.
*/
P3D Robot::computeCOM(){
	P3D COM = root->getCMPosition() * root->rbProperties.mass;
	double totalMass = root->rbProperties.mass;

	for (uint i=0; i <jointList.size(); i++){
		totalMass += jointList[i]->child->rbProperties.mass;
		COM += jointList[i]->child->getCMPosition() * jointList[i]->child->rbProperties.mass;
	}

	return COM / totalMass;
}

/**
	This method is used to compute the velocity of the center of mass of the articulated figure.
*/
V3D Robot::computeCOMVelocity(){
	V3D COMVel = root->getCMVelocity() * root->rbProperties.mass;
	double totalMass = root->rbProperties.mass;

	for (uint i=0; i <jointList.size(); i++){
		totalMass += jointList[i]->child->rbProperties.mass;
		COMVel += jointList[i]->child->getCMVelocity() * jointList[i]->child->rbProperties.mass;
	}

	return COMVel / totalMass;
}

/**
	this method is used to rotate the robot (well, the robot whose state is passed in as a parameter) 
	about the vertical axis, so that it's default heading has the value that is given as a parameter
*/
void Robot::setHeading(double val){
	RobotState state(this);
	populateState(&state);
	state.setHeading(val);
	setState(&state);
}

/**
	this method is used to read the reduced state of the robot from the file
*/
void Robot::loadReducedStateFromFile(const char* fName){
	RobotState state(this);
	state.readFromFile(fName);
	setState(&state);
}

/**
	this method is used to write the reduced state of the robot to a file
*/
void Robot::saveReducedStateToFile(const char* fName){
	RobotState state(this);
	state.writeToFile(fName, this);
}

void setupSimpleRobotStructure(Robot* robot) {
	for (int i = 0; i<robot->getJointCount(); i++) {
		if (robot->getJoint(i)->child->rbProperties.getEndEffectorPointCount() > 0) {
			//this rigid body needs to be the end segment of a limb...
			GenericLimb* limb = new SimpleLimb(robot->getJoint(i)->child->name.c_str(), robot->getJoint(i)->child, robot->getRoot());
			robot->bFrame->addLimb(limb);
		}
	}
	// for (uint i = 0; i<robot->jointList.size(); i++)
		// robot->bFrame->addBodyLink(robot->jointList[i]->child);
}

/**
this method is used to return a reference to the articulated figure's rigid body whose name is passed in as a parameter,
or NULL if it is not found.
*/
RigidBody* Robot::getRBByName(const char* jName) {
	for (uint i = 0; i<jointList.size(); i++) {
		if (strcmp(jointList[i]->parent->name.c_str(), jName) == 0)
			return jointList[i]->parent;
		if (strcmp(jointList[i]->child->name.c_str(), jName) == 0)
			return jointList[i]->child;
	}
	return NULL;
}

void Robot::addWheelsAsAuxiliaryRBs(AbstractRBEngine* rbEngine) {
	for (int i = 0; i < getRigidBodyCount(); i++) {
		RigidBody* rb = getRigidBody(i);
		for (uint j = 0; j < rb->rbProperties.endEffectorPoints.size(); j++) {
			RBEndEffector* ee = &(rb->rbProperties.endEffectorPoints[j]);
			if (ee->isActiveWheel() || ee->isFreeToMoveWheel()) {
				//must create a new joint and a new rigid body...
				RigidBody* wheelRB = new RigidBody();
				double radius = ee->featureSize;
				V3D axis = ee->localCoordsWheelAxis;
				double volume = 2 * PI * radius * radius * 0.01;
				wheelRB->rbProperties.mass = volume * 500;
				wheelRB->rbProperties.MOI_local.coeffRef(0, 0) = wheelRB->rbProperties.MOI_local.coeffRef(1, 1) = wheelRB->rbProperties.MOI_local.coeffRef(2, 2) = 2.0/5.0 * wheelRB->rbProperties.mass * radius * radius;
				wheelRB->name = rb->name + "_wheel_" + std::to_string(j);
				wheelRB->cdps.push_back(new SphereCDP(P3D(), radius));
				wheelRB->state.orientation = rb->state.orientation;

				HingeJoint* newJoint = new HingeJoint();
				newJoint->rotationAxis = ee->localCoordsWheelAxis;
				newJoint->child = wheelRB;
				newJoint->cJPos = P3D();
				newJoint->parent = rb;
				newJoint->pJPos = ee->coords;
				newJoint->child->pJoints.push_back(newJoint);
				newJoint->parent->cJoints.push_back(newJoint);
				newJoint->name = newJoint->child->name + "_" + newJoint->parent->name;

				newJoint->fixJointConstraints(true, false, false, false);

				if (ee->isActiveWheel())
					newJoint->controlMode = VELOCITY_MODE;
				else
					newJoint->controlMode = PASSIVE;

				rbEngine->addRigidBodyToEngine(wheelRB);
				rbEngine->addJointToEngine(newJoint);

				auxiliaryJointList.push_back(newJoint);
				ee->wheelJoint = newJoint;
				if (ee->meshIndex >= 0)
					ee->initialMeshTransformation = rb->meshTransformations[ee->meshIndex];
			}
		}
	}
}
