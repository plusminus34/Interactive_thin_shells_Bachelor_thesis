
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


double Robot::getApproxBodyFrameHeight() {
	P3D comPos;
	double totalMass = 0;
	//this is the position of the "COM"
	for (uint k = 0; k < bFrame->bodyLinks.size(); k++) {
		totalMass += bFrame->bodyLinks[k]->rbProperties.mass;
		comPos += bFrame->bodyLinks[k]->getCMPosition() * bFrame->bodyLinks[k]->rbProperties.mass;
	}
	comPos /= totalMass;

	//and average position of the feet. We want to make sure they end up touching the ground
	double eeY = 0;
	int totalNEEs = 0;
	int nLegs = bFrame->limbs.size();
	for (int i = 0; i < nLegs; i++) {
		int nEEs = bFrame->limbs[i]->getLastLimbSegment()->rbProperties.getEndEffectorPointCount();
		for (int j = 0; j < nEEs; j++) {
			P3D eeLocalCoords = bFrame->limbs[i]->getLastLimbSegment()->rbProperties.getEndEffectorPoint(j);
			P3D eeWorldCoords = bFrame->limbs[i]->getLastLimbSegment()->getWorldCoordinates(eeLocalCoords);
			eeY += eeWorldCoords[1];
			totalNEEs++;
		}
	}
	if (totalNEEs > 0) eeY /= totalNEEs;
	return comPos[1] - eeY;
}

/**
	This method is used to populate the relative orientation of the parent and child bodies of joint i.
*/
void Robot::getRelativeOrientationForJoint(int i, Quaternion* qRel){
	//rotation from child frame to world, and then from world to parent == rotation from child to parent
	*qRel = jointList[i]->computeRelativeOrientation();
}

/**
	This method is used to get the relative angular velocities of the parent and child bodies of joint i,
	expressed in parent's local coordinates. 
	We'll assume that i is in the range 0 - jointList.size()-1!!!
*/
void Robot::getRelativeAngularVelocityForJoint(int i, V3D* wRel){
	*wRel = jointList[i]->child->state.angularVelocity - jointList[i]->parent->state.angularVelocity;
	//we will store wRel in the parent's coordinates, to get an orientation invariant expression for it
	*wRel = jointList[i]->parent->getLocalCoordinates(*wRel);
}


/**
	uses the state of the robot to populate the input
*/
void Robot::populateState(ReducedRobotState* state, bool useDefaultAngles) {
	//we'll push the root's state information - ugly code....
	state->setPosition(root->state.position);
	state->setOrientation(root->state.orientation);
	state->setVelocity(root->state.velocity);
	state->setAngularVelocity(root->state.angularVelocity);
	state->setHeadingAxis(Globals::worldUp);

	//now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
	//we are assuming here that each joint is revolute!!!
	Quaternion qRel;
	V3D wRel;

	for (uint i=0;i<jointList.size();i++){
		if (!useDefaultAngles){
			getRelativeOrientationForJoint(i, &qRel);
			state->setJointRelativeOrientation(qRel, i);
			getRelativeAngularVelocityForJoint(i, &wRel);
			state->setJointRelativeAngVelocity(wRel, i);
		}
		else {
			state->setJointRelativeOrientation(Quaternion(), i);
			state->setJointRelativeAngVelocity(V3D(), i);
			HingeJoint* hj = dynamic_cast<HingeJoint*>(jointList[i]);
			if (hj)
				state->setJointRelativeOrientation(getRotationQuaternion(hj->defaultAngle, hj->rotationAxis), i);
		}
	}
}

/**
	This method populates the state of the current robot with the values that are passed
	in the dynamic array. The same conventions as for the getState() method are assumed.
*/
void Robot::setState(ReducedRobotState* state){
	//kinda ugly code....
	root->state.position = state->getPosition();
	root->state.orientation = state->getOrientation();
	root->state.orientation.toUnit();
	root->state.velocity = state->getVelocity();
	root->state.angularVelocity = state->getAngularVelocity();

	//now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
	//we are assuming here that each joint is revolute!!!
	Quaternion qRel;
	V3D wRel;

	V3D r;
	V3D d;
	V3D vRel;

	for (size_t j=0;j<jointList.size();j++){
		qRel = state->getJointRelativeOrientation((int)j);
		qRel.toUnit();
		wRel = state->getJointRelativeAngVelocity((int)j);
		//transform the relative angular velocity to world coordinates
		wRel = jointList[j]->parent->getWorldCoordinates(wRel);

		//now that we have this information, we need to restore the state of the rigid body.

		//set the proper orientation
		jointList[j]->child->state.orientation = jointList[j]->parent->state.orientation * qRel;

		//and the proper angular velocity
		jointList[j]->child->state.angularVelocity = jointList[j]->parent->state.angularVelocity + wRel;
		//and now set the linear position and velocity
		jointList[j]->fixJointConstraints(true, true, true, true);
	}

//TODO: what should we do with the wheels here? Set them to zero angular speed? Or same as their parent?

	for (size_t j = 0; j<auxiliaryJointList.size(); j++)
		auxiliaryJointList[j]->fixJointConstraints(true, true, true, true);
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
	ReducedRobotState state(getReducedStateDimension());
	populateState(&state);
	state.setHeading(val);
	setState(&state);
}

/**
	this method is used to read the reduced state of the robot from the file
*/
void Robot::loadReducedStateFromFile(const char* fName){
	ReducedRobotState state(getReducedStateDimension());
	state.readFromFile(fName);
	setState(&state);
}

/**
	this method is used to write the reduced state of the robot to a file
*/
void Robot::saveReducedStateToFile(const char* fName){
	ReducedRobotState state(getReducedStateDimension());
	populateState(&state);
	state.writeToFile(fName, this);
}

/**
	This method is used to save the RBS corresponding to a virtual robot to file.
*/
void Robot::saveRBSToFile(char* fName){
	FILE* fp = fopen(fName, "w");

	//first need to write all the rigid bodies that belong to the articulated figure
	root->writeToFile(fp);
	for (uint i=0;i<jointList.size();i++)
		jointList[i]->child->writeToFile(fp);

	for (uint i=0;i<jointList.size();i++)
		jointList[i]->writeToFile(fp);

	fclose(fp);
}


void setupSimpleRobotStructure(Robot* robot) {
	for (int i = 0; i<robot->getJointCount(); i++) {
		if (robot->getJoint(i)->child->rbProperties.getEndEffectorPointCount() > 0) {
			//this rigid body needs to be the end segment of a limb... but now we must determine what's the parent of the limb...
			Joint* limbParentJoint = robot->getJoint(i);
			while (limbParentJoint->parent != robot->root && limbParentJoint->parent->cJoints.size() <= 1)
				limbParentJoint = limbParentJoint->parent->pJoints[0];
			GenericLimb* limb = new SimpleLimb(limbParentJoint->name.c_str(), limbParentJoint);
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

std::vector<RigidBody*> Robot::getEndEffectorRBs() {
	std::vector<RigidBody*> rbs;
	for (uint i = 0; i < (int)jointList.size(); i++) {
		if (jointList[i]->child->rbProperties.getEndEffectorPointCount() > 0)
			rbs.push_back(jointList[i]->child);
	}
	return rbs;
}

int Robot::getEndEffectorCount()
{
	return getEndEffectorRBs().size();
}

P3D Robot::getEndEffectorWorldPosition(int _i)
{
	std::vector<RigidBody*> eeBodies = getEndEffectorRBs();
	assert(_i<eeBodies.size());

	P3D eeLocal = eeBodies[_i]->rbProperties.endEffectorPoints[0].coords;
	P3D eeWorld = eeBodies[_i]->getWorldCoordinates(eeLocal);
	return eeWorld;
}

double Robot::getEndEffectorRadius(int _i)
{
	double r = 0.0;
	std::vector<RigidBody*> eeBodies = getEndEffectorRBs();
	assert(_i<eeBodies.size());

	SphereCDP* pSphere = (SphereCDP*)eeBodies[_i]->cdps[0];
	if(pSphere)
		r = pSphere->r;

	return r;
	//return eeBodies[_i]->rbProperties.endEffectorPoints[0].featureSize;
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

ReducedRobotState::ReducedRobotState(ReducedRobotState* start, ReducedRobotState* end, double t) {
	state = DynamicArray<double>(start->getStateSize());
	setPosition(start->getPosition() * (1 - t) + end->getPosition() * t);
	setVelocity(start->getVelocity() * (1 - t) + end->getVelocity() * t);
	setOrientation(start->getOrientation().sphericallyInterpolateWith(end->getOrientation(), t));
	setAngularVelocity(start->getAngularVelocity() * (1 - t) + end->getAngularVelocity() * t);
	int jCount = getJointCount();
	for (int i = 0; i < jCount; i++) {
		setJointRelativeAngVelocity(start->getJointRelativeAngVelocity(i) * (1 - t) + end->getJointRelativeAngVelocity(i) * t, i);
		setJointRelativeOrientation(start->getJointRelativeOrientation(i).sphericallyInterpolateWith(end->getJointRelativeOrientation(i), t), i);
	}
	headingAxis = start->headingAxis * (1 - t) + end->headingAxis * t;
	headingAxis.normalize();
}

void ReducedRobotState::writeToFile(const char* fName, Robot* robot) {
	if (fName == NULL)
		throwError("cannot write to a file whose name is NULL!");

	FILE* fp = fopen(fName, "w");

	if (fp == NULL)
		throwError("cannot open the file \'%s\' for reading...", fName);

	V3D velocity = getVelocity();
	Quaternion orientation = getOrientation();
	V3D angVelocity = getAngularVelocity();
	P3D position = getPosition();

	double heading = getHeading();
	//setHeading(0);

	fprintf(fp, "# order is:\n# Heading Axis\n# Heading\n# Position\n# Orientation\n# Velocity\n# AngularVelocity\n\n# Relative Orientation\n# Relative Angular Velocity\n#----------------\n\n# Heading Axis\n %lf %lf %lf\n# Heading\n%lf\n\n", headingAxis[0], headingAxis[1], headingAxis[2], heading);

	if (robot != NULL)
		fprintf(fp, "# Root(%s)\n", robot->root->name.c_str());

	fprintf(fp, "%lf %lf %lf\n", position[0], position[1], position[2]);
	fprintf(fp, "%lf %lf %lf %lf\n", orientation.s, orientation.v[0], orientation.v[1], orientation.v[2]);
	fprintf(fp, "%lf %lf %lf\n", velocity[0], velocity[1], velocity[2]);
	fprintf(fp, "%lf %lf %lf\n\n", angVelocity[0], angVelocity[1], angVelocity[2]);

	int nJoints = getJointCount();
	for (int i = 0; i < nJoints; i++) {
		orientation = getJointRelativeOrientation(i);
		angVelocity = getJointRelativeAngVelocity(i);
		if (robot != NULL)
			fprintf(fp, "# %s\n", robot->jointList[i]->name.c_str());
		fprintf(fp, "%lf %lf %lf %lf\n", orientation.s, orientation.v[0], orientation.v[1], orientation.v[2]);
		fprintf(fp, "%lf %lf %lf\n\n", angVelocity[0], angVelocity[1], angVelocity[2]);
	}

	fclose(fp);
	//now restore the state of this reduced state...
	//setHeading(heading);
}


bool ReducedRobotState::isSameAs(const ReducedRobotState& other) {
	if (getStateSize() != other.getStateSize())
		return false;

	if (V3D(getPosition(), other.getPosition()).length() > TINY)
		return false;

	if ((getVelocity() - other.getVelocity()).length() > TINY)
		return false;

	if ((getAngularVelocity() - other.getAngularVelocity()).length() > TINY)
		return false;

	for (int i = 0; i < getJointCount(); i++) {
		if ((getJointRelativeAngVelocity(i) - other.getJointRelativeAngVelocity(i)).length() > TINY)
			return false;

		Quaternion q1 = getJointRelativeOrientation(i);
		Quaternion q2 = other.getJointRelativeOrientation(i);

		if (q1 != q2 && q1 != (q2 * -1)) {
			//				Logger::consolePrint("%lf %lf %lf %lf <-> %lf %lf %lf %lf\n", q1.s, q1.v.x(), q1.v.y(), q1.v.z(), q2.s, q2.v.x(), q2.v.y(), q2.v.z());
			return false;
		}
	}

	return true;
}

void ReducedRobotState::readFromFile(const char* fName) {
	if (fName == NULL)
		throwError("cannot read a file whose name is NULL!");

	FILE* fp = fopen(fName, "r");
	if (fp == NULL)
		throwError("cannot open the file \'%s\' for reading...", fName);

	double temp1, temp2, temp3, temp4;

	char line[100];

	//read the heading first...
	double heading;
	readValidLine(line, 100, fp);
	sscanf(line, "%lf %lf %lf", &headingAxis[0], &headingAxis[1], &headingAxis[2]);

	readValidLine(line, 100, fp);
	sscanf(line, "%lf", &heading);

	readValidLine(line, 100, fp);
	sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
	setPosition(P3D(temp1, temp2, temp3));
	readValidLine(line, 100, fp);
	sscanf(line, "%lf %lf %lf %lf", &temp1, &temp2, &temp3, &temp4);
	setOrientation(Quaternion(temp1, temp2, temp3, temp4).toUnit());
	readValidLine(line, 100, fp);
	sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
	setVelocity(V3D(temp1, temp2, temp3));
	readValidLine(line, 100, fp);
	sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
	setAngularVelocity(V3D(temp1, temp2, temp3));

	int jCount = getJointCount();
	for (int i = 0; i<jCount; i++) {
		readValidLine(line, 100, fp);
		sscanf(line, "%lf %lf %lf %lf", &temp1, &temp2, &temp3, &temp4);
		setJointRelativeOrientation(Quaternion(temp1, temp2, temp3, temp4).toUnit(), i);
		readValidLine(line, 100, fp);
		sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
		setJointRelativeAngVelocity(V3D(temp1, temp2, temp3), i);
	}

	//now set the heading...
	setHeading(heading);

	fclose(fp);
}


//setting the heading...
void ReducedRobotState::setHeading(double heading) {
	//this means we must rotate the angular and linear velocities of the COM, and augment the orientation
	Quaternion oldHeading, newHeading, qRoot;
	//get the current root orientation, that contains information regarding the current heading
	qRoot = getOrientation();
	//get the twist about the vertical axis...
	oldHeading = computeHeading(qRoot, headingAxis);
	//now we cancel the initial twist and add a new one of our own choosing
	newHeading = getRotationQuaternion(heading, headingAxis) * oldHeading.getComplexConjugate();
	//add this component to the root.
	setOrientation(newHeading * qRoot);
	//and also update the root velocity and angular velocity
	setVelocity(newHeading.rotate(getVelocity()));
	setAngularVelocity(newHeading.rotate(getAngularVelocity()));
}
