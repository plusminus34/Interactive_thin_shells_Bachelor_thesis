#include <GUILib/GLUtils.h>
#include <ControlLib/BodyFrame.h>
#include <ControlLib/GenericLimb.h>
#include <RBSimLib/RigidBody.h>
#include <ControlLib/Robot.h>

BodyFrame::BodyFrame(Robot* robot){
	this->robot = robot;
	addBodyLink(robot->getRoot());
}

BodyFrame::~BodyFrame(void){
}

// adds a limb to the body frame
void BodyFrame::addLimb(GenericLimb* l){
	addBodyLink(l->origin);
	limbs.push_back(l);
	l->parentBodyFrame = this;
}

// adds a new rigid body to the bodyframe
void BodyFrame::addBodyLink(RigidBody* rb){
	if (rb == NULL) return;
	//make sure the body does not get added to the list multiple times...
	if (isPartOfBodyFrame(rb))
		return;

	bodyLinks.push_back(rb);

	//update the total mass, as well as the list of internal joints, and joints to the outside...
	totalMass = 0;
	for (uint i=0; i<bodyLinks.size();i++)
		totalMass += bodyLinks[i]->rbProperties.mass;

	jointsToTheOutside.clear();
	internalJoints.clear();
	//go through the child joints of all the body links, and if they do not lead to another link that's already in the body frame, then add them to the "joints to the outside" list
	for (uint i=0; i<bodyLinks.size();i++){
		for (uint j=0; j<bodyLinks[i]->cJoints.size();j++){
			RigidBody* child = bodyLinks[i]->cJoints[j]->child;
			if (isPartOfBodyFrame(child) == false)
				jointsToTheOutside.push_back(bodyLinks[i]->cJoints[j]);
			else
				internalJoints.push_back(bodyLinks[i]->cJoints[j]);			}
		}
	//make sure everything is updated whenever we modify the structure of the body frame
	updateStateInformation();
}

// returns the closest rigid body on the body frame for rb
RigidBody* BodyFrame::getClosestBodyFrameLinkTo(RigidBody* rb){
	RigidBody* result = rb;

	while (isPartOfBodyFrame(result) == false && result != NULL) {
		if (result->pJoints.size() > 0)
			result = result->pJoints[0]->parent;
		else
			result = NULL;
	}

	return result;
}

// returns the index of the rb in the list of links making up the bodyframe, or -1 if it doesn't belong to it
int BodyFrame::getRBIndex(RigidBody* rb){
	for (uint i=0; i<bodyLinks.size();i++)
		if (bodyLinks[i] == rb)
			return i;
	return -1;
}

// returns true if the rb is a part of the body frame
bool BodyFrame::isPartOfBodyFrame(RigidBody* rb){
	return getRBIndex(rb) != -1;
}

// returns true if the joint belongs to the set of internal joints
bool BodyFrame::isInternalJoint(Joint* j){
	for (uint i=0;i<internalJoints.size();i++)
		if (internalJoints[i] == j)
			return true;
	return false;
}

// updates the contact status of the limbs based on contacts at the limb's end effectors
void BodyFrame::updateLimbGroundContactInformation() {
	for (uint i = 0; i < limbs.size(); i++)
		limbs[i]->inContact = limbs[i]->getLastLimbSegment()->inContact;
}

// updates the state and heading of the body frame
void BodyFrame::updateStateInformation(){
	bodyState = RBState();
	bodyMomentOfInertia.setZero();

	bodyState.orientation.s = 0;
	bodyState.orientation.v.setZero();

	//todo: we're using linear interpolation for the quaternions. Not ideal, but probably will do the trick. Revisit at some point nevertheless.
	for (uint i=0; i<bodyLinks.size();i++){
		double w = bodyLinks[i]->rbProperties.mass/totalMass;
		bodyState.position += V3D(bodyLinks[i]->state.position) * w;
		//make sure we avoid the q and -q problem
		if (bodyLinks[i]->state.orientation.s > 0)
			bodyState.orientation += bodyLinks[i]->state.orientation * w;
		else
			bodyState.orientation += bodyLinks[i]->state.orientation * -w;

		bodyState.angularVelocity += bodyLinks[i]->state.angularVelocity * w;
		bodyState.velocity += bodyLinks[i]->state.velocity * w;
	}

	bodyState.orientation.toUnit();
	bodyHeading = computeHeading(bodyState.orientation, Globals::worldUp);
	
	//now estimate the moment of inertia of the body - sum up the mois of the different rigid bodies, computed about the CM position of the body frame
	for (uint i=0; i<bodyLinks.size(); i++)
		bodyMomentOfInertia += bodyLinks[i]->getWorldMOIAboutPoint(bodyState.position);

	//now compute the local coordinates of the moment of inertia, which may be needed for planning or other things...
	//local to world R = bodyState.orientation.getRotationMatrix();
	//world MOI is (R * I_local * R'), so I_local = R' * I * R
	bodyMomentOfInertia_local = bodyState.orientation.getComplexConjugate().getRotationMatrix() * bodyMomentOfInertia * bodyState.orientation.getRotationMatrix();
}


//draws the body frame
void BodyFrame::drawBodyFrame(){
	glPushMatrix();

	glTranslated(bodyState.position[0], bodyState.position[1], bodyState.position[2]);
	//and rotation part
	V3D rotAxis; double rotAngle;
	bodyState.orientation.getAxisAngle(rotAxis, rotAngle);
	glRotated(DEG(rotAngle), rotAxis[0], rotAxis[1], rotAxis[2]);


	//draw a coordinate frame at the center of the body frame
	glColor3d(1, 0, 0);
	drawArrow(P3D(), P3D() + Globals::worldUp * 0.2, 0.01);
	glColor3d(0, 1, 0);
	drawArrow(P3D(), P3D() + robot->right * 0.2, 0.01);
	glColor3d(0, 0, 1);
	drawArrow(P3D(), P3D() + robot->forward * 0.2, 0.01);

	//draw the MOI box for the body frame - the moment of inertia we have currently is in world coords, so we need to approximate the local one
	drawMOIApproximation(bodyMomentOfInertia_local, totalMass);
	glPopMatrix();	
}


P3D BodyFrame::getBodyFrameCoordsForWorldFramePoint(const P3D& wfPoint){
	//assume the bodyState.position is the origin of the body frame, of course
	return P3D(getBodyFrameCoordsForWorldFrameVector(V3D(bodyState.position, wfPoint)));
}

P3D BodyFrame::getHeadingFrameCoordsForBodyFramePoint(const P3D& bfPoint){
	//assume the bodyState.position is the origin of the body frame, of course
	return P3D(getHeadingFrameCoordsForBodyFrameVector(V3D(bfPoint)));
}

P3D BodyFrame::getWorldFrameCoordsForBodyFramePoint(const P3D& bfPoint){
	//assume the bodyState.position is the origin of the body frame, of course
	return bodyState.position + getWorldFrameCoordsForBodyFrameVector(V3D(bfPoint));
}

P3D BodyFrame::getHeadingFrameCoordsForWorldFramePoint(const P3D& wfPoint){
	return P3D(getHeadingFrameCoordsForWorldFrameVector(V3D(bodyState.position, wfPoint)));
}

P3D BodyFrame::getWorldFrameCoordsForHeadingFramePoint(const P3D& hfPoint){
	return bodyState.position + getWorldFrameCoordsForHeadingFrameVector(V3D(hfPoint));
}

V3D BodyFrame::getBodyFrameCoordsForWorldFrameVector(const V3D& wfVector){
	return bodyState.orientation.inverseRotate(wfVector);
}

V3D BodyFrame::getWorldFrameCoordsForBodyFrameVector(const V3D& bfVector){
	return bodyState.orientation.rotate(bfVector);
}

V3D BodyFrame::getHeadingFrameCoordsForBodyFrameVector(const V3D& bfVector){
	return getHeadingFrameCoordsForWorldFrameVector(getWorldFrameCoordsForBodyFrameVector(bfVector));
}

V3D BodyFrame::getHeadingFrameCoordsForWorldFrameVector(const V3D& wfVector){
	return bodyHeading.inverseRotate(wfVector);
}

V3D BodyFrame::getWorldFrameCoordsForHeadingFrameVector(const V3D& hfVector){
	return bodyHeading.rotate(hfVector);
}


