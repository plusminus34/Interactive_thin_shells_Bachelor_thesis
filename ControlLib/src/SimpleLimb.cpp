#include <ControlLib/SimpleLimb.h>

/**
	constructor
*/
SimpleLimb::SimpleLimb(const char *limbName, Joint *_rootJoint) {
	name.assign(limbName);
	this->rootJoint = _rootJoint;
	origin = rootJoint->parent;
	initializeJointList();
}

/**
	destructor
*/
SimpleLimb::~SimpleLimb(void) {

}

/**
	returns the length of the limb
*/
double SimpleLimb::getLength() {
	return limbLength;
}

/**
	returns the first segment of the limb
*/
RigidBody* SimpleLimb::getFirstLimbSegment() {
	return jointList[0]->child;
}

/**
	returns the last segment of the limb (before the foot/hand, if there is one)
*/
RigidBody* SimpleLimb::getLastLimbSegment() {
	return jointList.back()->child;
}

/**
	this method is used to collect all the joint links of the leg to the list...
*/
void SimpleLimb::initializeJointList() {
	jointList.clear();
	jointList.push_back(rootJoint);
	limbLength = 0;

	while (jointList.back()->child->cJoints.size() > 0) {
		jointList.push_back(jointList.back()->child->cJoints[0]);
		limbLength += V3D(jointList[jointList.size()-2]->cJPos, jointList[jointList.size() - 1]->pJPos).length();
	}

	//now add the length of the last part of the leg...
	limbLength += V3D(jointList[jointList.size() - 1]->cJPos, jointList.back()->child->rbProperties.getEndEffectorPosition()).length();
}
