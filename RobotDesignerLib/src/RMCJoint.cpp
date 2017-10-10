#include <GUILib/GLUtils.h>
#include <RobotDesignerLib/RMCJoint.h>
#include <Utils/Utils.h>

RMCJoint::RMCJoint(){
}

RMCJoint::RMCJoint(RMCPin* parentPin, RMCPin* childPin, const vector<Transformation>& transformations, int transIndex)
{
	this->parent = parentPin->rmc;
	this->child = childPin->rmc;
	parent->cJoints.push_back(this);
	child->pJoints.push_back(this);
	this->transformations = transformations;
	curTransIndex = transIndex;

	this->parentPin = parentPin;
	this->childPin = childPin;
	parentPin->idle = childPin->idle = false;
	parentPin->joint = childPin->joint = this;
}

RMCJoint::RMCJoint(RMC* parent, RMC* child, const vector<Transformation>& transformations, int transIndex)
{
	this->parent = parent;
	this->child = child;
	parent->cJoints.push_back(this);
	child->pJoints.push_back(this);
	this->transformations = transformations;
	curTransIndex = transIndex;
}

RMCJoint::~RMCJoint(void){

}

/**
	Projects the orientation of the child onto the constraint manifold that satisfies the type of joint this is
*/
void RMCJoint::fixRMCConstraint(bool ignoreMotorAngle) {
	if (curTransIndex >= (int)transformations.size()) return;

	Transformation trans;
	if (parentPin && childPin) {
		Transformation parentTrans = parentPin->transformation;
		Transformation childTrans = childPin->transformation;

		if (!ignoreMotorAngle)
		{
			if (getParent()->type == MOTOR_RMC && parentPin->type == HORN_PIN)
			{
				RMC* parentRMC = getParent();
				parentTrans *= Transformation(getRotationQuaternion(RAD(parentRMC->motorAngle), parentRMC->motorAxis).getRotationMatrix());
			}
			if (getChild()->type == MOTOR_RMC && childPin->type == HORN_PIN)
			{
				RMC* childRMC = getChild();
				childTrans *= Transformation(getRotationQuaternion(RAD(childRMC->motorAngle), childRMC->motorAxis).getRotationMatrix());
			}
		}

		trans = parentTrans * transformations[curTransIndex] * childTrans.inverse();
	}
	else
		trans = transformations[curTransIndex];

	Matrix3x3 rot = parent->state.orientation.getRotationMatrix() * trans.R;
	child->state.orientation.setRotationFrom(rot);
	child->state.position = parent->state.position + parent->state.orientation.rotate(trans.T);
}


void RMCJoint::fixRMCConstraintInverse()
{
	if (curTransIndex >= (int)transformations.size()) return;

	Transformation trans;
	if (parentPin && childPin) {
		Transformation parentTrans = parentPin->transformation;
		Transformation childTrans = childPin->transformation;
		trans = parentTrans * transformations[curTransIndex] * childTrans.inverse();
	}
	else
		trans = transformations[curTransIndex];

	trans = trans.inverse();

	Matrix3x3 rot = child->state.orientation.getRotationMatrix() * trans.R;
	parent->state.orientation.setRotationFrom(rot);
	parent->state.position = child->state.position + child->state.orientation.rotate(trans.T);
}

/**
	writes the joint info to file...
*/
void RMCJoint::writeToFile(FILE* fp) {
	
}


void RMCJoint::switchToNextTransformation() {
	curTransIndex = (curTransIndex + 1) % (int)transformations.size();
	//Logger::print("%d", curTransIndex);
}

