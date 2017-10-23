#include <RobotDesignerLib/ParameterizedRobotDesign.h>
#include <GUILib/GLContentManager.h>

ParameterizedRobotDesign::ParameterizedRobotDesign(Robot* robot) {
	this->robot = robot;

	for (int i = 0; i < robot->getJointCount(); i++)
		initialMorphology.push_back(JointParameters(robot->getJoint(i)));
}

ParameterizedRobotDesign::~ParameterizedRobotDesign() {
}
