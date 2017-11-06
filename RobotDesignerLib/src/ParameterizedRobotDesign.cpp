#include <RobotDesignerLib/ParameterizedRobotDesign.h>
#include <GUILib/GLContentManager.h>

ParameterizedRobotDesign::ParameterizedRobotDesign(Robot* robot) : defaultRobotState(robot) {
	this->robot = robot;

	updateMorphology();
}

void ParameterizedRobotDesign::updateMorphology() {
	defaultRobotState = ReducedRobotState(robot);

	initialJointMorphology.clear();
	initialEEMorphology.clear();

	for (int i = 0; i < robot->getJointCount(); i++)
		initialJointMorphology[robot->getJoint(i)] = JointParameters(robot->getJoint(i));

	for (int i = 0; i < robot->getRigidBodyCount(); i++)
		for (uint j = 0; j < robot->getRigidBody(i)->rbProperties.endEffectorPoints.size(); j++)
			initialEEMorphology[&robot->getRigidBody(i)->rbProperties.endEffectorPoints[j]] = EEParameters(&robot->getRigidBody(i)->rbProperties.endEffectorPoints[j]);

}


ParameterizedRobotDesign::~ParameterizedRobotDesign() {
}
