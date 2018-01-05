#include <ControlLib/IK_Plan.h>
#include <ControlLib/BodyFrame.h>
#include <ControlLib/GenericLimb.h>

IK_Plan::IK_Plan(Robot* robot){
	this->robot = robot;
	this->gcRobotRepresentation = new GeneralizedCoordinatesRobotRepresentation(robot);
	setCurrentIKState(RobotState(robot));
	setTargetIKState(RobotState(robot));
}

IK_Plan::~IK_Plan(void){
	delete gcRobotRepresentation;
}

void IK_Plan::getCurrentIKState(RobotState& rs){
	gcRobotRepresentation->setQ(currentRobotState);
	gcRobotRepresentation->getReducedRobotState(rs);
}

void IK_Plan::setCurrentIKState(const RobotState& rs){
	gcRobotRepresentation->getQFromReducedState(rs, currentRobotState);
}

void IK_Plan::setTargetIKState(const RobotState& rs){
	gcRobotRepresentation->getQFromReducedState(rs, targetRobotState);
}

void IK_Plan::setTargetEEPos(int index, const P3D& pos){
	endEffectors[index].targetEEPos = pos;
}

void IK_Plan::setCurrentIKStateFromRobot() {
	RobotState rs(robot);
	setCurrentIKState(rs);
}

void IK_Plan::setTargetIKStateFromRobot() {
	RobotState rs(robot);
	setTargetIKState(rs);
}

void IK_Plan::setCurrentIKStateToRobot() {
	RobotState rs(robot);
	getCurrentIKState(rs);
	robot->setState(&rs);
}

