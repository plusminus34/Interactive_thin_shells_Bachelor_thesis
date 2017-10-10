#include <ControlLib/IK_Plan.h>
#include <ControlLib/BodyFrame.h>
#include <ControlLib/GenericLimb.h>

IK_Plan::IK_Plan(Robot* robot){
	this->robot = robot;
	this->gcRobotRepresentation = new GeneralizedCoordinatesRobotRepresentation(robot);
	setCurrentIKState(ReducedRobotState(robot));
	setTargetIKState(ReducedRobotState(robot));
}

IK_Plan::~IK_Plan(void){
	delete gcRobotRepresentation;
}

void IK_Plan::getCurrentIKState(ReducedRobotState& rs){
	gcRobotRepresentation->setQ(currentRobotState);
	gcRobotRepresentation->getReducedRobotState(rs);
}

void IK_Plan::setCurrentIKState(const ReducedRobotState& rs){
	gcRobotRepresentation->getQFromReducedState(rs, currentRobotState);
}

void IK_Plan::setTargetIKState(const ReducedRobotState& rs){
	gcRobotRepresentation->getQFromReducedState(rs, targetRobotState);
}

void IK_Plan::setTargetEEPos(int index, const P3D& pos){
	endEffectors[index].targetEEPos = pos;
}

void IK_Plan::setCurrentIKStateFromRobot() {
	ReducedRobotState rs(robot);
	setCurrentIKState(rs);
}

void IK_Plan::setTargetIKStateFromRobot() {
	ReducedRobotState rs(robot);
	setTargetIKState(rs);
}

void IK_Plan::setCurrentIKStateToRobot() {
	ReducedRobotState rs(robot);
	getCurrentIKState(rs);
	robot->setState(&rs);
}

