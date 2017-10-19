#pragma once

#include <MathLib/MathLib.h>
#include <MathLib/P3D.h>
#include <MathLib/V3D.h>
#include <MathLib/MathLib.h>
#include <MathLib/Trajectory.h>
#include <ControlLib/Robot.h>
#include <ControlLib/GeneralizedCoordinatesRobotRepresentation.h>
#include <vector>
#include <map>

using namespace std;

class IK_EndEffector{
public:
	P3D targetEEPos;
	RigidBody* endEffectorRB;
	P3D endEffectorLocalCoords;

	IK_EndEffector(){
		endEffectorRB = NULL;
	}

};

/**
	This plan is for one specific moment in time - given targets for end effectors, COM and full-body state (with various weights to mimic regularizers or hard constraints), we need to compute the robot's joint angles
*/
class IK_Plan{
public:
	IK_Plan(Robot* robot);
	virtual ~IK_Plan(void);

	void setCurrentIKState(const ReducedRobotState& rs);
	void setTargetIKState(const ReducedRobotState& rs);
	void getCurrentIKState(ReducedRobotState& rs);

	void setCurrentIKStateFromRobot();
	void setTargetIKStateFromRobot();
	void setCurrentIKStateToRobot();

	void setTargetEEPos(int index, const P3D& pos);

	virtual void writeParametersToList(dVector& p){
		if (optimizeRootConfiguration)
			p = currentRobotState;
		else{
			p.resize(currentRobotState.size() - 6);
			for (int i = 0; i < p.size(); i++)
				p[i] = currentRobotState[i + 6];
		}
	}

	virtual void setParametersFromList(const dVector& p){
		if (optimizeRootConfiguration)
			currentRobotState = p;
		else
			for (int i = 0; i < p.size(); i++)
				currentRobotState[i + 6] = p[i];

		gcRobotRepresentation->setQ(currentRobotState);
	}

	void updateRobotRepresentation() {
		*gcRobotRepresentation = GeneralizedCoordinatesRobotRepresentation(robot);
	}

public:
	DynamicArray<IK_EndEffector> endEffectors;

	dVector targetRobotState;
	dVector currentRobotState;

	Robot* robot;
	GeneralizedCoordinatesRobotRepresentation* gcRobotRepresentation;

	bool optimizeRootConfiguration = true;
};

