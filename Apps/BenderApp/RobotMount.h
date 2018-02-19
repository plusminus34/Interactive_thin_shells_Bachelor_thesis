#pragma once
#include <iostream>

#include "ControlLib/GeneralizedCoordinatesRobotRepresentation.h"
#include "MathLib/P3D.h"

#include "Mount.h"

#include "ParameterSet.h"

class RBSphereCollisionObjective;
class IDCustomYuMiControlInterface;


class RobotMount : public Mount {

public:
	RigidBody * robotPart;

public:
	RobotMount(ParameterSet * parameters);
	RobotMount(ParameterSet * parameters, RigidBody * robotPart);

	virtual P3D transformation(P3D const & x0, ParameterSet * parameters_in);
	virtual void dxDpar(P3D const & x0, ParameterSet * parameters_in, std::vector<V3D> & grad);

	void move(int parameter_idx, double dp);

};



class RobotParameters : public ParameterSet {
	friend RobotMount;
	friend RBSphereCollisionObjective;
	friend IDCustomYuMiControlInterface;

protected:
	GeneralizedCoordinatesRobotRepresentation * robotParameters;
	bool robot_is_synced = false;
	std::vector<double> offsetFullRevolutions;

public:
	RobotParameters(GeneralizedCoordinatesRobotRepresentation * robotParameters);

	virtual void writeToList(dVector & par, int & cursor_idx_io);
	virtual void setFromList(dVector const & par, int & cursor_idx_io);
	virtual int getNPar() const {return(robotParameters->getDimensionCount()-6);}
	virtual std::pair<double, double> getParameterLimitsByLocalIdx(int idx);


	void syncRobotStateWithParameters();
	bool is_synced_with_robot() {return(robot_is_synced);}
};

