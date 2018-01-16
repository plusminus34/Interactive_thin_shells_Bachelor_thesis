#pragma once
#include <iostream>

#include "ControlLib/GeneralizedCoordinatesRobotRepresentation.h"
#include "MathLib/P3D.h"

#include "Mount.h"

#include "ParameterSet.h"



class RobotMount : public Mount {

public:
	RigidBody * robotPart;

public:
	RobotMount(ParameterSet * parameters);

	virtual P3D transformation(P3D const & x0, ParameterSet * parameters_in);
	//virtual void dxDpar(P3D const & x0, ParameterSet * parameters_in, std::vector<V3D> & grad);


};




class RobotParameters : public ParameterSet {

public:
	GeneralizedCoordinatesRobotRepresentation * robotParameters;
	bool robot_is_synced = false;

public:
	RobotParameters(GeneralizedCoordinatesRobotRepresentation * robotParameters) : robotParameters(robotParameters) {};

	virtual void writeToList(dVector & par, int & cursor_idx_io);
	virtual void setFromList(dVector & par, int & cursor_idx_io);
	virtual int getNPar() const {return(robotParameters->getDimensionCount()-6);}
};

