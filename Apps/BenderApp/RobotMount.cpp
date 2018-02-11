
#include <vector>
#include <cmath>
#include <iostream>

#include "MathLib/P3D.h"
#include "RBSimLib/HingeJoint.h"

#include "RobotMount.h"



RobotMount::RobotMount(ParameterSet * parameters)
	: Mount(parameters)
{
	if(! dynamic_cast<RobotParameters *>(parameters)) {
		std::cerr << "Error:" << __FILE__ << ":" << __LINE__ << std::endl;
		exit(3);
	}
}


P3D RobotMount::transformation(P3D const & x0, ParameterSet * parameters_in)
{
	RobotParameters * pars = static_cast<RobotParameters * >(parameters_in);

	if(! pars->is_synced_with_robot()) {
		std::cerr << "Error: " << __FILE__ << ":" << __LINE__ << std::endl; 
		exit(3);
	}

	return robotPart->getWorldCoordinates(x0);
}


void RobotMount::move(int parameter_idx, double dp)
{
	RobotParameters * pars = static_cast<RobotParameters * >(parameters);

	dVector par_vec;
	pars->pullVec(par_vec);
	par_vec[parameter_idx] += dp;
	pars->pushVec(par_vec);	
};


void RobotMount::dxDpar(P3D const & x0, ParameterSet * parameters_in, std::vector<V3D> & grad)
{
	RobotParameters * pars = static_cast<RobotParameters * >(parameters_in);
	if(! pars->is_synced_with_robot()) {
		std::cerr << "Error: " << __FILE__ << ":" << __LINE__ << std::endl; 
		exit(3);
	}


	MatrixNxM dpdq;
	pars->robotParameters->compute_dpdq(x0, robotPart, dpdq);

	int n = pars->getNPar();
	grad.resize(n);

	for(int i = 0; i < n; ++i) {
		for(int j = 0; j < 3; ++j) {
			grad[i][j] = dpdq(j, i+6);
		}
	}

}



RobotParameters::RobotParameters(GeneralizedCoordinatesRobotRepresentation * robotParameters)
	: robotParameters(robotParameters)
{
	offsetFullRevolutions.assign(getNPar(), 0.0);
	syncRobotStateWithParameters();
}


void RobotParameters::writeToList(dVector & par, int & cursor_idx_io)
{
	dVector par_local;
	robotParameters->getQ(par_local);

	int n = robotParameters->getDimensionCount();

	for(int i = 6; i < n; ++i) {
		double par_temp = par_local(i);
		par[cursor_idx_io++] = par_temp + 2.0*PI*offsetFullRevolutions[i-6];
	}
}



void RobotParameters::setFromList(dVector const & par, int & cursor_idx_io)
{
	robot_is_synced = false;
	
	int n = robotParameters->getDimensionCount();
	dVector par_local;
	robotParameters->getQ(par_local);

	for(int i = 6; i < n; ++i) {
		double par_temp = par[cursor_idx_io++];
		// limit angles to a range [-PI,PI] (the GeneralizedCoordinatesRobotRepresentation only operates with that range)
		// but: keep track of the full revolutions that were cut off
		
		double offsetRevs = std::round(par_temp / (2.0*PI));
		double par_bounded = par_temp - offsetRevs*2.0*PI;
		// prevent dangerous corner case: do not set a parameter in the GeneralizedCoordinatesRobotRepresentation
		// to precisely PI or -PI, as it is not obvious how accurately it handles that case
		// (it would be problematic if a parameter is set to -PI, but returned as +PI)
		if(IS_EQUAL(par_bounded, PI)) {
			par_bounded -= EPSILON;
			std::cout << "PI MAGIC!!!!!" << std::endl;
		}
		else if(IS_EQUAL(par_bounded, -PI)) {
			par_bounded += EPSILON;
			std::cout << "PI MAGIC!!!!!" << std::endl;
		}

		par_local(i) = par_bounded;
		offsetFullRevolutions[i-6] = offsetRevs;
	}

	robotParameters->setQ(par_local);

	syncRobotStateWithParameters();
}


std::pair<double, double> RobotParameters::getParameterLimitsByLocalIdx(int idx)
{
	// find the robot joint (hinge joint) the parameter corresponds to
	HingeJoint * joint = dynamic_cast<HingeJoint*>(robotParameters->getJointForQ(idx+6)); 

	if(!joint) {
		std::cerr << "Error: " << __FILE__ << ":" << __LINE__ << std::endl;
		exit(3);
	}

	// create pair of limits
	std::pair<double, double> limits(joint->minAngle, joint->maxAngle);

	return(limits);
}



void RobotParameters::syncRobotStateWithParameters() 
{
	robotParameters->syncRobotStateWithGeneralizedCoordinates();
	robot_is_synced = true;
}
