
#include <vector>
#include <cmath>
#include <iostream>

#include "MathLib/P3D.h"

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

/*
void RobotMount::dxDpar(P3D const & x0, ParameterSet * parameters_in, std::vector<V3D> & grad)
{
	RobotParameters * pars = static_cast<RobotParameters * >(parameters_in);
std::cout << "dxDpar... ";
	MatrixNxM dpdq;
	pars->robotParameters->compute_dpdq(x0, robotPart, dpdq);

	int n = pars->getNPar();
	grad.resize(n);

	for(int i = 0; i < n; ++n) {
		for(int j = 0; j < 3; ++j) {
			grad[i][j] = dpdq(j, i);
		}
	}
std::cout << "done" << std::endl;
}
*/


RobotParameters::RobotParameters(GeneralizedCoordinatesRobotRepresentation * robotParameters)
	: robotParameters(robotParameters)
{
	syncRobotStateWithParameters();
}


void RobotParameters::writeToList(dVector & par, int & cursor_idx_io)
{
	dVector par_local;
	robotParameters->getQ(par_local);

	int n = robotParameters->getDimensionCount();

	for(int i = 6; i < n; ++i) {
		par[cursor_idx_io++] = par_local(i);
	}
}



void RobotParameters::setFromList(dVector & par, int & cursor_idx_io)
{
	robot_is_synced = false;
	
	int n = robotParameters->getDimensionCount();
	dVector par_local;
	robotParameters->getQ(par_local);

	for(int i = 6; i < n; ++i) {
		par_local(i) = par[cursor_idx_io++];
	}

	robotParameters->setQ(par_local);

	syncRobotStateWithParameters();
}



void RobotParameters::syncRobotStateWithParameters() 
{
	robotParameters->syncRobotStateWithGeneralizedCoordinates();
	robot_is_synced = true;
}
