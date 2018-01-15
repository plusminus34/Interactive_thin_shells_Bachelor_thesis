#pragma once

#include <vector>

#include "MathLib/P3D.h"
#include "MathLib/V3D.h"

#include "ParameterSet.h"


class Mount {

public:
	//std::vector<double> parameters;
	
	ParameterSet* parameters;
	bool parameterOptimization = true;	// whether or not this mount is part of the global parameter optimization
	bool active = true;

public:
	//virtual std::vector<double> get_parameters() = 0;
	//virtual void set_parameters()
	Mount(ParameterSet * parameters) : parameters(parameters) {};

	virtual P3D transformation(P3D const & x0, ParameterSet * parameters_in) = 0;
	virtual void dxDpar(P3D const & x0, ParameterSet * parameters_in, std::vector<V3D> & grad);
    
	virtual void dxDparFD(P3D const & x0, ParameterSet * parameters_in, std::vector<V3D> & grad);

	virtual P3D getTransformedX(P3D const & x0);
	virtual void getDxDpar(P3D const & x0, std::vector<V3D> & grad);


	//void getDxDparFD(P3D const & x0, std::vector<V3D> & dxDpar);

	void reset();
};
