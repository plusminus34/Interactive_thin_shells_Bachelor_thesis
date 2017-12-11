#pragma once

#include <vector>

#include "MathLib/P3D.h"
#include "MathLib/V3D.h"

class Mount {

public:
	std::vector<double> parameters;
	int parametersStartIndex;	// position of parameter set in context of the global parameter set

public:
	//virtual std::vector<double> get_parameters() = 0;
	//virtual void set_parameters()

	virtual P3D transformation(P3D const & x0, std::vector<double> const & parameters) = 0;

	virtual void dxDpar(P3D const & x0, std::vector<double> const & parameters, std::vector<V3D> & grad);
    virtual void dxDparFD(P3D const & x0, std::vector<double> const & parameters, std::vector<V3D> & grad);

	virtual P3D getTransformedX(P3D const & x0);
	virtual void getDxDpar(P3D const & x0, std::vector<V3D> & grad);


	//void getDxDparFD(P3D const & x0, std::vector<V3D> & dxDpar);

	void reset();
};
