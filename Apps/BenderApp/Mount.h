#pragma once

#include <vector>

#include "MathLib/P3D.h"

class Mount {

public:
	std::vector<double> parameters;

public:

	virtual P3D transformation(P3D const & x0, std::vector<double> const & parameters);

	virtual void dxDpar(P3D const & x0, std::vector<double> const & parameters, std::vector<V3D> & DxDpar);
    virtual void dxDparFD(P3D const & x0, std::vector<double> const & parameters, std::vector<V3D> & DxDpar);

	virtual P3D getTransformedX(P3D const & x0);
	virtual void getDxDpar(P3D const & x0, std::vector<V3D> & DxDpar);

	//void getDxDparFD(P3D const & x0, std::vector<V3D> & dxDpar);
};
