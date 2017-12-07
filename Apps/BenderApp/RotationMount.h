#pragma once

#include "MathLib/P3D.h"
#include "Mount.h"

class RotationMount : public Mount {

public:
	// parameters are: [alpha, shift_1, shift_2]
public:
	RotationMount() : parameters(3, 0) {};
	RotationMount(double alpha, double shift_1, double shift_2) : parameters({alpha, shift_1, shift_2}) {};

	virtual P3D transformation(P3D const & x0, std::vector<double> const & parameters);


	virtual void getDxDpar(P3D const & x0, std::vector<V3D> & DxDpar);
};


