#pragma once
#include <iostream>

#include "MathLib/P3D.h"
#include "Mount.h"

class RotationMount : public Mount {

public:
	// parameters are: [alpha, shift_1, shift_2]
public:
	RotationMount() {parametersStartIndex = 0; parameters.assign(3, 0);}


	RotationMount(double alpha, double shift_1, double shift_2, int parametersStartIndex) {
		this->parametersStartIndex = parametersStartIndex;
		parameters.assign({alpha, shift_1, shift_2});
	};

	virtual P3D transformation(P3D const & x0, std::vector<double> const & parameters);


	//virtual void getDxDpar(P3D const & x0, std::vector<V3D> & grad);

	// manipulation of mount
	void rotate(P3D const & origin, double alpha);
	void shift(V3D const & delta);


};


