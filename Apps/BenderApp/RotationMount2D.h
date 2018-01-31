#pragma once
#include <iostream>

#include "MathLib/P3D.h"
#include "Mount.h"

class RotationMount2D : public Mount {

public:
	// parameters are: [alpha, shift_1, shift_2]
public:
	RotationMount2D(ParameterSet * parameters);

	/*
	RotationMount2D(double alpha, double shift_1, double shift_2, int parametersStartIndex) {
		this->parametersStartIndex = parametersStartIndex;
		parameters.assign({alpha, shift_1, shift_2});
	};
	*/

	virtual P3D transformation(P3D const & x0, ParameterSet * parameters_in);
	virtual void dxDpar(P3D const & x0, ParameterSet * parameters_in, std::vector<V3D> & grad);

	//virtual void getDxDpar(P3D const & x0, std::vector<V3D> & grad);

	// manipulation of mount
	void rotate(P3D const & origin, double alpha);
	void shift(V3D const & delta);


};


class Rotation2DParameters : public ParameterSet {

public:
	double alpha;
	V3D shift;

public:

	virtual void writeToList(dVector & par, int & cursor_idx_io);
	virtual void setFromList(dVector const & par, int & cursor_idx_io);
	virtual int getNPar() const {return(4);}

};