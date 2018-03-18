#pragma once
#include <iostream>

#include "MathLib/P3D.h"
#include "MathLib/Quaternion.h"
#include "Mount.h"

#include "ParameterSet.h"


class RotationMount3D : public Mount {

private:


public:

	RotationMount3D(ParameterSet * parameters);


	virtual P3D transformation(P3D const & x0, ParameterSet * parameters_in);
	virtual void dxDpar(P3D const & x0, ParameterSet * parameters_in, std::vector<V3D> & grad);


	// manipulation of mount
	void rotate(P3D const & origin, double alpha, double beta, double gamma);
	//void rotate(P3D const & origin, V3D const & axis, double phi);
	void shift(V3D const & delta);


};



class EulerRotationParameters : public ParameterSet {

public:
	double alpha = 0.0, beta = 0.0, gamma = 0.0;
	V3D shift = V3D(0.0);

public:

	virtual void writeToList(dVector & par, int & cursor_idx_io);
	virtual void setFromList(dVector const & par, int & cursor_idx_io);
	virtual int getNPar() const {return(6);}

};