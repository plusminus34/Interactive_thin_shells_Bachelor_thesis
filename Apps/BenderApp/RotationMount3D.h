#pragma once
#include <iostream>

#include "MathLib/P3D.h"
#include "MathLib/Quaternion.h"
#include "Mount.h"

#include "ParameterSet.h"


class RotationMount3D : public Mount {

public:
	// parameters are: [alpha, beta, gamma, shift_1, shift_2, shift_3] where qi are the components of a quaternion

public:
	//RotationMount3D() {parametersStartIndex = 0; parameters.assign(6, 0.0);}

//	RotationMount3D(double alpha, double shift_1, double shift_2, int parametersStartIndex) {
//		this->parametersStartIndex = parametersStartIndex;
//		parameters.assign({alpha, shift_1, shift_2});
//	};
	RotationMount3D(ParameterSet * parameters);


	virtual P3D transformation(P3D const & x0, ParameterSet * parameters_in);
	virtual void dxDpar(P3D const & x0, ParameterSet * parameters_in, std::vector<V3D> & grad);

	//virtual void getDxDpar(P3D const & x0, std::vector<V3D> & grad);

	// manipulation of mount
	void rotate(P3D const & origin, double alpha, double beta, double gamma);
	//void rotate(P3D const & origin, V3D const & axis, double phi);
	void shift(V3D const & delta);


};



class EulerRotationParameters : public ParameterSet {

public:
	double alpha, beta, gamma;
	V3D shift;

public:

	virtual void writeToList(dVector & par, int & cursor_idx_io);
	virtual void setFromList(dVector & par, int & cursor_idx_io);
	virtual int getNPar() const {return(6);}

};