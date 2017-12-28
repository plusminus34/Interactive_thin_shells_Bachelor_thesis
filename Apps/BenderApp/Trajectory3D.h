#pragma once


#include "MathLib/Trajectory.h"


class Trajectory3Dplus;

class Trajectory3Dplus : public GenericTrajectory<P3D> {

public:
	Trajectory3Dplus *discreteSpline = NULL;


public:
	Trajectory3Dplus() {};
	~Trajectory3Dplus();

	void getMinDistanceLinear(P3D const & pt, double & d, int & i, bool & within_segment) const;

	void updateTvalues();
	void updateDiscreteSpline();

	void createDiscreteSpline(int n, Trajectory3Dplus & spline);


	void addKnotInteractive(P3D const & pt);


	void draw();

};