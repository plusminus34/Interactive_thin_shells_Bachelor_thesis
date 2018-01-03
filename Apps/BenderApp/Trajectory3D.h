#pragma once


#include "MathLib/Trajectory.h"


//class Trajectory3Dplus;

class Trajectory3Dplus : public GenericTrajectory<P3D> {

public:
	//Trajectory3Dplus *discreteSpline = NULL;


public:
	//Trajectory3Dplus() {};
	//~Trajectory3Dplus();

	void createFromNodes(DynamicArray<Node *> const & nodes, dVector const & x);

	V3D evaluate_gradient_catmull_rom(double t, bool equalEndpointSlopes = true);

	void getMinDistanceLinear(P3D const & pt, double & d, int & i, bool & within_segment) const;

	void setTValueToLength();
	//void updateDiscreteSpline();

	void createDiscreteSpline(int n, Trajectory3Dplus & spline);


	void addKnotInteractive(P3D const & pt);
	void addKnotBack(P3D const & pt);

	void draw(V3D lineColor, int lineWidth, V3D knotColor, double knotSize);


	static double distanceSquaredFromTo(Trajectory3Dplus const & traj1, Trajectory3Dplus const & traj2);

};