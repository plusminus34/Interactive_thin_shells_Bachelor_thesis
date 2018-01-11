#pragma once

#include "MathLib/Ray.h"
#include "MathLib/Trajectory.h"


//class Trajectory3Dplus;

class Trajectory3Dplus : public GenericTrajectory<P3D> {

public:


public:
	//Trajectory3Dplus() {};
	//~Trajectory3Dplus();

	void createFromNodes(DynamicArray<Node *> const & nodes, dVector const & x);

	V3D evaluate_gradient_catmull_rom(double t, bool equalEndpointSlopes = true);

	void getMinDistanceLinear(P3D const & pt, double & d, int & i, bool & within_segment) const;

	void setTValueToLength();

	void createDiscreteSpline(int n, Trajectory3Dplus & spline);


	void addKnotInteractive(P3D const & pt);
	void addKnotBack(P3D const & pt);

	void removeKnotInteractive(int knotID);

	int getSelectedKnotID(Ray const & ray);
	double getDistanceToRay(Ray const & ray, P3D *closestPtOnRay = NULL);

	void draw(V3D lineColor, int lineWidth, V3D knotColor, double knotSize);


	static double distanceSquaredFromTo(Trajectory3Dplus const & traj1, Trajectory3Dplus const & traj2);

};