#pragma once

//#include "RBSimLib/AbstractRBEngine.h"
#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/SoftUnilateralConstraint.h>

#include "RobotMount.h"	// defines RobotParameters



class RBPointDistanceObjective : public ObjectiveFunction {


public:
	RobotParameters * robotParameters;
	RigidBody * rb1, * rb2;
	P3D pt1, pt2;	// local coordinates of rigid body

	SoftUnilateralConstraint lowerConstraint;
	SoftUnilateralUpperConstraint upperConstraint;

public:
	RBPointDistanceObjective(RobotParameters * robotParameters,
							 RigidBody * rb1, RigidBody * rb2,
							 P3D pt1, P3D pt2,
							 double lowerBound, double upperBound,
							 double stiffness, double epsilon);


	virtual double computeValue(const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

	double distance(RigidBody * rb1, RigidBody * rb2, P3D const & pt1, P3D const & pt2);
	void dDistanceDpar(RigidBody * rb1, RigidBody * rb2, P3D const & pt1, P3D const & pt2, dVector & grad);
};