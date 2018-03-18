
#include <iostream>

#include "RBPointDistanceObjective.h"





RBPointDistanceObjective::RBPointDistanceObjective(RobotParameters * robotParameters,
												   RigidBody * rb1, RigidBody * rb2,
												   P3D pt1, P3D pt2,
												   double lowerBound, double upperBound,
												   double stiffness, double epsilon)
	: robotParameters(robotParameters), 
	  rb1(rb1), rb2(rb2), 
	  pt1(pt1), pt2(pt2),
	lowerConstraint(lowerBound, stiffness, epsilon), 
	upperConstraint(upperBound, stiffness, epsilon)
{

}




double RBPointDistanceObjective::computeValue(const dVector& p)
{

	// set (robot joint angle) parameters from global parameter list
	int i_io = robotParameters->parametersStartIndex;
	robotParameters->setFromList(p, i_io);
	// distance
	double d = distance(rb1, rb2, pt1, pt2);
	// energy
	double e = 0.0;
	e += lowerConstraint.computeValue(d);
	e += upperConstraint.computeValue(d);

	return(e);
}



void RBPointDistanceObjective::addGradientTo(dVector& grad, const dVector& p)	// gradient with respect to the global parameter set
{

	// set (robot joint angle) parameters from global parameter list
	int i_io = robotParameters->parametersStartIndex;
	robotParameters->setFromList(p, i_io);
	// distance
	double d = distance(rb1, rb2, pt1, pt2);

	dVector ddistancedrobotpar;
	dDistanceDpar(rb1, rb2, pt1, pt2, ddistancedrobotpar);

	double dEdd = lowerConstraint.computeDerivative(d) + upperConstraint.computeDerivative(d);

	for(int k = 0; k < ddistancedrobotpar.size(); ++k) {
		grad[robotParameters->parametersStartIndex + k] += ddistancedrobotpar[k] * dEdd;
	}

}

double RBPointDistanceObjective::distance(RigidBody * rb1, RigidBody * rb2, P3D const & pt1, P3D const & pt2)
{
	// points in global coordinates
	P3D c1 = robotParameters->robotParameters->getWorldCoordinatesFor(pt1, rb1);
	P3D c2 = robotParameters->robotParameters->getWorldCoordinatesFor(pt2, rb2);
	// distance
	double d = (c1 - c2).length();
	
	return(d);
}


void RBPointDistanceObjective::dDistanceDpar(RigidBody * rb1, RigidBody * rb2, P3D const & pt1, P3D const & pt2, dVector & grad) // gradient with respect to the (free) robot parameters
{
	// points in global coordinates
	P3D c1 = robotParameters->robotParameters->getWorldCoordinatesFor(pt1, rb1);
	P3D c2 = robotParameters->robotParameters->getWorldCoordinatesFor(pt2, rb2);

	// derivatives of points, with respect to ALL robot parameters
	MatrixNxM dc1dq;	// dq here refers to the parameter set of the GeneralizedCoordinatesRobotRepresentation
	MatrixNxM dc2dq;
	robotParameters->robotParameters->compute_dpdq(pt1, rb1, dc1dq);
	robotParameters->robotParameters->compute_dpdq(pt1, rb1, dc2dq);

	// distance and distance gradient
	V3D deltac = (c2 - c1);
	MatrixNxM ddeltacdq = dc2dq - dc1dq;
	int n = robotParameters->getNPar();

	// gradient with respect to the FREE robot parameters
	grad.resize(n);
	for(int i = 0; i < n; ++i) {
		grad[i] = deltac.unit().dot(static_cast<V3D>(ddeltacdq.col(i+6)));
	}

}





