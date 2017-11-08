#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class MPO_FeetSlidingObjective : public ObjectiveFunction {

public:
	MPO_FeetSlidingObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_FeetSlidingObjective(void);

	virtual double computeValue(const dVector& p);

	virtual void addGradientTo(dVector& grad, const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);

private:
	template <class T>
	using Vector3T = Eigen::Matrix<T, 3, 1>;

	template<class T>
	static Vector3T<T> computeConstraint(const Vector3T<T> &eePosjp1, const Vector3T<T> &eePosj, double dt, const Vector3T<T> &omega, const Vector3T<T> &radius) {
		return (eePosjp1 - eePosj)/(T)dt + omega.cross(radius);
	}

	template<class T>
	static T computeEnergy(const Vector3T<T> &eePosjp1, const Vector3T<T> &eePosj, double dt, const Vector3T<T> &wheelRadiusV, const T &wheelRadius, const Vector3T<T> &wheelAxis, const T &speed, double c, double weight) {
		Vector3T<T> rr = wheelRadiusV*wheelRadius;
		Vector3T<T> omega = wheelAxis*speed;
		Vector3T<T> constraint = computeConstraint(eePosjp1, eePosj, dt, omega, rr);
		return (T)0.5 * constraint.squaredNorm() * c * weight;
	}

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	double wheelRadius = 1.0;
	Eigen::Vector3d wheelRadiusV = Eigen::Vector3d(0, -1, 0);
	Eigen::Vector3d wheelAxis = Eigen::Vector3d(1, 0, 0);
	double wheelAxisAlpha = M_PI*0.25; // rotation around y axis
};

