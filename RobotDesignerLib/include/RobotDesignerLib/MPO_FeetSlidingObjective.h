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
#if 0
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
#endif

private:
	template <class T>
	using Vector3T = Eigen::Matrix<T, 3, 1>;

	template<class T>
	static Vector3T<T> computeConstraint(const Vector3T<T> &eePosjp1, const Vector3T<T> &eePosj, double h, const Vector3T<T> &omega, const Vector3T<T> &radius)
	{
		return (eePosjp1 - eePosj)/(T)h + omega.cross(radius);
	}

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	double wheelRadius = 0.0;
};

