#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

/*
 * Ground reaction forces (GRFs) at passive wheels are zero in the
 * direction of the wheel. These constraints ensure that this component
 * of the GRF is indeed zero.
 */
class MPO_PassiveWheelsGRFConstraints : public ObjectiveFunction {

public:
	MPO_PassiveWheelsGRFConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_PassiveWheelsGRFConstraints(void);

	virtual double computeValue(const dVector& p);

	virtual void addGradientTo(dVector& grad, const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);

private:

	template<class T>
	T computeForceForward(const Vector3T<T> &wheelAxisLocal, const Vector3T<T> &yawAxis, T yawAngle, const Vector3T<T> &tiltAxis, T tiltAngle,
					const Vector3T<T> &contactForce) const
	{
		Vector3T<T> forward = yawAxis.cross(wheelAxisLocal);
		Vector3T<T> forwardWorld = LocomotionEngine_EndEffectorTrajectory::rotateWheelAxisWith(forward, yawAxis, yawAngle, tiltAxis, tiltAngle);

		T forceForward = forwardWorld.dot(contactForce);
		return forceForward;
	}

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	template<class T>
	struct DOF {
		T* v;
		int i;
	};
};
