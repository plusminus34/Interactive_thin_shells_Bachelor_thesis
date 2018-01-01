#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class MPO_FixedWheelObjective : public ObjectiveFunction {

public:
	MPO_FixedWheelObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_FixedWheelObjective(void);

	virtual double computeValue(const dVector& p);

	virtual void addGradientTo(dVector& grad, const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);

private:

	// TODO: do we need a local wheel axis and a global??
	template<class T>
	T computeEnergy(const Vector3T<T> &wheelAxisLocal, const Vector3T<T> &rhoLocal,
					const RigidBody *rb, const VectorXT<T> &qi,const VectorXT<T> &qip,
					const Vector3T<T> &yawAxis, T yawAnglej, T yawAnglejp,
					const Vector3T<T> &tiltAxis, T tiltAnglej, T tiltAnglejp,
					T wheelSpeedj, T wheelSpeedjp, T dt) const
	{
		// wheel axis from robot
		Vector3T<T> rhoRoboti = theMotionPlan->robotRepresentation->getWorldCoordinatesForVectorT(rhoLocal, rb, qi);
		Vector3T<T> rhoRobotip = theMotionPlan->robotRepresentation->getWorldCoordinatesForVectorT(rhoLocal, rb, qip);
		// wheel axis from wheel angles
		Vector3T<T> rhoWorldi = LocomotionEngine_EndEffectorTrajectory::rotateVectorUsingWheelAngles(rhoLocal, yawAxis, yawAnglej, tiltAxis, tiltAnglej);
		Vector3T<T> rhoWorldip = LocomotionEngine_EndEffectorTrajectory::rotateVectorUsingWheelAngles(rhoLocal, yawAxis, yawAnglejp, tiltAxis, tiltAnglejp);
		Vector3T<T> rhoWorldipRot = rotateVec(rhoWorldip, (wheelSpeedj+wheelSpeedjp)*(T)0.5*dt, wheelAxisLocal);
		Vector3T<T> err = rhoRoboti.cross(rhoRobotip) - rhoWorldi.cross(rhoWorldipRot);

		return (T)0.5 * err.squaredNorm() * (T)weight;
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
