#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class MPO_RobotWheelAxisObjective : public ObjectiveFunction {

public:
	MPO_RobotWheelAxisObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_RobotWheelAxisObjective(void);

	virtual double computeValue(const dVector& p);

	virtual void addGradientTo(dVector& grad, const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);

private:

	// TODO: do we need a local wheel axis and a global??
	template<class T>
	T computeEnergy(const Vector3T<T> &wheelAxisLocal, const Vector3T<T> &eePosLocal,
					const RigidBody *rb, const VectorXT<T> &q,
					const Vector3T<T> &yawAxis, T yawAngle,
					const Vector3T<T> &tiltAxis, T tiltAngle) const
	{
		// point at center of wheel in world coordinates
		Vector3T<T> pO = theMotionPlan->robotRepresentation->getWorldCoordinatesForT(eePosLocal, rb, q);
		// point at 'end' of wheel axis in world coordinates
		Vector3T<T> tmp = eePosLocal+wheelAxisLocal;
		Vector3T<T> pW = theMotionPlan->robotRepresentation->getWorldCoordinatesForT(tmp, rb, q);
		// wheel axis in world coordinates, according to robot pose
		Vector3T<T> currentAxis = pW-pO;

		// wheel axis from wheel angles
		Vector3T<T> wheelAxisWorld = LocomotionEngine_EndEffectorTrajectory::rotateWheelAxisWith(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle);
		Vector3T<T> err = wheelAxisWorld - currentAxis;

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
