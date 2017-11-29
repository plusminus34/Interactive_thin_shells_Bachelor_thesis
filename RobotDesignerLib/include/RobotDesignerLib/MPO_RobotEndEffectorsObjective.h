#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class MPO_RobotEndEffectorsObjective : public ObjectiveFunction {

public:
	MPO_RobotEndEffectorsObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_RobotEndEffectorsObjective(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	template<class T>
	T computeEnergy(const Vector3T<T> &eePos, const Vector3T<T> &rho,
					const Vector3T<T> &yawAxis, T yawAngle,
					const Vector3T<T> &tiltAxis, T tiltAngle,
					const Vector3T<T> &eePosLocal, const VectorXT<T> &q_t, const RigidBody *rb) const {

		// Note/TODO: `eePosLocal` is the local position of the wheel center in wheel rigid body coordinates
		//            `eePos` is the world position of the wheel contact point
		//            This is to prevent a dependency of `robotEEPos` on the orientation of the wheel

		// end effector position according to robot pose (/robot state)
		Vector3T<T> robotEEPos = theMotionPlan->robotRepresentation->getWorldCoordinatesForT(eePosLocal, rb, q_t);

		// `eePos` is at the contact point, thus we need the vector connecting wheel center and contact point
		Vector3T<T> rhoRot = LocomotionEngine_EndEffectorTrajectory::rotateWheelAxisWith(rho, yawAxis, yawAngle, tiltAxis, tiltAngle);


		Vector3T<T> err;
		err = robotEEPos - eePos - rhoRot;
		T error = (T)0.5 * err.squaredNorm() * (T)weight;
		return error;
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

