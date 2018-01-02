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

	template<class T>
	T computeEnergy(const Vector3T<T> &wheelAxisLocal, const Vector3T<T> &rhoLocal,
					const RigidBody *rb, const VectorXT<T> &qj,const VectorXT<T> &qjp,
					const Vector3T<T> &yawAxis, T yawAnglej, T yawAnglejp,
					const Vector3T<T> &tiltAxis, T tiltAnglej, T tiltAnglejp,
					T wheelSpeedj, T wheelSpeedjp, T dt) const
	{
		/*
		 * rotation angle of wheel around wheel axis:
		 *
		 * from robot point of view:
		 * sin(alpha) = `rho_robot_j` x `rho_robot_rot_jp`
		 *		where:	`rho_robot_j`		: `rho_local` rotated by attached RB at time `j`
		 *				`rho_robot_rot_j`	: `rho_local` rotated by attached RB at time `jp`
		 *
		 * from wheel point of view:
		 * sin(beta) = `rho_j` x `rho_rot_jp`
		 * 		where:	`rho_j`		 : `rho_local` rotated by tilt and yaw at time `j` (so always pointing down)
		 *				`rho_rot_jp` : `rho_local` rotated by wheel speed
		 *								  and then rotated by tilt and yaw at time `jp`
		 *
		 * We want sin(alpha) = sin(beta)
		 * => `rho_robot_j` x `rho_robot_rot_jp` - `rho_j` x `rho_rot_jp` = 0
		 */

		// wheel axis from robot
		Vector3T<T> rhoRobotj = theMotionPlan->robotRepresentation->getWorldCoordinatesForVectorT(rhoLocal, rb, qj);
		Vector3T<T> rhoRobotjp = theMotionPlan->robotRepresentation->getWorldCoordinatesForVectorT(rhoLocal, rb, qjp);
		// wheel axis from wheel angles
		Vector3T<T> rhoWheelj = LocomotionEngine_EndEffectorTrajectory::rotateVectorUsingWheelAngles(rhoLocal, yawAxis, yawAnglej, tiltAxis, tiltAnglej);
		Vector3T<T> rhoWheelLocalRotjp = rotateVec(rhoLocal, (wheelSpeedj+wheelSpeedjp)*(T)0.5*dt, wheelAxisLocal);
		Vector3T<T> rhoWheelRotjp = LocomotionEngine_EndEffectorTrajectory::rotateVectorUsingWheelAngles(rhoWheelLocalRotjp, yawAxis, yawAnglejp, tiltAxis, tiltAnglejp);
		Vector3T<T> err = rhoRobotj.cross(rhoRobotjp) - rhoWheelj.cross(rhoWheelRotjp);

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
