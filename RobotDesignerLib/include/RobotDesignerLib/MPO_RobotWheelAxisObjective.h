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
	T computeEnergy(const LocomotionEngine_EndEffectorTrajectory &eeTraj, const VectorXT<T> &q,
					T yawAngle,
					T tiltAngle) const
	{
		// wheel axis from robot
		Vector3T<T> wheelAxisRobot = theMotionPlan->robotRepresentation->getWorldCoordinatesForVectorT(eeTraj.endEffectorRB->rbProperties.endEffectorPoints[eeTraj.CPIndex].localCoordsWheelAxis, eeTraj.endEffectorRB, q);
		// wheel axis from wheel angles
		Vector3T<T> wheelAxisWorld = LocomotionEngine_EndEffectorTrajectory::rotVecByYawTilt(eeTraj.wheelAxisLocal, eeTraj.wheelYawAxis, yawAngle, eeTraj.wheelTiltAxis, tiltAngle);
		Vector3T<T> err = wheelAxisWorld - wheelAxisRobot;

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
