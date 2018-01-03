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
	template<class T>
	static Vector3T<T> computeConstraintWheel(const Vector3T<T> &eePosjp1, const Vector3T<T> &eePosj, double dt, const Vector3T<T> &omega, const Vector3T<T> &radius) {
		return (eePosjp1 - eePosj)/(T)dt + omega.cross(radius);
	}

	template<class T>
	static T computeEnergyWheel(const Vector3T<T> &eePosjp1, const Vector3T<T> &eePosj, double dt,
						   const Vector3T<T> &rhoLocal, const Vector3T<T> &wheelAxisLocal,
						   const Vector3T<T> &axisYaw, const T &alphaj, const T &alphajp1,
						   const Vector3T<T> &axisTilt, const T &betaj, const T &betajp1,
						   const T &speedj, const T &speedjp1, double c,
						   double weight) {

		Vector3T<T> wheelAxisWorld = LocomotionEngine_EndEffectorTrajectory::rotVecByYawTilt(wheelAxisLocal, axisYaw, (T)0.5*(alphaj+alphajp1), axisTilt, (T)0.5*(betaj+betajp1));

		Vector3T<T> rhoWorld = LocomotionEngine_EndEffectorTrajectory::rotVecByYawTilt(rhoLocal, axisYaw, (T)0.5*(alphaj+alphajp1), axisTilt, (T)0.5*(betaj+betajp1));
		// interpolate (average) wheel speed
		Vector3T<T> omega = wheelAxisWorld*(speedj+speedjp1)*0.5;
		Vector3T<T> constraint = computeConstraintWheel(eePosjp1, eePosj, dt, omega, rhoWorld);
		return (T)0.5 * constraint.squaredNorm() * c * weight;
	}

	template<class T>
	static T computeEnergyFoot(const Vector3T<T> &eePosjp1, const Vector3T<T> &eePosj, double c, double weight) {
		Vector3T<T> constraint = (eePosjp1-eePosj);
		return (T)0.5 * constraint.squaredNorm() * c * weight;
	}

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	template<class T>
	struct DOF {
		T* v;
		int i;
	};

	// eePosj (3), eePosjm1 (3), speedj, speedjm1, alphaj, alphajm1, betaj, betajm1
	static const int numDOFsWheel = 12;
	// eePosj (3), eePosjm1 (3)
	static const int numDOFsFoot = 6;
};

