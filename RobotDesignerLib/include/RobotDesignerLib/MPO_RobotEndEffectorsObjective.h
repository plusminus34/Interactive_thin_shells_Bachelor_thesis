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
	T computeEnergy(const Vector3T<T> &eePos, const Vector3T<T> &eePosLocal, const VectorXT<T> &q_t, const RigidBody *rb) const {
		Vector3T<T> robotEEPos = theMotionPlan->robotRepresentation->getWorldCoordinatesForT(eePosLocal, rb, q_t);

		Vector3T<T> err;
		err = robotEEPos - eePos;
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

