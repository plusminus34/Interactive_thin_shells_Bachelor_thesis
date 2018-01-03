#ifndef MPO_COM_ZERO_VELOCITY_CONSTRAINT_H
#define MPO_COM_ZERO_VELOCITY_CONSTRAINT_H

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

#include <memory>

class MPO_COMZeroVelocityConstraint : public ObjectiveFunction {
public:
	MPO_COMZeroVelocityConstraint(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, int timeIndex, double weight);
	virtual ~MPO_COMZeroVelocityConstraint(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	template<class T>
	T computeEnergy(const Vector3T<T> &comPosj, const Vector3T<T> &comPosjp1) const {
		Vector3T<T> constraint = comPosj-comPosjp1;
		return (T)0.5 * constraint.dot(constraint) * weight;
	}

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	template<class T>
	struct DOF {
		T* v;
		int i;
	};

	int timeIndex;
};

#endif // MPO_START_VELOCITY_CONSTRAINT_H
