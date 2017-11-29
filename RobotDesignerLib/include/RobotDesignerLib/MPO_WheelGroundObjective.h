#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class MPO_WheelGroundObjective : public ObjectiveFunction {

public:
	MPO_WheelGroundObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_WheelGroundObjective(void);

	virtual double computeValue(const dVector& p);

	virtual void addGradientTo(dVector& grad, const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);

private:
	template<class T>
	static T computeEnergy(const T &eePosY,
						   const T &wheelRadius,
						   const T &beta,
						   double c,
						   double weight) {
		// distance of wheel center to ground
//		T d = wheelRadius*cos(beta);

		T constraint = eePosY;
		return (T)0.5 * constraint*constraint * c * weight;
	}

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	template<class T>
	struct DOF {
		T* v;
		int i;
	};

	// DOFs: eePosY, beta
	static const int numDOFs = 2;
};

