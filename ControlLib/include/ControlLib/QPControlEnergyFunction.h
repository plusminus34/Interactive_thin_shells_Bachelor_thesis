#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <ControlLib/QPControlPlan.h>

class QPC_EndEffectorAccelerationObjective {
public:
	QPC_EndEffectorAccelerationObjective(QPControlPlan *qpControlPlan, QPControl_EndEffector* endEffector) {
		this->endEffector = endEffector;
		this->theQPCPlan = qpControlPlan;
	}

	virtual ~QPC_EndEffectorAccelerationObjective() {};

	// Foot position and widget position 1/2 * || (J * qdotdot + Jdot * qdot)=a(q) - d_a || ^ 2
	virtual double computeValue(double weight) {
		// Get qdot and qdotdot
		dVector qDot;
		theQPCPlan->robotRepresentation->getQDot(qDot);

		// NOTE: Now is calculating 0.5 * || J * qDotDot + JDot * qDot + J * qDot(current velocity) / dt || ^ 2
        V3D err = (V3D)(endEffector->J * theQPCPlan->a + endEffector->Jdot * qDot) - endEffector->targetEEAcceleration;

		return 0.5 * err.length2() * weight;
	}

	// Gradient || (J * qdotdot + Jdot * qdot)=a(q) - d_a || * dp/dq(Jacobian)
	virtual void addGradientTo(dVector& grad, double weight) {
		// Get qdot & qdotdot
		dVector qDot;
		theQPCPlan->robotRepresentation->getQDot(qDot);

		dVector gradient;
		gradient.resize(theQPCPlan->robotRepresentation->getDimensionCount());
		gradient.setZero();
		// Compute current acceleration
        V3D err = (V3D)(endEffector->J * theQPCPlan->a + endEffector->Jdot * qDot) - endEffector->targetEEAcceleration;
		// J transpose
		gradient += endEffector->J.transpose() * err;

		// Apply gradient
		for (int i = 0; i < theQPCPlan->robotRepresentation->getDimensionCount(); ++i)
			grad[theQPCPlan->generalizedAccelerationsParamsStartIndex + i] += gradient[i] * weight;
	}

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p, double weight) {
		MatrixNxM hessian = endEffector->J.transpose() * endEffector->J * weight;
		for (int i = 0; i < hessian.innerSize(); ++i)
			for (int j = i; j < hessian.innerSize(); ++j) {
				if (!IS_ZERO(hessian(j, i))) {
					MTriplet mt(
						theQPCPlan->generalizedAccelerationsParamsStartIndex + j,
						theQPCPlan->generalizedAccelerationsParamsStartIndex + i,
						hessian(j, i)
						);
					hessianEntries.push_back(mt);
				}
			}
	}

public:
	QPControlPlan *theQPCPlan;
	QPControl_EndEffector* endEffector;
};

template <typename T> class QPC_EndEffectorListAccelerationObjective : public ObjectiveFunction {
public:
	QPC_EndEffectorListAccelerationObjective(QPControlPlan *qpControlPlan,
		std::string objectiveDescription, DynamicArray<T>* endEffectorList,
		double weight) : theQPCPlan(qpControlPlan), eeAccObj(qpControlPlan, NULL){
		this->description = objectiveDescription;
		this->weight = weight;
		this->endEffectorList = endEffectorList;
	}

	virtual ~QPC_EndEffectorListAccelerationObjective() {};

	virtual double computeValue(const dVector& p) {
		// Get qdot and qdotdot
		dVector qDot;
		theQPCPlan->robotRepresentation->getQDot(qDot);

		double error = 0;
		for (uint i = 0; i < endEffectorList->size(); i++) {
			eeAccObj.endEffector = &endEffectorList->at(i);
			error += eeAccObj.computeValue(weight);
		}

		return error;
	}

	// Gradient || (J * qdotdot + Jdot * qdot)=a(q) - d_a || * dp/dq(Jacobian)
	virtual void addGradientTo(dVector& grad, const dVector& p) {
		for (uint i = 0; i < endEffectorList->size(); i++) {
			eeAccObj.endEffector = &endEffectorList->at(i);
			eeAccObj.addGradientTo(grad, weight);
		}
	}

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
		for (uint i = 0; i < endEffectorList->size(); ++i) {
			eeAccObj.endEffector = &endEffectorList->at(i);
			eeAccObj.addHessianEntriesTo(hessianEntries, p, weight);
		}
	}

private:
	QPControlPlan *theQPCPlan;
	DynamicArray<T> *endEffectorList;

	QPC_EndEffectorAccelerationObjective eeAccObj;
};

class QPC_GeneralizedAccelerationsObjective : public ObjectiveFunction {
public:
	QPC_GeneralizedAccelerationsObjective(QPControlPlan *qpControlPlan,
		std::string objectiveDescription,
		double weight,
		int targetAccelerationStartIndex,
		int targetAccelerationLength) :
		theQPCPlan(qpControlPlan),
		targetAccelerationStartIndex(targetAccelerationStartIndex),
		targetAccelerationLength(targetAccelerationLength) {

		this->description = objectiveDescription;
		this->weight = weight;
	}

	virtual ~QPC_GeneralizedAccelerationsObjective() {}

	virtual void printObjectiveDetails(const dVector& p) {
		for (int i = 0; i < targetAccelerationLength; i++) {
			double tmp = theQPCPlan->targetGeneralizedAccelerations[targetAccelerationStartIndex + i] - theQPCPlan->a[targetAccelerationStartIndex + i];
			Logger::logPrint("%d: %lf %lf --> %lf\n", targetAccelerationStartIndex + i, theQPCPlan->targetGeneralizedAccelerations[targetAccelerationStartIndex + i], theQPCPlan->a[targetAccelerationStartIndex + i], tmp*tmp);
		}
	}

	virtual double computeValue(const dVector& p) {
		double val = 0;
		for (int i = 0; i < targetAccelerationLength; i++) {
			double tmp = theQPCPlan->targetGeneralizedAccelerations[targetAccelerationStartIndex + i] - theQPCPlan->a[targetAccelerationStartIndex + i];
			val += tmp * tmp;
		}

		return 0.5 * val * weight;
	}

	virtual void addGradientTo(dVector& grad, const dVector& p) {
		for (int i = 0; i < targetAccelerationLength; i++) {
			double tmp = theQPCPlan->targetGeneralizedAccelerations[targetAccelerationStartIndex + i] - theQPCPlan->a[targetAccelerationStartIndex + i];
			grad[theQPCPlan->generalizedAccelerationsParamsStartIndex + targetAccelerationStartIndex + i] += -tmp * weight;
		}
	}

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
		for (int i = 0; i < targetAccelerationLength; ++i) {
			int index = theQPCPlan->generalizedAccelerationsParamsStartIndex + targetAccelerationStartIndex + i;
			MTriplet tmp(index, index, weight);
			hessianEntries.push_back(tmp);
		}
	}

private:
	QPControlPlan *theQPCPlan;
	int targetAccelerationStartIndex, targetAccelerationLength;
};


class QPC_SwingLimbJointAccelerationsObjective : public ObjectiveFunction {
public:
	QPC_SwingLimbJointAccelerationsObjective(QPControlPlan *qpControlPlan, std::string objectiveDescription, double weight) {
		theQPCPlan = qpControlPlan;
		this->description = objectiveDescription;
		this->weight = weight;
	}

	virtual ~QPC_SwingLimbJointAccelerationsObjective() {}

	virtual void printObjectiveDetails(const dVector& p) {
		for (uint i = 0; i < theQPCPlan->dofUsedInSwingLimbs.size(); i++) {
			if (theQPCPlan->dofUsedInSwingLimbs[i] == false)
				continue;
			double tmp = theQPCPlan->targetGeneralizedAccelerations[i] - theQPCPlan->a[i];
			Logger::logPrint("%d: %lf %lf --> %lf\n", i, theQPCPlan->targetGeneralizedAccelerations[i], theQPCPlan->a[i], tmp*tmp);
		}
	}

	virtual double computeValue(const dVector& p) {
		double val = 0;
		for (uint i = 0; i < theQPCPlan->dofUsedInSwingLimbs.size(); i++) {
			if (theQPCPlan->dofUsedInSwingLimbs[i] == false)
				continue;
			double tmp = theQPCPlan->targetGeneralizedAccelerations[i] - theQPCPlan->a[i];
			val += tmp * tmp;
		}

		return 0.5 * val * weight;
	}

	virtual void addGradientTo(dVector& grad, const dVector& p) {
		for (uint i = 0; i < theQPCPlan->dofUsedInSwingLimbs.size(); i++) {
			if (theQPCPlan->dofUsedInSwingLimbs[i] == false)
				continue;
			double tmp = theQPCPlan->targetGeneralizedAccelerations[i] - theQPCPlan->a[i];
			grad[theQPCPlan->generalizedAccelerationsParamsStartIndex + i] += -tmp * weight;
		}
	}

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
		for (uint i = 0; i < theQPCPlan->dofUsedInSwingLimbs.size(); i++) {
			if (theQPCPlan->dofUsedInSwingLimbs[i] == false)
				continue;
			int index = theQPCPlan->generalizedAccelerationsParamsStartIndex + i;
			MTriplet tmp(index, index, weight);
			hessianEntries.push_back(tmp);
		}
	}

private:
	QPControlPlan *theQPCPlan;
	dVector qDotDotTargets;
};

class QPControlEnergyFunction : public ObjectiveFunction {
public:
	DynamicArray<ObjectiveFunction*> objectives;

	QPControlEnergyFunction(QPControlPlan* mp);
	virtual ~QPControlEnergyFunction(void);

	//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
	void updateRegularizingSolutionTo(const dVector &currentP);
	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

	//this method gets called whenever a new best solution to the objective function is found
	virtual void setCurrentBestSolution(const dVector& p);

	void setupSubObjectives();

	bool printDebugInfo;

public:
	double regularizer;

private:

	//this is the configuration of the sim mesh that is used as a regularizing solution...
	dVector m_p0;
	dVector tmpVec;

	//the energy function operates on a motion plan...
	QPControlPlan* theQPCPlan;
};

