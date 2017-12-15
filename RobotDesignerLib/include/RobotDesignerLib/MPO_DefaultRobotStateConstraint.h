#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class MPO_DefaultRobotStateConstraint : public ObjectiveFunction {
public:
	MPO_DefaultRobotStateConstraint(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, int timeIndex, double weight);
	virtual ~MPO_DefaultRobotStateConstraint(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:

	template <class T>
	using VectorXT = Eigen::Matrix<T, -1, 1>;

	template <class T>
	class VXT : public VectorXT<T>
	{
	public:
		VXT<T>()
			: Eigen::Matrix<T, -1, 1>()
		{
		}

		template<class U>
		VXT<T>(const VectorXT<U> &other)
		{
			this->resize(other.size());
			for (int i = 0; i < other.size(); ++i) {
				this->data()[i] = other(i);
			}
		}
	};

	template<class T>
	T computeEnergy(const VectorXT<T> &q0, const VectorXT<T> &q) const {

		VectorXT<T> c = q - q0;
		return (T)0.5 * c.dot(c) * weight;
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
