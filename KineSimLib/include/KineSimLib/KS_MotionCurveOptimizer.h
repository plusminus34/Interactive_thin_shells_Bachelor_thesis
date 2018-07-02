#pragma once

#include "KS_ParameterizedMechanismOptimizerBase.h"
#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <MathLib/PolyLine3d.h>

/**
	This is an abstract implementation for objective functions that operates on parameterized mechanical assembly. These objective functions rely on the jacobian
	ds/dp, which is evaluated at a series of points (si, ti) (state and corresponding ticker value).
*/
class KS_MotionCurveOptimizer : public KS_ParameterizedMechanismOptimizerBase, public ObjectiveFunction{
public:
	KS_MotionCurveOptimizer(KS_ParameterizedMechanicalAssembly* mechanism, int numberOfEvaluationPoints, int targetComponentIndex, Point3d localCoordsOfTracerPoint);
	virtual ~KS_MotionCurveOptimizer();

	void setTargetMotionCurve(const PolyLine3d& target);

	//this should always return the current value of the objective function
	virtual double computeValue(const dVector& p);
	virtual dVector* getGradientAt(const dVector& p);
	//this method gets called whenever a new best solution to the objective function is found
	virtual void setCurrentBestSolution(const dVector& p);

	Point3d getTargetPointForFrame(int i);
	Point3d getTargetPointForFrame(int i, double offset);

protected:
	PolyLine3d motionTrailTarget;
	int targetComponentIndex;
	Point3d localCoordsOfTracerPoint;
	dVector objective;
	//we compute an offset that we use when comparing motions, since they are probably not going to be well-aligned by default
	double curveMatchOffset;

	double computeBestCurveMatchOffset(DynamicArray<dVector> *stateArray);
	double getDistanceToTargetCurve(DynamicArray<dVector> *stateArray, double offset);

	dVector									gradient;
	DynamicArray<dVector>					dsdpArray;

	DynamicArray<dVector>					tmpAssemblyStateArray;
	dVector									tmpTickerValues;
	dVector									bestParams;

	double regularizer;
	//regularizing solution
	dVector									p0;
};


