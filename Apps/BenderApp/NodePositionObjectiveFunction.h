#pragma once

#include <OptimizationLib/ObjectiveFunction.h>

class BenderApp2D;

class NodePositionObjectiveFunction : public ObjectiveFunction {

public:

	BenderApp2D * app;


	NodePositionObjectiveFunction() {};
	NodePositionObjectiveFunction(BenderApp2D * app) : app(app) {};

	virtual double computeValue(const dVector& p);

	virtual void addGradientTo(dVector& grad, const dVector& s);

	virtual void setCurrentBestSolution(const dVector& s);
	
};