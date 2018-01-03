#pragma once

#include <OptimizationLib/ObjectiveFunction.h>

class BenderApp;

class NodePositionObjectiveFunction : public ObjectiveFunction {

public:

	BenderApp * app;


	NodePositionObjectiveFunction() {};
	NodePositionObjectiveFunction(BenderApp * app) : app(app) {};

	virtual double computeValue(const dVector& p);

	virtual void addGradientTo(dVector& grad, const dVector& s);

	virtual void setCurrentBestSolution(const dVector& s);
	
};