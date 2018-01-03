
#include "BenderApp.h"

#include "NodePositionObjectiveFunction.h"

double NodePositionObjectiveFunction::computeValue(const dVector& p)
{
	return(app->peekOofXi(p));
}



void NodePositionObjectiveFunction::addGradientTo(dVector& grad, const dVector& p)
{
	dVector dodxi;
	app->computeDoDxi(dodxi);

	grad += dodxi;
}


void NodePositionObjectiveFunction::setCurrentBestSolution(const dVector& p)
{
	app->xi = p;
std::cout << "xi = ";
for(int i = 0; i < app->xi.size(); ++i) {
	std::cout << app->xi(i) << " ";
}
std::cout << std::endl;

	app->pushXi();
	app->solveMesh();
}