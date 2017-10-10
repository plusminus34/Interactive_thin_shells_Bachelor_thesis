#include <OptimizationLib/ConstrainedObjectiveFunction.h>

ConstrainedObjectiveFunction::ConstrainedObjectiveFunction(ObjectiveFunction* obj, FunctionConstraints* C){
	this->objective_ = obj;
	this->constraints_ = C;
}

ConstrainedObjectiveFunction::~ConstrainedObjectiveFunction(){
}

void ConstrainedObjectiveFunction::setCurrentBestSolution(const dVector& p){
	this->objective_->setCurrentBestSolution(p);
}
