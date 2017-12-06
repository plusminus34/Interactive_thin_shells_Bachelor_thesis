#include <OptimizationLib/ObjectiveFunctionAD.h>
#include <Utils/Logger.h>

ObjectiveFunctionAD::ObjectiveFunctionAD(){
}

ObjectiveFunctionAD::~ObjectiveFunctionAD(){
}

double ObjectiveFunctionAD::computeValue(const dVector &p)
{
	DOFColl<double> dofColl = collectDOFs(p);
	return computeEnergy(dofColl) * weight;
}

void ObjectiveFunctionAD::addGradientTo(dVector &grad, const dVector &p)
{
	DOFColl<ScalarDiff> dofColl = collectDOFsD(p);

	for (auto &dof : dofColl.dofs) {
		dof.v->deriv() = 1.0;
		ScalarDiff energy = computeEnergy(dofColl);
		grad[dof.i] = energy.deriv() * weight;
		dof.v->deriv() = 0.0;
	}
}

void ObjectiveFunctionAD::addHessianEntriesTo(std::vector<MTriplet> &hessianEntries, const dVector &p)
{
	DOFColl<ScalarDiffDiff> dofColl = collectDOFsDD(p);

	for (auto &dofi : dofColl.dofs) {
		dofi.v->deriv().value() = 1.0;
		for (auto &dofj : dofColl.dofs) {
			dofj.v->value().deriv() = 1.0;
			ScalarDiffDiff energy = computeEnergy(dofColl);
			ADD_HES_ELEMENT(hessianEntries, dofi.i, dofj.i, energy.deriv().deriv(), weight);
			dofj.v->value().deriv() = 0.0;
		}
		dofi.v->deriv().value() = 0.0;
	}
}
