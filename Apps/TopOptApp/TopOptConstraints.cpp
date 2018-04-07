#include "TopOptConstraints.h"

TopOptConstraints::TopOptConstraints(SimulationMesh* simMesh){
	this->simMesh = simMesh;
	resize(ineqConstraintVals, 1);
	resize(d, 1);
	resize(f, 1);

	d[0] = -10000;
	f[0] = 10000;
}

//and bound constraints for the parameters min <= p <= max
const dVector& TopOptConstraints::getBoundConstraintsMinValues() {
	l.resize(simMesh->elements.size());

	//not quite zero for the lower limit, but close enough
	l.setConstant(0.00001);

	return l;
}

const dVector& TopOptConstraints::getBoundConstraintsMaxValues() {
	u.resize(simMesh->elements.size());

	//1 as an upper limit
	u.setConstant(1.0);

	return u;
}

TopOptConstraints::~TopOptConstraints(void){
}

int TopOptConstraints::getEqualityConstraintCount() {
	return 0;
}

int TopOptConstraints::getInequalityConstraintCount() {
	return 1;
}

const dVector& TopOptConstraints::getInequalityConstraintValues(const dVector& p) {
	resize(ineqConstraintVals, 1);

	double currentMass = 0;
	//we want the mass (e.g. density params * per element mass) to be bounded...
	for (uint i = 0; i < simMesh->elements.size(); i++) {
		if (CSTElement2D* e = dynamic_cast<CSTElement2D*>(simMesh->elements[i]))
			currentMass += e->getMass() * p[i];
	}

	ineqConstraintVals[0] = currentMass;

	
	return ineqConstraintVals;
}


//get the actual value of the equality constraints...
const dVector& TopOptConstraints::getEqualityConstraintValues(const dVector& p){
	resize(eqConstraintVals, 0);
	resize(b, 0);

	return eqConstraintVals;
}

/*!
*  evaluates the constraint jacobian
*/
void TopOptConstraints::addEqualityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p) {

}


/*!
*  evaluates the constraint jacobian
*/
void TopOptConstraints::addInequalityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p) {
//	FunctionConstraints::addInequalityConstraintsJacobianEntriesTo(jacobianEntries, p);
//	return;

	for (uint i = 0; i < simMesh->elements.size(); i++) {
		if (CSTElement2D* e = dynamic_cast<CSTElement2D*>(simMesh->elements[i]))
			jacobianEntries.push_back(MTriplet(0, i, e->getMass()));
	}
}

