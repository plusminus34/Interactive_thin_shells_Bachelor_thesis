#include <ControlLib/QPControlConstraints.h>



QPControlConstraints::QPControlConstraints(QPControlPlan* mp){
	qpPlan = mp;
	dVector params;
	mp->writeParametersToList(params);
	getEqualityConstraintValues(params);
}

//and bound constraints for the parameters min <= p <= max
const dVector& QPControlConstraints::getBoundConstraintsMinValues() {
	qpPlan->getParameterMinValues(l);

	return l;
}

const dVector& QPControlConstraints::getBoundConstraintsMaxValues() {
	qpPlan->getParameterMaxValues(u);
	return u;
}

QPControlConstraints::~QPControlConstraints(void){
}

int QPControlConstraints::getEqualityConstraintCount() {
	int nEqConstraints = 0;
	FequalsMAConstraintsStartIndex = -1;
	eeNoSlipConstraintsStartIndex = -1;

	FequalsMAConstraintsStartIndex = nEqConstraints;
	nEqConstraints += qpPlan->robotRepresentation->getDimensionCount();

//	eeNoSlipConstraintsStartIndex = nEqConstraints;
//	nEqConstraints += qpPlan->contactEndEffectors.size() * 3;
	return nEqConstraints;
}

int QPControlConstraints::getInequalityConstraintCount() {
	//NOTE: friction constraints go here...
	int ineqCount = 0;

	// 4 constraints for each end effector
	for (uint i = 0; i < qpPlan->contactEndEffectors.size(); ++i)
		ineqCount += 4;

	return ineqCount;
}

void QPControlConstraints::addFequalsMAConstraints(int constraintStartIndex){
	if (constraintStartIndex < 0) return;

//	want Ma - J'f - [0 u]' = -C, 
//	where M is the generalized mass matrix, 
//	C is the vector that combines gravitational forces, coriolis and centrifugal terms...
//	a is the set of generalized accelerations and u is the set of generalized joint torques...
//	f is the set of ground reaction forces...
//	u are the interal joint torques...

	dVector Ma = qpPlan->M * qpPlan->a;

	int nDim = qpPlan->robotRepresentation->getDimensionCount();

	dVector Jtf;
	resize(Jtf, nDim);
	for (uint i = 0; i < qpPlan->contactEndEffectors.size(); i++)
		Jtf += qpPlan->contactEndEffectors[i].J.transpose() * qpPlan->contactEndEffectors[i].contactForce;

	//now, write out these values in the right place...
	for (int i = 0; i < nDim; i++){
		eqConstraintVals[i + constraintStartIndex] = Ma[i] - qpPlan->u[i] - Jtf[i];
		b[i + constraintStartIndex] = /*-qpPlan->C[i] +*/ qpPlan->gravitationalForces[i];
	}
}

void QPControlConstraints::addEndEffectorNoSlipConstraints(int constraintStartIndex) {
	return; // no longer in use...
/*
	if (constraintStartIndex < 0) return;

	//so: velocity of an end effector is J * qDot. Acceleration of this end effector is J * qDotDot + JDot * qDot. We want the foot not to move (or come to a stop?)
	//so then, for each end effector, we want J * qDotDot = - eps / dt * J * qDot - JDot * qDot such that it does not move...

	dVector qDot;
	qpPlan->robotRepresentation->getQDot(qDot);
	int nDim = qpPlan->robotRepresentation->getDimensionCount();

//	print("..\\out\\qDot.m", qDot);

	for (uint eeIndex = 0; eeIndex < qpPlan->contactEndEffectors.size(); eeIndex++){
		dVector J_qDotDot = qpPlan->contactEndEffectors[eeIndex].J * qpPlan->a;
		dVector JDot_qDot = qpPlan->contactEndEffectors[eeIndex].Jdot * qDot;

		dVector currentEEVel = qpPlan->contactEndEffectors[eeIndex].J * qDot;
//		Logger::consolePrint("vel for ee %d: %lf %lf %lf\n", eeIndex, currentEEVel[0], currentEEVel[1], currentEEVel[2]);

		//now, write out these values in the right place...
		for (int i = 0; i < 3; i++) {
			eqConstraintVals[constraintStartIndex] = J_qDotDot[i];
			b[constraintStartIndex] = -JDot_qDot[i]  - currentEEVel[i] / (qpPlan->dt) * 1.0;
			constraintStartIndex++;
		}
	}
*/
}

//get the actual value of the equality constraints...
const dVector& QPControlConstraints::getEqualityConstraintValues(const dVector& p){
	//first decide how many constraints there are
	int nEqConstraints = getEqualityConstraintCount();

	resize(eqConstraintVals, nEqConstraints);
	resize(b, nEqConstraints);

	qpPlan->setParametersFromList(p);
	
	addFequalsMAConstraints(FequalsMAConstraintsStartIndex);
//	addEndEffectorNoSlipConstraints(eeNoSlipConstraintsStartIndex);

	return eqConstraintVals;
}


// Inequality constraints
const dVector& QPControlConstraints::getInequalityConstraintValues(const dVector& p) {
	int nIneqConstraints = getInequalityConstraintCount();

	resize(ineqConstraintVals, nIneqConstraints);

	// d <= Cx <= f
	// for matlab this goes in a form Cx <= f
	resize(d, nIneqConstraints);
	for (int i = 0; i < nIneqConstraints; ++i)
		d[i] = -INFINITY;
	f.setZero(nIneqConstraints);

	qpPlan->setParametersFromList(p);

	int j = 0;
	for (uint i = 0; i < qpPlan->contactEndEffectors.size(); ++i) {
		double fx = p[qpPlan->groundReactionForcesParamsStartIndex + 3 * i + 0],
			fz = p[qpPlan->groundReactionForcesParamsStartIndex + 3 * i + 2];
		double ub = qpPlan->contactEndEffectors[i].frictionCoeff * p[qpPlan->groundReactionForcesParamsStartIndex + 3 * i + 1];

		// coeff * fy - fx
		ineqConstraintVals[j + 0] = (ub - fx) * -1;
		// coeff * fy + fx
		ineqConstraintVals[j + 1] = (ub + fx) * -1;
		// coeff * fy - fz
		ineqConstraintVals[j + 2] = (ub - fz) * -1;
		// coeff * fy + fz
		ineqConstraintVals[j + 3] = (ub + fz) * -1;

		j += 4;
	}
	// ineqConstraintVals.setZero (nIneqConstraints);
	return ineqConstraintVals;
}

/*!
*  evaluates the constraint jacobian
*/
/*
void QPControlConstraints::addEqualityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p) {
	//first decide how many constraints there are
	int nEqConstraints = getEqualityConstraintCount();

}
*/

