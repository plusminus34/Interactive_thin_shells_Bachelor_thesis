#include <OptimizationLib/FunctionConstraints.h>
#include <OptimizationLib/ooqpei_assert_macros.hpp>
#include <iostream>
#include <Utils/Utils.h>

FunctionConstraints::FunctionConstraints() {

}

FunctionConstraints::~FunctionConstraints() {

}

const dVector& FunctionConstraints::getEqualityConstraintsTargetValues() {
	return b;
}

const dVector& FunctionConstraints::getEqualityConstraintValues(const dVector& p) {
	return eqConstraintVals;
}

void FunctionConstraints::addEqualityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p) {
	addEstimatedEqualityConstraintsJacobianEntriesTo(jacobianEntries, p);
}

void FunctionConstraints::addInequalityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p) {
	addEstimatedInequalityConstraintsJacobianEntriesTo(jacobianEntries, p);
}

const dVector& FunctionConstraints::getInequalityConstraintsMinValues() {
	return d;
}

const dVector& FunctionConstraints::getInequalityConstraintsMaxValues() {
	return f;
}

const dVector& FunctionConstraints::getInequalityConstraintValues(const dVector& p) {
	return ineqConstraintVals;
}

const dVector& FunctionConstraints::getBoundConstraintsMinValues() {
	return l;
}

const dVector& FunctionConstraints::getBoundConstraintsMaxValues() {
	return u;
}

int FunctionConstraints::getEqualityConstraintCount() {
	return (int)getEqualityConstraintsTargetValues().size();
}

int FunctionConstraints::getInequalityConstraintCount() {
	return (int)getInequalityConstraintsMinValues().size();
}


void FunctionConstraints::addEstimatedEqualityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& params) {
	dVector pSet = params;

	int nConstraints = getEqualityConstraintCount();
	int p = (int)pSet.size();
	//the jacobian should have dimensions nConstraints x p

	if (nConstraints > 0) {
		double dp = 10e-6;

		dVector C_P(nConstraints), C_M(nConstraints), J_i_col(nConstraints);
		//this is a very slow method that evaluates the jacobian of the objective function through FD...
		for (int i = 0; i<p; i++) {
			double tmpVal = pSet(i);
			pSet(i) = tmpVal + dp;

			C_P = getEqualityConstraintValues(pSet);

			pSet(i) = tmpVal - dp;
			C_M = getEqualityConstraintValues(pSet);

			//now reset the ith param value
			pSet(i) = tmpVal;
			J_i_col = (C_P - C_M) / (2.0 * dp);


			//each vector is a column vector of the hessian, so copy it in place...
			for (int j = 0;j<nConstraints;j++)
				if (!IS_ZERO(J_i_col(j)))
					jacobianEntries.push_back(MTriplet(j, i, J_i_col[j]));
		}
	}
}

void FunctionConstraints::addEstimatedInequalityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& params) {
	dVector pSet = params;

	int nConstraints = getInequalityConstraintCount();
	int p = (int)pSet.size();
	//the jacobian should have dimensions nConstraints x p

	if (nConstraints > 0) {
		double dp = 10e-6;

		dVector C_P(nConstraints), C_M(nConstraints), J_i_col(nConstraints);
		//this is a very slow method that evaluates the jacobian of the objective function through FD...
		for (int i = 0;i<p;i++) {
			double tmpVal = pSet(i);
			pSet(i) = tmpVal + dp;
			C_P = getInequalityConstraintValues(pSet);

			pSet(i) = tmpVal - dp;
			C_M = getInequalityConstraintValues(pSet);

			//now reset the ith param value
			pSet(i) = tmpVal;
			J_i_col = 1.0 / (2.0*dp)*C_P + -1.0 / (2.0*dp)*C_M;

			//each vector is a column vector of the hessian, so copy it in place...
			for (int j = 0;j<nConstraints;j++)
				if (!IS_ZERO(J_i_col(j)))
					jacobianEntries.push_back(MTriplet(j, i, J_i_col[j]));
		}
	}
}

void FunctionConstraints::testJacobiansWithFD(const dVector& p) {
	DynamicArray<MTriplet> jacobianEntries;

	SparseMatrix FDJacobian;
	addEstimatedEqualityConstraintsJacobianEntriesTo(jacobianEntries, p);
	resize(FDJacobian, getEqualityConstraintCount(), p.size());
	FDJacobian.setFromTriplets(jacobianEntries.begin(), jacobianEntries.end());

	jacobianEntries.clear();

	SparseMatrix analyticJacobian;
	addEqualityConstraintsJacobianEntriesTo(jacobianEntries, p);
	resize(analyticJacobian, getEqualityConstraintCount(), p.size());
	analyticJacobian.setFromTriplets(jacobianEntries.begin(), jacobianEntries.end());

	jacobianEntries.clear();

	Logger::logPrint("Function Constraints: testing equality constraints jacobian...\n");
	for (int i = 0;i<FDJacobian.rows();i++) {
		for (int j = 0;j<p.size();j++) {
			double err = FDJacobian.coeff(i, j) - analyticJacobian.coeff(i, j);
			if (fabs(err) > 0.001)
				Logger::logPrint("Mismatch element %d,%d: Analytic val: %lf, FD val: %lf. Error: %lf\n", i, j, analyticJacobian.coeff(i, j), FDJacobian.coeff(i, j), err);
		}
	}

	addEstimatedInequalityConstraintsJacobianEntriesTo(jacobianEntries, p);
	resize(FDJacobian, getInequalityConstraintCount(), p.size());
	FDJacobian.setFromTriplets(jacobianEntries.begin(), jacobianEntries.end());

	jacobianEntries.clear();

	addInequalityConstraintsJacobianEntriesTo(jacobianEntries, p);
	resize(analyticJacobian, getInequalityConstraintCount(), p.size());
	analyticJacobian.setFromTriplets(jacobianEntries.begin(), jacobianEntries.end());

	jacobianEntries.clear();

	Logger::logPrint("Function Constraints: testing inequality constraints jacobian...\n");
	for (int i = 0;i<FDJacobian.rows();i++) {
		for (int j = 0;j<p.size();j++) {
			double err = FDJacobian.coeff(i, j) - analyticJacobian.coeff(i, j);
			if (fabs(err) > 0.001)
				Logger::logPrint("Mismatch element %d,%d: Analytic val: %lf, FD val: %lf. Error: %lf\n", i, j, analyticJacobian.coeff(i, j), FDJacobian.coeff(i, j), err);
		}
	}
}

void FunctionConstraints::printConstraintErrors(const dVector& p, double eqTol, double iqTol){
	dVector de = getEqualityConstraintValues(p);
	if (de.size() > 0) {
		dVector dtargets = getEqualityConstraintsTargetValues();
		dVector::Index maxIndex;
		double maxError = (de - dtargets).cwiseAbs().maxCoeff(&maxIndex);
		if (maxError > eqTol) {
			Logger::logPrint("-----> Max equality constraint error: %10.10lf at index %d\n", maxError, maxIndex);
		}
		else {
			Logger::logPrint("   Equality constraints are within the tolerance.\n");
		}
	}

	de = getInequalityConstraintValues(p);
	if (de.size() > 0) {
		dVector::Index maxIndexMin;
		dVector::Index maxIndexMax;
		double maxErrorMin = (-de + getInequalityConstraintsMinValues()).maxCoeff(&maxIndexMin);
		double maxErrorMax = (de - getInequalityConstraintsMaxValues()).maxCoeff(&maxIndexMax);
		dVector::Index maxIndex;
		double maxError = 0;
		if (maxErrorMin > maxErrorMax) {
			maxError = maxErrorMin;
			maxIndex = maxIndexMin;
		}
		else {
			maxError = maxErrorMax;
			maxIndex = maxIndexMax;
		}
		if (maxError > iqTol) {
			Logger::logPrint("------> Max inequality constraint error: %10.10lf at index %d (%lf < %lf < %lf) \n", maxError, maxIndex, getInequalityConstraintsMinValues()(maxIndex),de(maxIndex), getInequalityConstraintsMaxValues()(maxIndex));
		}
		else {
			Logger::logPrint("   Inequality constraints are within the tolerance.\n");
		}
	}


	dVector minVals = getBoundConstraintsMinValues();
	dVector maxVals = getBoundConstraintsMaxValues();

	if (minVals.size() == maxVals.size() && minVals.size() == p.size()) {
		for (int i = 0;i<p.size();i++) {
			if (minVals(i) != maxVals(i) && (minVals(i)>p(i) || p[i]>maxVals(i))) {
				Logger::logPrint("-------> Error: Bound %d: %lf < %lf < %lf\n", i, minVals(i), p(i), maxVals(i));
			}
		}
	}

	Logger::logPrint("\n");
}

