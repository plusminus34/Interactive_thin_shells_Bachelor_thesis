#include <OptimizationLib/SoftConstraints.h>
#include <OptimizationLib/ooqpei_assert_macros.hpp>
#include <iostream>
#include <Utils/Utils.h>

SoftConstraints::SoftConstraints() {

}

SoftConstraints::~SoftConstraints() {

}

// returns 1/2 C'C, where C is the current set of equality constraint values
double SoftConstraints::computeValue(const dVector& p) {
	return weight * 0.5 * getEqualityConstraintValues(p).squaredNorm();
}


