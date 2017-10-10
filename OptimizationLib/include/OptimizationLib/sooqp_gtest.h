

#pragma once

#include <OptimizationLib/ooqpei_gtest_eigen.hpp>
#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/FunctionConstraints.h>
#include <MathLib/Matrix.h>
#include <string>

namespace sooqp {

static inline void test_objective_function_gradient(ObjectiveFunction* objective, const std::string& testName, int nParameters) {
  dVector p = dVector::Random(nParameters);
  dVector trueGradient(p.size()); trueGradient.setZero();
  objective->addGradientTo(trueGradient, p);
  dVector estimatedGradient(p.size()); estimatedGradient.setZero();
  objective->addEstimatedGradientTo(estimatedGradient, p);
  EXPECT_EQ(estimatedGradient.size(), trueGradient.size());
  ASSERT_DOUBLE_SPARSE_MX_EQ(estimatedGradient, trueGradient, 1e-2, std::string{"Gradient of test "} + testName);
}


static inline void test_objective_function_hessian(ObjectiveFunction* objective, const std::string& testName, int nParameters) {
  dVector p = dVector::Random(nParameters);

  SparseMatrix H;
  int N = p.size();
  resize(H, N, N);
  DynamicArray<MTriplet> hessianEntries;
  hessianEntries.clear();
  objective->addHessianEntriesTo(hessianEntries, p);
  H.setFromTriplets(hessianEntries.begin(), hessianEntries.end());
  SparseMatrix estimatedHessian;
  resize(estimatedHessian, N, N);
  hessianEntries.clear();
  objective->addEstimatedHessianEntriesTo(hessianEntries, p);
  estimatedHessian.setFromTriplets(hessianEntries.begin(), hessianEntries.end());
  ASSERT_DOUBLE_SPARSE_MX_EQ(estimatedHessian, H, 1e0, std::string{"Hessian of test "} + testName);
}

static inline void test_objective_function_lower_hessian(ObjectiveFunction* objective, const std::string& testName, int nParameters) {
  dVector p = dVector::Random(nParameters);
  SparseMatrix H;
  int N = p.size();
  resize(H, N, N);
  DynamicArray<MTriplet> hessianEntries;
  hessianEntries.clear();
  objective->addHessianEntriesTo(hessianEntries, p);
  H.setFromTriplets(hessianEntries.begin(), hessianEntries.end());
  SparseMatrix estimatedHessian;
  resize(estimatedHessian, N, N);
  hessianEntries.clear();
  objective->addEstimatedHessianEntriesTo(hessianEntries, p);
  estimatedHessian.setFromTriplets(hessianEntries.begin(), hessianEntries.end());
  SparseMatrix estimatedHessianLower = estimatedHessian.triangularView<Eigen::Lower>();
  SparseMatrix trueHessianLower = H.triangularView<Eigen::Lower>();
  ASSERT_DOUBLE_SPARSE_MX_EQ(estimatedHessianLower, trueHessianLower, 1e-2, std::string{"Lower Hessian of test "} + testName);
}

static inline void test_function_constraints_equality_jacobian(FunctionConstraints* constraints, const std::string& testName, int nParameters) {
  dVector p = dVector::Random(nParameters);
  SparseMatrix trueJacobian = constraints->getEqualityConstraintsJacobian(p);
  SparseMatrix estimatedJacobian;
  constraints->estimateEqualityConstraintsJacobianAt(estimatedJacobian,p);
  ASSERT_DOUBLE_SPARSE_MX_EQ(estimatedJacobian, trueJacobian, 1e-2, std::string{"Equality Constraints Jacobian of test "} + testName);
}

static inline void test_function_constraints_inequality_jacobian(FunctionConstraints* constraints, const std::string& testName, int nParameters) {
  dVector p = dVector::Random(nParameters);
  SparseMatrix trueJacobian = constraints->getInequalityConstraintsJacobian(p);
  SparseMatrix estimatedJacobian;
  constraints->estimateInequalityConstraintsJacobianAt(estimatedJacobian,p);
  ASSERT_DOUBLE_SPARSE_MX_EQ(estimatedJacobian, trueJacobian, 1e-2, std::string{"Inequality Constraints Jacobian of test "} + testName);
}

} //namespace sooqp
