#pragma once

#include <math.h>
#include <MathLib/MathLib.h>
#include <MathLib/Matrix.h>

/*!
	A multi-dimensional function that expresses linear equality and inequality constraints applicable to an objective function:

	Equality constrains:    A(p) = b
	Inequality constraint:  d <= C(p) <= f
	Bound constraint:       l <= p <= u
*/
class FunctionConstraints {
public:
  /*! Constructor
   * Fill the constant variables in the constructor of the derived class:
   *  d, f, l, u
   */
	FunctionConstraints();
	virtual ~FunctionConstraints();

	virtual int getEqualityConstraintCount();
	virtual int getInequalityConstraintCount();

	/*! Derive this method and implement the analytic Jacobian of te equality constraints,
	 * otherwise it is estimated through finite-difference.
	 *
	 *  Equality constraints of the form A(p) = b (if they were really linear, A(p) = A*p)
	 *  Assume the constraints are either linear, or they should be linearized at p.
	 */
	virtual void addEqualityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p);

	/*! @returns b of A(p) = b
	 */
	virtual const dVector& getEqualityConstraintsTargetValues();

	/*! Derive this method and compute eqConstraintVals.
	 * Get the actual value of the equality constraints.
	 * @returns A(p) of A(p) = b
	 */
	virtual const dVector& getEqualityConstraintValues(const dVector& p);

	/*! inequality constraints of the form d <= C(p) <= f
	 * @returns dC/dp
	 */
	virtual void addInequalityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p);

  /*! @returns d of d <= C(p) <= f
   */
	virtual const dVector& getInequalityConstraintsMinValues();

  /*! @returns f of d <= C(p) <= f
   */
	virtual const dVector& getInequalityConstraintsMaxValues();

	/*! Derive this method and compute ineqConstraintVals.
	 * get the actual value of the equality constraints...
	 * @returns C(p)  of d <= C(p) <= f
	 */
	virtual const dVector& getInequalityConstraintValues(const dVector& p);

	/*! @returns min of constraint min <= p <= max
	 */
	virtual const dVector& getBoundConstraintsMinValues();

  /*! @returns max of constraint min <= p <= max
   */
	virtual const dVector& getBoundConstraintsMaxValues();

public:
  //!  Estimates the Jacobian of the equality constraints through finite-difference (FD)
  void addEstimatedEqualityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p);

	//!  Estimates the Jacobian of the inequality constraints through finite-difference (FD)
  void addEstimatedInequalityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p);

	//! Tests Jacobians of equality and inequality constraints
  void testJacobiansWithFD(const dVector& p);

  void printConstraintErrors(const dVector& p, double eqTol = 0.00001, double iqTol = 0.00001);

protected:
  dVector b;
  dVector d;
  dVector f;
  dVector l;
  dVector u;
  //! A(p)
  dVector eqConstraintVals;
  //! C(p)
  dVector ineqConstraintVals;
};
