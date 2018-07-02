#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include "KS_MechanicalAssembly.h"
#include "KS_BindComponentsConnection.h"
#include "KS_MotorConnection.h"
#include <MathLib/SparseMatrix.h>
#include <Eigen/Sparse>

class KS_LinkageLagrangian : public ObjectiveFunction {
public:
	KS_LinkageLagrangian(KS_MechanicalAssembly* a, DynamicArray<double> &s, std::pair<Point3d, int> endEff, DynamicArray<Point3d> &c);
	virtual ~KS_LinkageLagrangian(void);

	void initialize(KS_BindComponentsConnection* p1, KS_BindComponentsConnection* p2, KS_MotorConnection* motor);

	//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of p.
	virtual void updateRegularizingSolutionToCurrentIterationBest(const dVector &currentP);
	virtual double computeValue(const dVector& p);
	virtual double computeValue(const double* p);

	virtual double angleSensitivity(const dVector &p);
	virtual void momentArmStateGradient(dVector &dM_ds, const dVector &p);
	virtual void angleSensitivityStateGradient(dVector &dE_ds, const dVector &s, const dVector &p);
	virtual void precomputeDerivativeInformationAt(const dVector &p);
	virtual SparseMatrix* getCurrentHessian();
	virtual dVector* getCurrentGradient();
	virtual Eigen::SparseMatrix<double>* getCurrentHessianEigen();

	void setRegularizer(double val);
	void setMeritFunctionTradeOff(double val){mu = val;}
	double getMeritFunctionTradeOff(){return mu;}
	int getObjectiveFunctionNumOfParameters(){return stateSize+paramSize;}

	void computeGradient(dVector& gradient, const dVector &p);
	double getValue(const dVector& p);
	void setState(int s, const dVector &p);

private:
	KS_MechanicalAssembly* assembly;
	std::vector<KS_Constraint*> constraints;

	DynamicArray<double> ticks;

	std::pair<Point3d, int> endEffector;
	DynamicArray<Point3d> curve;

	KS_BindComponentsConnection* pin1;
	KS_BindComponentsConnection* pin2;
	KS_MotorConnection* actuator;

	int assemblyConstraintSize, assemblyStateSize;
	int stateSize, paramSize, constraintSize;

	double wCurve, wArm, wAngle;
	double epsilon;
	double regularizer;
	double mu;

	//The total energy of the system is the sum of the individual energy terms of the constraint. The gradient of this scalar function is this
	dVector dE_ds;
	//and the hessian of the sum of the energy terms is a sparse matrix...
	SparseMatrix ddE_dsds;

	SparseMatrix dCds;

	Eigen::SparseMatrix<double, Eigen::ColMajor> Hessian;
};

