#pragma once
#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

using namespace std;

class MPO_TorqueAngularAccelObjective : public ObjectiveFunction {
public:
	MPO_TorqueAngularAccelObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_TorqueAngularAccelObjective(void);

	virtual double computeValue(const dVector& p);
#if 1
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
#endif
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	bool checkAngleAxisSingularity(Matrix3x3& R);

	void getdwdRFromRotationMatrix(Matrix3x3* dwdRArray, Matrix3x3& R);
	void getdwdRFromRotationMatrix(Matrix3x3* dwdRArray, Matrix3x3* dw00dRArray, Matrix3x3* dw01dRArray, Matrix3x3* dw02dRArray, Matrix3x3* dw12dRArray, Matrix3x3& R);
	void AngleAxisInnerHessianHelper(DynamicArray<MTriplet>& hessianEntries, Vector3d vec, int index1, int index2, Matrix3x3& dummy, Matrix3x3& R, Matrix3x3& R1, Matrix3x3& R2, Matrix3x3* A1, Matrix3x3* A2, Matrix3x3* crossMat, Matrix3x3& dw1, Matrix3x3& dw2, double scale);
	void AngleAxisOutterHessianHelper(DynamicArray<MTriplet>& hessianEntries, int jpp, int jp, int jm, int jmm, Matrix3x3& dw1, Matrix3x3& dw2, Matrix3x3& dw3, Matrix3x3& dw4, double scale);
	void TorqueInnerHessianHelper(DynamicArray<MTriplet>& hessianEntries, Vector3d vec, int j);
	void AngleAxisTorqueCrossHessianHelper(DynamicArray<MTriplet>& hessianEntries, int j, int jpp, int jp, int jm, int jmm, Matrix3x3& dw1, Matrix3x3& dw2, Matrix3x3& dw3, Matrix3x3& dw4, double scale);

	void getCrossProductTensor(Matrix3x3* tensor);
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;


	//matrices used to work around (near)singular configurations...
	vector<Matrix3x3> dummyR;
	vector<Matrix3x3> dummyQ;
public:
	void updateDummyMatrices();

};
