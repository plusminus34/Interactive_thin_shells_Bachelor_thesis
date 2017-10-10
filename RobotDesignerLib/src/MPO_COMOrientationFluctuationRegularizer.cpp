#include <RobotDesignerLib/MPO_COMOrientationFluctuationRegularizer.h>



MPO_COMOrientationFluctuationRegularizer::MPO_COMOrientationFluctuationRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_COMOrientationFluctuationRegularizer::~MPO_COMOrientationFluctuationRegularizer(void) {}

double MPO_COMOrientationFluctuationRegularizer::computeValue(const dVector& s) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	//we want to penalize sum of eulerangle differences
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) {		
		double val = 0;
		for (int j = 0; j < theMotionPlan->nSamplePoints-1; j++) {
			V3D orientationJplus(theMotionPlan->COMTrajectory.orientation[0][j + 1], theMotionPlan->COMTrajectory.orientation[1][j + 1], theMotionPlan->COMTrajectory.orientation[2][j + 1]);
			V3D orientationJ(theMotionPlan->COMTrajectory.orientation[0][j], theMotionPlan->COMTrajectory.orientation[1][j], theMotionPlan->COMTrajectory.orientation[2][j]);
			V3D diff = orientationJplus - orientationJ;
			val += diff.length2();
		}
		V3D orientationStart(theMotionPlan->COMTrajectory.orientation[0][0], theMotionPlan->COMTrajectory.orientation[1][0], theMotionPlan->COMTrajectory.orientation[2][0]);
		V3D orientationEnd(theMotionPlan->COMTrajectory.orientation[0][theMotionPlan->nSamplePoints - 1], theMotionPlan->COMTrajectory.orientation[1][theMotionPlan->nSamplePoints - 1], theMotionPlan->COMTrajectory.orientation[2][theMotionPlan->nSamplePoints - 1]);
		V3D diff = orientationStart - orientationEnd;
		val += diff.length2();
		return val * weight;
	}
	return 0;
}

void MPO_COMOrientationFluctuationRegularizer::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) {
		for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {

			V3D gradient;
			V3D orientationJplus(theMotionPlan->COMTrajectory.orientation[0][j + 1], theMotionPlan->COMTrajectory.orientation[1][j + 1], theMotionPlan->COMTrajectory.orientation[2][j + 1]);
			V3D orientationJ(theMotionPlan->COMTrajectory.orientation[0][j], theMotionPlan->COMTrajectory.orientation[1][j], theMotionPlan->COMTrajectory.orientation[2][j]);
			V3D orientationJminus(theMotionPlan->COMTrajectory.orientation[0][j - 1], theMotionPlan->COMTrajectory.orientation[1][j - 1], theMotionPlan->COMTrajectory.orientation[2][j - 1]);

			if (j == 0) {
				int endIndex = theMotionPlan->nSamplePoints-1;
				V3D orientationJend(theMotionPlan->COMTrajectory.orientation[0][endIndex], theMotionPlan->COMTrajectory.orientation[1][endIndex], theMotionPlan->COMTrajectory.orientation[2][endIndex]);
				gradient = (orientationJ * 2 - orientationJplus - orientationJend) * 2;
			}
			else if (j == theMotionPlan->nSamplePoints - 1) {
				V3D orientationJstart(theMotionPlan->COMTrajectory.orientation[0][0], theMotionPlan->COMTrajectory.orientation[1][0], theMotionPlan->COMTrajectory.orientation[2][0]);
				gradient = (orientationJ * 2 - orientationJminus - orientationJstart) * 2;
			}
			else {
				gradient = (orientationJ * 2 - orientationJplus - orientationJminus) * 2;
			}			


			if (theMotionPlan->COMOrientationsParamsStartIndex >= 0) {
				grad[theMotionPlan->COMOrientationsParamsStartIndex + 3 * j + 0] += gradient[0] * weight;
				grad[theMotionPlan->COMOrientationsParamsStartIndex + 3 * j + 1] += gradient[1] * weight;
				grad[theMotionPlan->COMOrientationsParamsStartIndex + 3 * j + 2] += gradient[2] * weight;
			}

		}				
	}

}

void MPO_COMOrientationFluctuationRegularizer::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	auto writeHessian3x3 = [this](DynamicArray<MTriplet>& list, int s, int t, Matrix3x3 mat, double w) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				ADD_HES_ELEMENT(list, s + i, t + j, mat(i, j), weight);
			}
		}
	};

	if (theMotionPlan->COMOrientationsParamsStartIndex >= 0) {
		for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
			Matrix3x3 I; I.setIdentity();
			int index = theMotionPlan->COMOrientationsParamsStartIndex + 3 * j;

			if (j == 0) {
				int indexend= theMotionPlan->COMOrientationsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints-1);
				writeHessian3x3(hessianEntries, index,		index,	 4 * I, weight);
				writeHessian3x3(hessianEntries, index +3,	index,	-2 * I, weight);
				writeHessian3x3(hessianEntries, indexend, index, -2 * I, weight);
			}
			else if (j == theMotionPlan->nSamplePoints - 1) {
				int indexstart = theMotionPlan->COMOrientationsParamsStartIndex + 3 * (0);
				writeHessian3x3(hessianEntries, index,		index,	4 * I, weight);
				//writeHessian3x3(hessianEntries, index -3,	index,	-2 * I, weight);
				writeHessian3x3(hessianEntries, indexstart, index, -2 * I, weight);
			}
			else {
				writeHessian3x3(hessianEntries, index,		index,	 4 * I, weight);
				writeHessian3x3(hessianEntries, index +3,	index,	-2 * I, weight);
				//writeHessian3x3(hessianEntries, index -3,	index,	-2 * I, weight);
			}		
		
		}
	}

}
