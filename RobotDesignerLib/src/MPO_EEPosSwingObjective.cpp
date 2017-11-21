#include <RobotDesignerLib/MPO_EEPosSwingObjective.h>
#include <MathLib/AutoDiff.h>

MPO_EEPosSwingObjective::MPO_EEPosSwingObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_EEPosSwingObjective::~MPO_EEPosSwingObjective(void){
}

double MPO_EEPosSwingObjective::computeValue(const dVector& p){
	// assume the parameters of the motion plan have been set already by the collection of objective functions class
	// theMotionPlan->setMPParametersFromList(p);

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	double retVal = 0;
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		for (uint i=0;i<theMotionPlan->endEffectorTrajectories.size();i++){
			const DynamicArray<double> &targetEEPosY = theMotionPlan->endEffectorTrajectories[i].targetEEPosY;
			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j-1];
			double eePosY = theMotionPlan->endEffectorTrajectories[i].EEPos[j](1);
			// p[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 1];

			retVal += computeEnergy(eePosY, targetEEPosY[j], c);
		}
	}
	return retVal * weight;
}

void MPO_EEPosSwingObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if(theMotionPlan->feetPositionsParamsStartIndex >= 0)
	{
		typedef AutoDiffT<double, double> ScalarDiff;

		//and now compute the gradient with respect c and eePos
		int nLimbs = theMotionPlan->endEffectorTrajectories.size();
		for (int j=0; j<theMotionPlan->nSamplePoints; j++){
			for (int i=0; i<nLimbs; i++){
				const DynamicArray<double> &targetEEPosY = theMotionPlan->endEffectorTrajectories[i].targetEEPosY;
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
				double eePosY = p[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 1];

				ScalarDiff eePosYAd(eePosY, 1.0);
				ScalarDiff targetEEPosYAd(targetEEPosY[j], 0.0);
				ScalarDiff cAd(c, 0.0);

				ScalarDiff energy = computeEnergy(eePosYAd, targetEEPosYAd, cAd);
				grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 1] += energy.deriv() * weight;
			}
		}
	}
}

void MPO_EEPosSwingObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->feetPositionsParamsStartIndex >= 0){

		typedef AutoDiffT<double, double> ScalarDiff;
		typedef AutoDiffT<ScalarDiff, ScalarDiff> ScalarDiffDiff;

		int nLimbs = theMotionPlan->endEffectorTrajectories.size();
		for (int j=0;j<theMotionPlan->nSamplePoints;j++){
			for (int i=0;i<nLimbs;i++){
				const DynamicArray<double> &targetEEPosY = theMotionPlan->endEffectorTrajectories[i].targetEEPosY;
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
				double eePosY = p[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 1];

				ScalarDiffDiff eePosYAd(ScalarDiff(eePosY, 1), ScalarDiff(1, 0));
				ScalarDiffDiff targetEEPosYAd(ScalarDiff(targetEEPosY[j], 0), ScalarDiff(0, 0));
				ScalarDiffDiff cAd(ScalarDiff(c, 0), ScalarDiff(0, 0));

				ScalarDiffDiff energy = computeEnergy(eePosYAd, targetEEPosYAd, cAd);
				ADD_HES_ELEMENT(hessianEntries,
								theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 1,
								theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 1,
								energy.deriv().deriv(), weight);
			}
		}
	}
}

template<class T>
T MPO_EEPosSwingObjective::computeEnergy(const T &eePosY, const T &targetEEPosY, const T &c)
{
	T d = eePosY - targetEEPosY;
	return (T)0.5 * d*d * ((T)1-c);
}
