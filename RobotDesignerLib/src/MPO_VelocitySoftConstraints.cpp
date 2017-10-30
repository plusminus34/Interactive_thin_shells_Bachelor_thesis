#include <RobotDesignerLib/MPO_VelocitySoftConstraints.h>

#if 0

/*****************************************************************************************************
***************************** Lower bound on GRFs - want pushing-only forces *************************
******************************************************************************************************/


MPO_GRFRegularizer::MPO_GRFRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_GRFRegularizer::~MPO_GRFRegularizer(void){
}

double MPO_GRFRegularizer::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//theMotionPlan->setMPParametersFromList(s);


	double retVal = 0;
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		for (uint i=0;i<theMotionPlan->endEffectorTrajectories.size();i++){
			//we want the vertical component of the GRF to be > 0
			double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
			//if a GRF is not used/useful (e.g. the leg is in swing), then we should encourage a small value for it...
			retVal += 0.5 * theMotionPlan->endEffectorTrajectories[i].contactForce[j].length2() * (1 - c);
		}
	}

/*
	double val = -10;
	for (int i = 0; i < 1000; i++) {
		Logger::logPrint("%lf\t%lf\n", val, suc.computeValue(val));
		val += 0.1;
	}
	exit(0);
*/
	return retVal * weight;
}


void MPO_GRFRegularizer::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
//	theMotionPlan->setMPParametersFromList(p);


	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
				//we want the vertical component of the GRF to be > 0
				double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];


				for (int k = 0; k < 3;k++)
					grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k] += theMotionPlan->endEffectorTrajectories[i].contactForce[j][k] * (1-c) * weight;

			}
		}
}

void MPO_GRFRegularizer::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
				//we want the vertical component of the GRF to be > 0
				double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				for (int k = 0; k < 3; k++)
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k, (1-c), weight);
			}
		}
}

#endif // 0

/*****************************************************************************************************
 ***************************** Upper limit on GRFs ***************************************************
 *****************************************************************************************************/

MPO_VelocitySoftBoundConstraints::MPO_VelocitySoftBoundConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int startQIndex, int endQIndex) {

	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
	this->startQIndex = startQIndex;
	this->endQIndex = endQIndex;

	assert(theMotionPlan->jointVelocityLimit >= 0);
	constraintLowerBound = std::shared_ptr<SoftUnilateralConstraint>(new SoftUnilateralConstraint(-theMotionPlan->jointVelocityLimit, 10, theMotionPlan->jointVelocityEpsilon));
	constraintUpperBound = std::shared_ptr<SoftUnilateralConstraint>(new SoftUnilateralConstraint(theMotionPlan->jointVelocityLimit, 10, theMotionPlan->jointVelocityEpsilon));
}

MPO_VelocitySoftBoundConstraints::~MPO_VelocitySoftBoundConstraints(void) {
}

double MPO_VelocitySoftBoundConstraints::computeValue(const dVector& s) {

	double retVal = 0;

	int nSamplePoints = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

	double dt = theMotionPlan->motionPlanDuration / (double)nSamplePoints;
	dt = 1.0; // TODO: '1.0' or 'theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints' ?

	for (int j=0; j<nSamplePoints; j++){

		int jm, jp;

		theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
		if (jm == -1 || jp == -1) continue;

		double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;
		dt = 1.0;

		//we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
		for (int i=startQIndex; i<=endQIndex; i++){
			double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[jm][i]) / dt;
			retVal += constraintLowerBound->computeValue(velocity);
			retVal += constraintUpperBound->computeValue(velocity);
		}
	}

	return retVal * weight;
}

void MPO_VelocitySoftBoundConstraints::addGradientTo(dVector& grad, const dVector& p) {

	int nSamplePoints = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

	double dt = theMotionPlan->motionPlanDuration / (double)nSamplePoints;
	dt = 1.0; // TODO: '1.0' or 'theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints' ?

	for (int j=0; j<nSamplePoints; j++){

		int jm, jp;

		theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
		if (jm == -1 || jp == -1) continue;

		double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;
		dt = 1.0;

		//we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
		for (int i=startQIndex; i<=endQIndex; i++){
			double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[jm][i]) / dt;

			// grad = w * dc(velocity)/dx * dvelocity/dx

			// lower bound gradient
			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jm + i]
					-= weight * constraintLowerBound->computeDerivative(velocity) / dt;
			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jp + i]
					+= weight * constraintLowerBound->computeDerivative(velocity) / dt;

			// upper bound gradient
			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jm + i]
					-= weight * constraintUpperBound->computeDerivative(velocity) / dt;
			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jp + i]
					+= weight * constraintUpperBound->computeDerivative(velocity) / dt;		}
	}
}

void MPO_VelocitySoftBoundConstraints::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	int nSamplePoints = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

	double dt = theMotionPlan->motionPlanDuration / (double)nSamplePoints;
	dt = 1.0; // TODO: '1.0' or 'theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints' ?

	for (int j=0; j<nSamplePoints; j++){

		int jm, jp;

		theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
		if (jm == -1 || jp == -1) continue;

		double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;
		dt = 1.0;

		//we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
		for (int i=startQIndex; i<=endQIndex; i++){
			double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[jm][i]) / dt;

			if (theMotionPlan->robotStatesParamsStartIndex >= 0){

				// lower bound hessian
				// d_jm_jm
				addMTripletToList_reflectUpperElements(
							hessianEntries,
							theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
							theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
							weight * constraintLowerBound->computeSecondDerivative(velocity) / (dt*dt));
				// d_jp_jp
				addMTripletToList_reflectUpperElements(
							hessianEntries,
							theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
							theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
							weight * constraintLowerBound->computeSecondDerivative(velocity) / (dt*dt));
				// d_jm_jp
				addMTripletToList_reflectUpperElements(
							hessianEntries,
							theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
							theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
							- weight * constraintLowerBound->computeSecondDerivative(velocity) / (dt*dt));

				// upper bound hessian
				// d_jm_jm
				addMTripletToList_reflectUpperElements(
							hessianEntries,
							theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
							theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
							weight * constraintUpperBound->computeSecondDerivative(velocity) / (dt*dt));
				// d_jp_jp
				addMTripletToList_reflectUpperElements(
							hessianEntries,
							theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
							theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
							weight * constraintUpperBound->computeSecondDerivative(velocity) / (dt*dt));
				// d_jm_jp
				addMTripletToList_reflectUpperElements(
							hessianEntries,
							theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
							theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
							- weight * constraintUpperBound->computeSecondDerivative(velocity) / (dt*dt));
			}
		}
	}
}


